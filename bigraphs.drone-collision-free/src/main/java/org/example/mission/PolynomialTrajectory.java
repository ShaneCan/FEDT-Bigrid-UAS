package org.example.mission;

import java.util.*;

/** Septic Hermite segments, represented as Bezier control points for certification. */
public final class PolynomialTrajectory {
  public record Crossing(double at, int from, int to, Vec3 point, Vec3 normal) {}

  public record Piece(double duration, Vec3[] control, int from, int to) {
    public Piece {
      control = control.clone();
    }

    public Vec3[] control() {
      return control.clone();
    }

    public Vec3 at(double u) {
      Vec3[] q = control.clone();
      for (int n = 7; n > 0; n--)
        for (int i = 0; i < n; i++) q[i] = q[i].mul(1 - u).add(q[i + 1].mul(u));
      return q[0];
    }

    public double speedBound() {
      Vec3[] d = new Vec3[7];
      for (int i = 0; i < 7; i++) d[i] = control[i + 1].sub(control[i]).mul(7 / duration);
      return derivativeBound(d, 0);
    }

    public double accelerationBound() {
      Vec3[] d = new Vec3[6];
      for (int i = 0; i < 6; i++)
        d[i] =
            control[i + 2]
                .sub(control[i + 1].mul(2))
                .add(control[i])
                .mul(42 / (duration * duration));
      return derivativeBound(d, 0);
    }

    public Map<String, Object> json() {
      Map<String, Object> m = new LinkedHashMap<>();
      m.put("duration", duration);
      for (int a = 0; a < 3; a++) {
        double[] power = new double[8];
        for (int k = 0; k < 8; k++)
          for (int i = 0; i <= k; i++)
            power[k] +=
                choose(7, k)
                    * choose(k, i)
                    * ((k - i) % 2 == 0 ? 1 : -1)
                    * control[i].axis(a)
                    / Math.pow(duration, k);
        m.put(a == 0 ? "poly_x" : a == 1 ? "poly_y" : "poly_z", power);
      }
      m.put("poly_yaw", new double[] {0});
      return m;
    }
  }

  public final List<Piece> pieces;
  public final List<Integer> path;
  public final Set<Integer> initialCells;
  public final double slot;

  public PolynomialTrajectory(
      GridGraph g,
      List<Integer> path,
      Vec3 actualStart,
      double slot,
      double maxSpeed,
      double acceleration,
      double margin) {
    this.path = List.copyOf(path);
    this.initialCells = g.footprint(actualStart, margin);
    this.slot = slot;
    List<Vec3> points = new ArrayList<>();
    for (int c : path) points.add(g.center(c));
    points.set(0, actualStart);
    if (g.cell(actualStart) != path.getFirst())
      throw new IllegalArgumentException("Start lies outside planned first cell");
    List<Piece> result = null;
    // Reducing tangents changes geometry only. Timing remains fixed for CBS.
    for (double smoothing : new double[] {.8, .4, .15, 0}) {
      Vec3[] v = new Vec3[points.size()];
      Arrays.fill(v, Vec3.ZERO);
      for (int i = 1; i < v.length - 1; i++)
        if (!path.get(i - 1).equals(path.get(i)) && !path.get(i).equals(path.get(i + 1)))
          v[i] = points.get(i + 1).sub(points.get(i - 1)).mul(smoothing / (2 * slot));
      // A diagonal must cross its two cell faces together. Keep straight
      // diagonal runs moving, and stop only at a diagonal direction change.
      for (int i = 1; i < v.length - 1; i++) {
        int a = path.get(i - 1), b = path.get(i), c = path.get(i + 1);
        boolean diagonal =
            (g.x(a) != g.x(b) && g.y(a) != g.y(b)) || (g.x(b) != g.x(c) && g.y(b) != g.y(c));
        Vec3 in = points.get(i).sub(points.get(i - 1)), out = points.get(i + 1).sub(points.get(i));
        if (diagonal && in.dot(out) < in.norm() * out.norm() - 1e-9) v[i] = Vec3.ZERO;
      }
      result = new ArrayList<>();
      boolean safe = true;
      for (int i = 0; i < points.size() - 1; i++) {
        int a = path.get(i), b = path.get(i + 1);
        if (!g.legal(a, b)) throw new IllegalArgumentException("Trajectory uses an illegal edge");
        Vec3 p = points.get(i), q = points.get(i + 1);
        Vec3[] control = new Vec3[8];
        for (int j = 0; j < 4; j++) {
          control[j] = p.add(v[i].mul(j * slot / 7));
          control[7 - j] = q.sub(v[i + 1].mul(j * slot / 7));
        }
        Piece piece = new Piece(slot, control, a, b);
        if (!contained(piece.control, initialCorridor(g, piece, i), margin, 0)
            || !traceValid(g, piece)
            || piece.speedBound() > maxSpeed + 1e-8
            || piece.accelerationBound() > acceleration + 1e-8) {
          safe = false;
          break;
        }
        result.add(piece);
      }
      if (safe) break;
      result = null;
    }
    if (result == null)
      throw new IllegalArgumentException(
          "Cannot certify trajectory inside its grid corridor and motion limits");
    if (result.isEmpty())
      result =
          List.of(
              new Piece(
                  slot,
                  new Vec3[] {
                    actualStart,
                    actualStart,
                    actualStart,
                    actualStart,
                    actualStart,
                    actualStart,
                    actualStart,
                    actualStart
                  },
                  path.getFirst(),
                  path.getFirst()));
    pieces = List.copyOf(result);
    if (!certifyBraking(g, margin, maxSpeed, .5, .1))
      throw new IllegalArgumentException(
          "Cannot certify the trajectory braking envelope inside its planned grid corridor");
  }

  private List<GridGraph.Box> initialCorridor(GridGraph g, Piece p, int index) {
    Set<Integer> cells = new HashSet<>(g.corridor(p.from, p.to));
    if (index == 0) cells.addAll(initialCells);
    return cells.stream().map(g::box).toList();
  }

  public boolean certifyBraking(
      GridGraph g, double margin, double maxSpeed, double brakeAcceleration, double latency) {
    double factor = Math.max(.2, 1.5 * maxSpeed / brakeAcceleration) / 2 + latency;
    for (int k = 0; k < pieces.size(); k++) {
      Piece p = pieces.get(k);
      Set<Integer> cells = new HashSet<>();
      if (k == 0) cells.addAll(initialCells);
      for (int j = Math.max(0, k - 1); j <= Math.min(pieces.size() - 1, k + 1); j++)
        cells.addAll(g.corridor(pieces.get(j).from, pieces.get(j).to));
      Vec3[] velocity = new Vec3[7], end = new Vec3[8];
      for (int i = 0; i < 7; i++)
        velocity[i] = p.control[i + 1].sub(p.control[i]).mul(7 / p.duration);
      for (int i = 0; i < 8; i++) {
        Vec3 elevated =
            i == 0
                ? velocity[0]
                : i == 7
                    ? velocity[6]
                    : velocity[i - 1].mul(i / 7.0).add(velocity[i].mul(1 - i / 7.0));
        end[i] = p.control[i].add(elevated.mul(factor));
      }
      if (!brakeContained(p.control, end, cells.stream().map(g::box).toList(), margin, 0))
        return false;
    }
    return true;
  }

  private static boolean brakeContained(
      Vec3[] start, Vec3[] end, List<GridGraph.Box> boxes, double margin, int depth) {
    Vec3[] both = new Vec3[16];
    System.arraycopy(start, 0, both, 0, 8);
    System.arraycopy(end, 0, both, 8, 8);
    if (covered(bounds(both, margin), boxes)) return true;
    if (depth == 16) return false;
    Vec3[][] a = split(start), b = split(end);
    return brakeContained(a[0], b[0], boxes, margin, depth + 1)
        && brakeContained(a[1], b[1], boxes, margin, depth + 1);
  }

  public double duration() {
    return pieces.size() * slot;
  }

  public double stationarySeconds() {
    return pieces.stream()
        .filter(p -> Arrays.stream(p.control).allMatch(v -> v.distance(p.control[0]) < 1e-9))
        .mapToDouble(Piece::duration)
        .sum();
  }

  public int interiorStops() {
    int stops = 0;
    for (int i = 0; i < pieces.size() - 1; i++) {
      var p = pieces.get(i);
      if (p.control[7].sub(p.control[6]).norm() * 7 / p.duration < 1e-6) stops++;
    }
    return stops;
  }

  public Vec3 at(double time) {
    int i = Math.min(pieces.size() - 1, Math.max(0, (int) (time / slot)));
    return pieces.get(i).at(Math.max(0, Math.min(1, (time - i * slot) / slot)));
  }

  public List<Crossing> crossings(GridGraph g) {
    List<Crossing> result = new ArrayList<>();
    double offset = 0;
    for (Piece piece : pieces) {
      for (Crossing c : trace(g, piece))
        result.add(new Crossing(c.at + offset, c.from, c.to, c.point, c.normal));
      offset += piece.duration;
    }
    return List.copyOf(result);
  }

  private static boolean traceValid(GridGraph g, Piece p) {
    try {
      List<Crossing> trace = trace(g, p);
      return p.from == p.to
          ? trace.isEmpty()
          : trace.size() == 1 && trace.getFirst().from == p.from && trace.getFirst().to == p.to;
    } catch (IllegalArgumentException e) {
      return false;
    }
  }

  private static List<Crossing> trace(GridGraph g, Piece p) {
    TreeSet<Double> cuts = new TreeSet<>();
    cuts.add(0.0);
    cuts.add(1.0);
    for (int axis = 0; axis < 3; axis++) {
      int count = axis == 0 ? g.cols : axis == 1 ? g.rows : g.layers;
      for (int i = 0; i < count - 1; i++) {
        double plane = g.origin.axis(axis) + (axis == 2 ? i + 1 : i + .5) * g.step.axis(axis);
        double[] values = new double[8];
        for (int j = 0; j < 8; j++) values[j] = p.control[j].axis(axis) - plane;
        roots(values, 0, 1, cuts, 0);
      }
    }
    List<Double> merged = new ArrayList<>();
    for (double t : cuts) if (merged.isEmpty() || t - merged.getLast() > 1e-6) merged.add(t);
    if (merged.getLast() < 1 - 1e-6) merged.add(1.0);
    List<Crossing> result = new ArrayList<>();
    int previous = p.from;
    for (int i = 1; i < merged.size(); i++) {
      int cell = g.cell(p.at((merged.get(i - 1) + merged.get(i)) / 2));
      if (cell != previous) {
        if (!g.legal(previous, cell))
          throw new IllegalArgumentException("Smoothed curve crosses a missing network edge");
        Vec3 delta = g.center(cell).sub(g.center(previous));
        result.add(
            new Crossing(
                merged.get(i - 1) * p.duration,
                previous,
                cell,
                p.at(merged.get(i - 1)),
                delta.mul(1 / delta.norm())));
        previous = cell;
      }
    }
    if (previous != p.to)
      throw new IllegalArgumentException("Smoothed grid trace misses planned endpoint");
    return result;
  }

  private static void roots(double[] v, double lo, double hi, Set<Double> out, int depth) {
    double min = Arrays.stream(v).min().orElse(0), max = Arrays.stream(v).max().orElse(0);
    if (min > 1e-11 || max < -1e-11 || Math.max(Math.abs(min), Math.abs(max)) < 1e-11) return;
    if (depth == 30 || hi - lo < 1e-9) {
      out.add((lo + hi) / 2);
      return;
    }
    double[] q = v.clone(), a = new double[v.length], b = new double[v.length];
    a[0] = q[0];
    b[v.length - 1] = q[v.length - 1];
    for (int k = 1; k < v.length; k++) {
      for (int i = 0; i < v.length - k; i++) q[i] = (q[i] + q[i + 1]) / 2;
      a[k] = q[0];
      b[v.length - 1 - k] = q[v.length - 1 - k];
    }
    roots(a, lo, (lo + hi) / 2, out, depth + 1);
    roots(b, (lo + hi) / 2, hi, out, depth + 1);
  }

  public List<Map<String, Object>> json() {
    return pieces.stream().map(Piece::json).toList();
  }

  private static double choose(int n, int k) {
    double c = 1;
    for (int i = 1; i <= k; i++) c = c * (n - i + 1) / i;
    return c;
  }

  private static double derivativeBound(Vec3[] p, int depth) {
    double max = Arrays.stream(p).mapToDouble(Vec3::norm).max().orElse(0);
    if (depth == 6) return max;
    Vec3[][] s = split(p);
    return Math.max(derivativeBound(s[0], depth + 1), derivativeBound(s[1], depth + 1));
  }

  public static Vec3[][] split(Vec3[] p) {
    int n = p.length;
    Vec3[] q = p.clone(), l = new Vec3[n], r = new Vec3[n];
    l[0] = q[0];
    r[n - 1] = q[n - 1];
    for (int k = 1; k < n; k++) {
      for (int i = 0; i < n - k; i++) q[i] = q[i].add(q[i + 1]).mul(.5);
      l[k] = q[0];
      r[n - 1 - k] = q[n - 1 - k];
    }
    return new Vec3[][] {l, r};
  }

  private static GridGraph.Box bounds(Vec3[] p, double margin) {
    double[] lo = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY},
        hi = {-lo[0], -lo[1], -lo[2]};
    for (Vec3 v : p)
      for (int a = 0; a < 3; a++) {
        lo[a] = Math.min(lo[a], v.axis(a) - margin);
        hi[a] = Math.max(hi[a], v.axis(a) + margin);
      }
    return new GridGraph.Box(new Vec3(lo[0], lo[1], lo[2]), new Vec3(hi[0], hi[1], hi[2]));
  }

  public static boolean covered(GridGraph.Box b, List<GridGraph.Box> boxes) {
    List<List<Double>> cuts = new ArrayList<>();
    for (int a = 0; a < 3; a++) {
      TreeSet<Double> s = new TreeSet<>();
      s.add(b.lower().axis(a));
      s.add(b.upper().axis(a));
      for (var box : boxes)
        for (double v : new double[] {box.lower().axis(a), box.upper().axis(a)})
          if (v > b.lower().axis(a) && v < b.upper().axis(a)) s.add(v);
      List<Double> values = new ArrayList<>(s), mid = new ArrayList<>();
      for (int i = 1; i < values.size(); i++) mid.add((values.get(i - 1) + values.get(i)) / 2);
      if (mid.isEmpty()) mid.add(values.getFirst());
      cuts.add(mid);
    }
    for (double x : cuts.get(0))
      for (double y : cuts.get(1))
        for (double z : cuts.get(2))
          if (boxes.stream().noneMatch(box -> box.contains(new Vec3(x, y, z), 0))) return false;
    return true;
  }

  public static boolean certifyCorridor(GridGraph g, Piece p, double margin) {
    return contained(p.control, g.corridor(p.from, p.to).stream().map(g::box).toList(), margin, 0);
  }

  private static boolean contained(Vec3[] p, List<GridGraph.Box> boxes, double margin, int depth) {
    if (covered(bounds(p, margin), boxes)) return true;
    if (depth == 16) return false;
    Vec3[][] s = split(p);
    return contained(s[0], boxes, margin, depth + 1) && contained(s[1], boxes, margin, depth + 1);
  }

  /** A box lower bound on relative Bezier positions proves continuous separation. */
  public static boolean separated(Vec3[] a, Vec3[] b, double distance, int depth) {
    Vec3[] d = new Vec3[a.length];
    for (int i = 0; i < a.length; i++) d[i] = a[i].sub(b[i]);
    GridGraph.Box box = bounds(d, 0);
    double lower = 0;
    for (int axis = 0; axis < 3; axis++) {
      double lo = box.lower().axis(axis), hi = box.upper().axis(axis);
      lower += lo > 0 ? lo * lo : hi < 0 ? hi * hi : 0;
    }
    if (lower >= distance * distance - 1e-10) return true;
    if (depth == 16) return false;
    Vec3[][] sa = split(a), sb = split(b);
    return separated(sa[0], sb[0], distance, depth + 1)
        && separated(sa[1], sb[1], distance, depth + 1);
  }
}
