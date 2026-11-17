package org.example.mission;

import java.util.*;

/** Geometry and directed topology. Cell numbering matches the provider's site indices. */
public final class GridGraph {
  public record Box(Vec3 lower, Vec3 upper) {
    public boolean contains(Vec3 p, double margin) {
      for (int a = 0; a < 3; a++)
        if (p.axis(a) < lower.axis(a) + margin - 1e-9 || p.axis(a) > upper.axis(a) - margin + 1e-9)
          return false;
      return true;
    }

    public boolean contains(Box b) {
      return contains(b.lower, 0) && contains(b.upper, 0);
    }
  }

  public record Edge(int from, int to, String route) {}

  public final int rows, cols, layers;
  public final Vec3 origin, step;
  public final Set<Integer> blocked;
  private final Map<Integer, List<Edge>> edges;

  public GridGraph(
      int rows,
      int cols,
      int layers,
      Vec3 origin,
      Vec3 step,
      Set<Integer> blocked,
      Collection<Edge> links) {
    if (rows < 1 || cols < 1 || layers < 1 || Math.min(step.x(), Math.min(step.y(), step.z())) <= 0)
      throw new IllegalArgumentException("Invalid grid geometry");
    this.rows = rows;
    this.cols = cols;
    this.layers = layers;
    this.origin = origin;
    this.step = step;
    this.blocked = Set.copyOf(blocked);
    edges = new HashMap<>();
    for (int c : blocked) requireCell(c);
    for (Edge e : links) {
      requireCell(e.from);
      requireCell(e.to);
      int dx = Math.abs(x(e.from) - x(e.to)),
          dy = Math.abs(y(e.from) - y(e.to)),
          dz = Math.abs(z(e.from) - z(e.to));
      if (!((dz == 0 && Math.max(dx, dy) == 1) || (dz == 1 && dx + dy == 0)))
        throw new IllegalArgumentException("Off-network link: " + e);
      if (!route(e.from, e.to).equals(e.route))
        throw new IllegalArgumentException("Direction disagrees with geometry: " + e);
      edges.computeIfAbsent(e.from, k -> new ArrayList<>()).add(e);
    }
    edges.replaceAll((k, v) -> v.stream().sorted(Comparator.comparingInt(Edge::to)).toList());
  }

  public int size() {
    return rows * cols * layers;
  }

  public int x(int c) {
    return c % (rows * cols) / rows;
  }

  public int y(int c) {
    return c % rows;
  }

  public int z(int c) {
    return c / (rows * cols);
  }

  public int index(int x, int y, int z) {
    if (x < 0 || x >= cols || y < 0 || y >= rows || z < 0 || z >= layers)
      throw new IllegalArgumentException("Position is outside the grid");
    return z * rows * cols + x * rows + y;
  }

  public void requireCell(int c) {
    if (c < 0 || c >= size()) throw new IllegalArgumentException("Invalid grid cell " + c);
  }

  public Vec3 center(int c) {
    requireCell(c);
    return new Vec3(
        origin.x() + x(c) * step.x(),
        origin.y() + y(c) * step.y(),
        origin.z() + (z(c) + .5) * step.z());
  }

  public Box box(int c) {
    Vec3 p = center(c), half = step.mul(.5);
    return new Box(p.sub(half), p.add(half));
  }

  public int cell(Vec3 p) {
    return index(
        (int) Math.floor((p.x() - origin.x()) / step.x() + .5),
        (int) Math.floor((p.y() - origin.y()) / step.y() + .5),
        (int) Math.floor((p.z() - origin.z()) / step.z()));
  }

  public Set<Integer> footprint(Vec3 position, double margin) {
    int lo = cell(position.sub(new Vec3(margin - 1e-9, margin - 1e-9, margin - 1e-9)));
    int hi = cell(position.add(new Vec3(margin - 1e-9, margin - 1e-9, margin - 1e-9)));
    Set<Integer> cells = new HashSet<>();
    for (int x = x(lo); x <= x(hi); x++)
      for (int y = y(lo); y <= y(hi); y++)
        for (int z = z(lo); z <= z(hi); z++) cells.add(index(x, y, z));
    if (!Collections.disjoint(cells, blocked))
      throw new IllegalArgumentException("Initial safety envelope intersects a blocked cell");
    return Set.copyOf(cells);
  }

  public List<Edge> outgoing(int c) {
    return edges.getOrDefault(c, List.of()).stream().filter(e -> legal(e.from, e.to)).toList();
  }

  public Collection<Edge> links() {
    return edges.values().stream().flatMap(Collection::stream)
        .sorted(Comparator.comparingInt(Edge::from).thenComparingInt(Edge::to).thenComparing(Edge::route)).toList();
  }

  public boolean legal(int a, int b) {
    return !blocked.contains(a)
        && !blocked.contains(b)
        && (a == b || edges.getOrDefault(a, List.of()).stream().anyMatch(e -> e.to == b))
        && support(a, b).stream().noneMatch(blocked::contains);
  }

  public Set<Integer> support(int a, int b) {
    if (x(a) != x(b) && y(a) != y(b))
      return Set.of(index(x(a), y(b), z(a)), index(x(b), y(a), z(a)));
    return Set.of();
  }

  public Set<Integer> corridor(int a, int b) {
    Set<Integer> s = new HashSet<>(support(a, b));
    s.add(a);
    s.add(b);
    return Set.copyOf(s);
  }

  public String route(int a, int b) {
    int dx = x(b) - x(a), dy = y(b) - y(a), dz = z(b) - z(a);
    if (dz != 0) return dz > 0 ? "UpRoute" : "DownRoute";
    return (dx > 0 ? "Forward" : dx < 0 ? "Back" : "")
        + (dy > 0 ? "Left" : dy < 0 ? "Right" : "")
        + "Route";
  }

  /** Test fixture only; production topology always comes from the world service. */
  public static GridGraph rectangular(int rows, int cols, int layers, Set<Integer> blocked) {
    List<Edge> links = new ArrayList<>();
    GridGraph g =
        new GridGraph(rows, cols, layers, Vec3.ZERO, new Vec3(.5, .5, .6), blocked, List.of());
    for (int a = 0; a < g.size(); a++)
      for (int b = 0; b < g.size(); b++) {
        int dx = Math.abs(g.x(a) - g.x(b)),
            dy = Math.abs(g.y(a) - g.y(b)),
            dz = Math.abs(g.z(a) - g.z(b));
        if ((dz == 0 && Math.max(dx, dy) == 1) || (dz == 1 && dx + dy == 0))
          links.add(new Edge(a, b, g.route(a, b)));
      }
    return new GridGraph(rows, cols, layers, g.origin, g.step, blocked, links);
  }
}
