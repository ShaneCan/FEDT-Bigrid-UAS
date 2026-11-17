package org.example.mission;

import java.util.*;

/** Time-expanded A* with conflict-based search. No partial or unchecked fallback. */
public final class CooperativePlanner {
  public record Agent(int id, int start, int goal, Vec3 position) {}

  public record Plan(Map<Integer, PolynomialTrajectory> trajectories, double slot, int expanded) {}

  public interface Reservations {
    Set<Integer> occupied(int slot);
  }

  private record Constraint(int agent, int slot, int from, int to) {}

  private record Conflict(int a, int b, int slot) {}

  private record Node(
      List<Constraint> constraints, Map<Integer, List<Integer>> paths, long order) {}

  private record State(int cell, int time, State parent, double f, double length) {}

  public final GridGraph grid;
  public final double speed, acceleration, margin, slot;
  private final int horizon, nodeLimit;

  public CooperativePlanner(
      GridGraph grid,
      double speed,
      double acceleration,
      double margin,
      int horizon,
      int nodeLimit) {
    if (speed <= 0 || acceleration <= 0 || margin <= 0 || horizon < 1 || nodeLimit < 1)
      throw new IllegalArgumentException("Invalid planner limits");
    this.grid = grid;
    this.speed = speed;
    this.acceleration = acceleration;
    this.margin = margin;
    this.horizon = horizon;
    this.nodeLimit = nodeLimit;
    slot =
        Math.max(
            2.5 * Math.max(Math.hypot(grid.step.x(), grid.step.y()), grid.step.z()) / speed,
            Math.sqrt(12 * grid.step.norm() / acceleration));
  }

  public Plan plan(List<Agent> requested, Reservations fixed) {
    List<Agent> agents = requested.stream().sorted(Comparator.comparingInt(Agent::id)).toList();
    if (agents.stream().map(Agent::id).distinct().count() != agents.size())
      throw new IllegalArgumentException("Duplicate drone IDs");
    Map<Integer, Agent> byId = new TreeMap<>();
    Map<Integer, List<Integer>> initial = new TreeMap<>();
    for (Agent a : agents) {
      grid.requireCell(a.start);
      grid.requireCell(a.goal);
      if (!grid.legal(a.start, a.start) || !grid.legal(a.goal, a.goal))
        throw new IllegalArgumentException("Blocked start or goal for D" + a.id);
      byId.put(a.id, a);
    }
    if (agents.stream().map(Agent::start).distinct().count() != agents.size()
        || agents.stream().map(Agent::goal).distinct().count() != agents.size())
      throw new IllegalStateException("Starts and permanent goals must be distinct");
    // A priority-respecting reservation solution is an inexpensive CBS
    // incumbent. If it fails, CBS explores both conflict constraints.
    try {
      for (Agent a : agents) {
        Map<Integer, List<Integer>> higher = Map.copyOf(initial);
        Reservations reserved =
            t -> {
              Set<Integer> cells = new HashSet<>(fixed.occupied(t));
              for (List<Integer> p : higher.values())
                cells.addAll(grid.corridor(at(p, t), at(p, t + 1)));
              return cells;
            };
        initial.put(a.id, search(a, List.of(), initialReservations(a, agents, reserved), fixed));
      }
    } catch (NoSuchElementException e) {
      System.out.println(
          "Priority reservation seed unavailable: " + e.getMessage() + "; resolving with CBS");
      initial.clear();
      for (Agent a : agents)
        initial.put(a.id, search(a, List.of(), initialReservations(a, agents, fixed), fixed));
    }
    Comparator<Node> priority =
        Comparator.<Node>comparingInt(n -> n.paths.values().stream().mapToInt(List::size).sum())
            .thenComparingLong(Node::order);
    PriorityQueue<Node> open = new PriorityQueue<>(priority);
    open.add(new Node(List.of(), initial, 0));
    Set<String> seen = new HashSet<>();
    long order = 0;
    for (int expanded = 0; !open.isEmpty() && expanded < nodeLimit; expanded++) {
      Node n = open.remove();
      Map<Integer, PolynomialTrajectory> curves = new TreeMap<>();
      for (var e : n.paths.entrySet())
        curves.put(
            e.getKey(),
            new PolynomialTrajectory(
                grid,
                e.getValue(),
                byId.get(e.getKey()).position,
                slot,
                speed,
                acceleration,
                margin));
      Conflict c = conflict(n.paths, curves);
      if (c == null) return new Plan(Map.copyOf(curves), slot, expanded);
      // Branch order favors the smaller numeric ID. Both branches remain
      // available when a lower-priority path alone cannot resolve it.
      for (int id : new int[] {Math.max(c.a, c.b), Math.min(c.a, c.b)}) {
        List<Integer> p = n.paths.get(id);
        Constraint extra = new Constraint(id, c.slot, at(p, c.slot), at(p, c.slot + 1));
        List<Constraint> next = new ArrayList<>(n.constraints);
        next.add(extra);
        String key =
            next.stream()
                .sorted(
                    Comparator.comparingInt(Constraint::agent)
                        .thenComparingInt(Constraint::slot)
                        .thenComparingInt(Constraint::from)
                        .thenComparingInt(Constraint::to))
                .toList()
                .toString();
        if (!seen.add(key)) continue;
        try {
          List<Integer> replacement =
              search(byId.get(id), next, initialReservations(byId.get(id), agents, fixed), fixed);
          Map<Integer, List<Integer>> paths = new TreeMap<>(n.paths);
          paths.put(id, replacement);
          open.add(new Node(List.copyOf(next), paths, ++order));
        } catch (NoSuchElementException ignored) {
        }
      }
    }
    throw new IllegalStateException(
        "No certified joint plan within CBS limits; no drones were dispatched");
  }

  private Reservations initialReservations(Agent a, List<Agent> agents, Reservations fixed) {
    Set<Integer> others = new HashSet<>();
    for (Agent other : agents) if (other.id != a.id) others.add(other.start);
    return t -> {
      if (t != 0) return fixed.occupied(t);
      Set<Integer> cells = new HashSet<>(fixed.occupied(0));
      cells.addAll(others);
      return cells;
    };
  }

  public Plan plan(List<Agent> agents) {
    return plan(agents, s -> Set.of());
  }

  public void certify(Map<Integer, PolynomialTrajectory> curves) {
    if (curves.isEmpty()) throw new IllegalArgumentException("Empty joint mission");
    Map<Integer, List<Integer>> paths = new TreeMap<>();
    for (var entry : curves.entrySet()) {
      if (Math.abs(entry.getValue().slot - slot) > 1e-9)
        throw new IllegalArgumentException("Trajectory timing differs from planner limits");
      paths.put(entry.getKey(), entry.getValue().path);
    }
    if (conflict(paths, curves) != null)
      throw new IllegalArgumentException(
          "Trajectories have a grid or continuous separation conflict");
  }

  private List<Integer> search(
      Agent a, List<Constraint> constraints, Reservations fixed, Reservations external) {
    // Joint agents occupy their center cells. Their actual body envelopes are
    // checked together by continuous trajectory certification, including t=0.
    // External reservations lack poses/curves, so retain the conservative
    // footprint check against those unknown trajectories.
    if (!Collections.disjoint(grid.footprint(a.position, margin), external.occupied(0)))
      throw new NoSuchElementException(
          "Initial envelope intersects a fixed reservation for D" + a.id);
    List<Set<Integer>> cached = new ArrayList<>();
    for (int t = 0; t <= horizon; t++) cached.add(Set.copyOf(fixed.occupied(t)));
    Reservations reservations = t -> cached.get(t);
    PriorityQueue<State> open =
        new PriorityQueue<>(
            Comparator.comparingDouble(State::f)
                // Equal-time diagonal zigzags can be longer than a straight route.
                // Prefer geometric distance before the deterministic cell-ID tie.
                .thenComparingDouble(
                    s -> s.length + grid.center(s.cell).distance(grid.center(a.goal)))
                .thenComparingInt(State::time)
                .thenComparingInt(State::cell));
    Set<Long> closed = new HashSet<>();
    State initial = new State(a.start, 0, null, heuristic(a.start, a.goal), 0);
    if (a.position.distance(grid.center(a.start)) > .001) {
      if (reservations.occupied(0).contains(a.start)
          || constraints.stream()
              .anyMatch(
                  c -> c.agent == a.id && c.slot == 0 && c.from == a.start && c.to == a.start))
        throw new NoSuchElementException("Initial connector is occupied for D" + a.id);
      open.add(
          new State(
              a.start,
              1,
              initial,
              1 + heuristic(a.start, a.goal),
              a.position.distance(grid.center(a.start))));
    } else open.add(initial);
    int last =
        constraints.stream()
            .filter(c -> c.agent == a.id)
            .mapToInt(Constraint::slot)
            .max()
            .orElse(-1);
    while (!open.isEmpty()) {
      State s = open.remove();
      long key = ((long) s.time << 32) | s.cell;
      if (!closed.add(key)) continue;
      if (s.cell == a.goal && s.time > last && goalSafe(s.cell, s.time, reservations)) {
        LinkedList<Integer> path = new LinkedList<>();
        for (State p = s; p != null; p = p.parent) path.addFirst(p.cell);
        return List.copyOf(path);
      }
      if (s.time >= horizon) continue;
      List<Integer> next =
          new ArrayList<>(grid.outgoing(s.cell).stream().map(GridGraph.Edge::to).toList());
      next.add(s.cell);
      for (int to : next) {
        Set<Integer> corridor = new HashSet<>(grid.corridor(s.cell, to));
        if (!Collections.disjoint(corridor, reservations.occupied(s.time))
            || constraints.stream()
                .anyMatch(
                    c -> c.agent == a.id && c.slot == s.time && c.from == s.cell && c.to == to))
          continue;
        open.add(
            new State(
                to,
                s.time + 1,
                s,
                s.time + 1 + heuristic(to, a.goal),
                s.length + grid.center(s.cell).distance(grid.center(to))));
      }
    }
    throw new NoSuchElementException("No time-expanded path for D" + a.id);
  }

  private boolean goalSafe(int goal, int start, Reservations fixed) {
    for (int t = start; t <= horizon; t++) if (fixed.occupied(t).contains(goal)) return false;
    return true;
  }

  private int heuristic(int a, int b) {
    return Math.max(Math.abs(grid.x(a) - grid.x(b)), Math.abs(grid.y(a) - grid.y(b)))
        + Math.abs(grid.z(a) - grid.z(b));
  }

  private Conflict conflict(
      Map<Integer, List<Integer>> paths, Map<Integer, PolynomialTrajectory> curves) {
    List<Integer> ids = paths.keySet().stream().sorted().toList();
    int length = paths.values().stream().mapToInt(List::size).max().orElse(1);
    for (int t = 0; t < length; t++)
      for (int i = 0; i < ids.size(); i++)
        for (int j = i + 1; j < ids.size(); j++) {
          int a = ids.get(i), b = ids.get(j);
          List<Integer> pa = paths.get(a), pb = paths.get(b);
          Set<Integer> ca = new HashSet<>(grid.corridor(at(pa, t), at(pa, t + 1))),
              cb = new HashSet<>(grid.corridor(at(pb, t), at(pb, t + 1)));
          // Envelope coverage is geometric, not another center-cell occupancy.
          boolean gridConflict = !Collections.disjoint(ca, cb);
          if (gridConflict
              || !PolynomialTrajectory.separated(
                  control(curves.get(a), t), control(curves.get(b), t), 2 * margin, 0))
            return new Conflict(a, b, t);
        }
    return null;
  }

  private static Vec3[] control(PolynomialTrajectory curve, int slot) {
    if (slot < curve.pieces.size()) return curve.pieces.get(slot).control();
    Vec3[] p = new Vec3[8];
    Arrays.fill(p, curve.at(curve.duration()));
    return p;
  }

  public static int at(List<Integer> path, int t) {
    return path.get(Math.min(t, path.size() - 1));
  }
}
