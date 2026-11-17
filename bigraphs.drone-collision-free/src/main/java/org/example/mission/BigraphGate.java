package org.example.mission;

import static org.bigraphs.framework.core.factory.BigraphFactory.*;

import java.util.*;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.framework.core.impl.signature.DynamicSignature;
import org.bigraphs.framework.core.reactivesystem.ParametricReactionRule;
import org.bigraphs.framework.simulation.matching.AbstractBigraphMatcher;

/** Exact-cell anchors are controls: matching cannot rename them like outer links. */
public final class BigraphGate {
  public record Occupant(int id, int cell, boolean flying) {}

  public record Snapshot(long version, List<Occupant> occupants) {
    public Snapshot {
      occupants = List.copyOf(occupants);
    }
  }

  public record Result(boolean allowed, double matchMs, long snapshotVersion) {}

  private final GridGraph grid;
  private final DynamicSignature signature;
  private final AbstractBigraphMatcher<PureBigraph> matcher =
      AbstractBigraphMatcher.create(PureBigraph.class);
  private final Map<String, ParametricReactionRule<PureBigraph>> rules = new HashMap<>();

  public BigraphGate(GridGraph grid, int count) {
    this.grid = grid;
    var b =
        pureSignatureBuilder()
            .add("Locale", 1)
            .add("OccupiedBy", 0)
            .add("Drone", 0)
            .add("Flying", 0)
            .add("Unavailable", 0)
            .add("Obstacle", 0);
    for (String route :
        List.of(
            "ForwardRoute",
            "BackRoute",
            "LeftRoute",
            "RightRoute",
            "ForwardLeftRoute",
            "ForwardRightRoute",
            "BackLeftRoute",
            "BackRightRoute",
            "UpRoute",
            "DownRoute")) b.add(route, 1);
    for (int x = 0; x < grid.cols; x++) b.add("X" + x, 0);
    for (int y = 0; y < grid.rows; y++) b.add("Y" + y, 0);
    for (int z = 0; z < grid.layers; z++) b.add("Z" + z, 0);
    for (int d = 0; d < count; d++) b.add("D" + d, 0);
    signature = b.create();
  }

  public synchronized Result check(Snapshot snapshot, int drone, int from, int to, boolean full)
      throws Exception {
    if (!grid.legal(from, to) || from == to) return new Result(false, 0, snapshot.version);
    Set<Integer> scope = full ? new TreeSet<>() : new TreeSet<>(grid.corridor(from, to));
    if (full) for (int i = 0; i < grid.size(); i++) scope.add(i);
    PureBigraph world = world(snapshot, scope);
    String key = drone + ":" + from + ":" + to;
    ParametricReactionRule<PureBigraph> rule = rules.get(key);
    if (rule == null) {
      rule = rule(drone, from, to);
      rules.put(key, rule);
    }
    long started = System.nanoTime();
    boolean allowed = matcher.match(world, rule).iterator().hasNext();
    return new Result(allowed, (System.nanoTime() - started) / 1e6, snapshot.version);
  }

  /** Match a singleton flying occupant in its expected current cell. */
  public synchronized Result checkState(Snapshot snapshot, int drone, int cell) throws Exception {
    grid.requireCell(cell);
    String key = "state:" + drone + ":" + cell;
    var rule = rules.get(key);
    if (rule == null) {
      var b = pureBuilder(signature);
      var locale = b.root().child("Locale", "cell" + cell).down();
      address(locale, cell);
      locale
          .site()
          .child("OccupiedBy")
          .down()
          .child("Drone")
          .down()
          .child("D" + drone)
          .child("Flying");
      PureBigraph redex = b.create();
      rule = new ParametricReactionRule<>(redex, redex);
      rules.put(key, rule);
    }
    PureBigraph world = world(snapshot, Set.of(cell));
    long begin = System.nanoTime();
    boolean allowed = matcher.match(world, rule).iterator().hasNext();
    return new Result(allowed, (System.nanoTime() - begin) / 1e6, snapshot.version);
  }

  public PureBigraph world(Snapshot snapshot, Set<Integer> scope) throws Exception {
    var b = pureBuilder(signature);
    var root = b.root();
    for (int c : scope) {
      var locale = root.child("Locale", "cell" + c).down();
      address(locale, c);
      for (var e : grid.links())
        if (e.from() == c && scope.contains(e.to())) locale.child(e.route(), "cell" + e.to());
      var occupied = locale.child("OccupiedBy").down();
      if (grid.blocked.contains(c)) occupied.child("Obstacle");
      for (Occupant o : snapshot.occupants)
        if (o.cell == c)
          occupied
              .child("Drone")
              .down()
              .child("D" + o.id)
              .child(o.flying ? "Flying" : "Unavailable");
    }
    return b.create();
  }

  public PureBigraph world(Snapshot snapshot) throws Exception {
    Set<Integer> cells = new TreeSet<>();
    for (int c = 0; c < grid.size(); c++) cells.add(c);
    return world(snapshot, cells);
  }

  private void address(
      org.bigraphs.framework.core.impl.pure.PureBigraphBuilder<DynamicSignature>.Hierarchy locale,
      int cell)
      throws Exception {
    locale.child("X" + grid.x(cell)).child("Y" + grid.y(cell)).child("Z" + grid.z(cell));
  }

  private ParametricReactionRule<PureBigraph> rule(int drone, int from, int to) throws Exception {
    var b = pureBuilder(signature);
    var root = b.root();
    var source = root.child("Locale", "cell" + from).down();
    address(source, from);
    source
        .child(grid.route(from, to), "cell" + to)
        .site()
        .child("OccupiedBy")
        .down()
        .child("Drone")
        .down()
        .child("D" + drone)
        .child("Flying");
    Set<Integer> empty = new TreeSet<>(grid.support(from, to));
    empty.add(to);
    for (int c : empty) {
      var locale = root.child("Locale", "cell" + c).down();
      address(locale, c);
      locale.site().child("OccupiedBy");
    }
    PureBigraph redex = b.create();
    return new ParametricReactionRule<>(redex, redex);
  }
}
