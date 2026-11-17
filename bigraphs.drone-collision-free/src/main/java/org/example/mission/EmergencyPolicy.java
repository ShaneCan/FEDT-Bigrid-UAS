package org.example.mission;

import java.util.*;

/** Shared hardware response policy. No Bigraph, HTTP, or scenario-specific branching. */
public final class EmergencyPolicy {
  public enum Action {
    NORMAL,
    RETURN_RECHARGE,
    RETURN_HOME,
    LAND_LOCAL,
    HEALTH_UNKNOWN
  }

  public record Response(Action action, String reason) {}

  public static Response required(
      HardwareHealth.View health, Action current, boolean takeoffProtected) {
    if (!health.fresh()) return new Response(Action.HEALTH_UNKNOWN, "health_unknown_or_stale");
    boolean low = health.battery().equals("Low"),
        empty = health.battery().equals("Empty"),
        bad = health.communication().equals("Bad");
    if (empty) return new Response(Action.LAND_LOCAL, "EMPTY_BATTERY");
    if (low && bad) return new Response(Action.LAND_LOCAL, "LOW_BATTERY_BAD_COMM");
    if (current != Action.NORMAL) return new Response(current, "continue_emergency");
    if (!takeoffProtected && low) return new Response(Action.RETURN_RECHARGE, "LOW_BATTERY");
    if (!takeoffProtected && bad) return new Response(Action.RETURN_HOME, "BAD_COMM");
    return new Response(Action.NORMAL, "normal");
  }

  public static int rechargeTarget(
      GridGraph grid, int current, int home, Set<Integer> stations, Set<Integer> unavailable) {
    Set<Integer> candidates = new TreeSet<>(stations);
    candidates.add(home);
    return candidates.stream()
        .filter(c -> !unavailable.contains(c))
        .min(
            Comparator.<Integer>comparingDouble(c -> grid.center(c).distance(grid.center(current)))
                .thenComparingInt(Integer::intValue))
        .orElse(-1);
  }

  public static Set<Integer> column(GridGraph grid, Vec3 actual, double margin) {
    int cell = grid.cell(actual);
    Set<Integer> result = new TreeSet<>();
    for (int z = 0; z <= grid.z(cell); z++) {
      Vec3 p = new Vec3(actual.x(), actual.y(), grid.origin.z() + (z + .5) * grid.step.z());
      result.addAll(grid.footprint(p, margin));
      if (z > 0
          && !grid.legal(
              grid.index(grid.x(cell), grid.y(cell), z),
              grid.index(grid.x(cell), grid.y(cell), z - 1)))
        throw new IllegalStateException("Emergency descent lacks a legal DownRoute");
    }
    return Set.copyOf(result);
  }
}
