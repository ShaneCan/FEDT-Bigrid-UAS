package org.example.mission;

import static org.bigraphs.framework.core.factory.BigraphFactory.*;

import java.util.*;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.framework.core.impl.signature.DynamicSignature;
import org.bigraphs.framework.core.reactivesystem.ParametricReactionRule;
import org.bigraphs.framework.simulation.matching.AbstractBigraphMatcher;

/** Actual health-rule matching, separate from spatial movement rules. Baseline never creates it. */
public final class HealthGate {
  private final DynamicSignature signature =
      pureSignatureBuilder()
          .add("Drone", 0)
          .add("Battery", 0)
          .add("Communication", 0)
          .add("Normal", 0)
          .add("Low", 0)
          .add("Empty", 0)
          .add("Bad", 0)
          .add("Unknown", 0)
          .create();
  private final AbstractBigraphMatcher<PureBigraph> matcher =
      AbstractBigraphMatcher.create(PureBigraph.class);
  private final Map<String, ParametricReactionRule<PureBigraph>> rules = new HashMap<>();

  public record Result(boolean allowed, double matchMs) {}

  public synchronized Result check(HardwareHealth.View health, EmergencyPolicy.Action action)
      throws Exception {
    var world = state(health.battery(), health.communication());
    List<String> accepted =
        switch (action) {
          case NORMAL -> List.of("Normal/Normal");
          case RETURN_RECHARGE -> List.of("Low/Normal", "Normal/Normal", "Normal/Bad");
          case RETURN_HOME -> List.of("Normal/Bad", "Normal/Normal", "Low/Normal");
          case LAND_LOCAL ->
              List.of(
                  "Normal/Normal",
                  "Normal/Bad",
                  "Low/Normal",
                  "Low/Bad",
                  "Empty/Normal",
                  "Empty/Bad");
          case HEALTH_UNKNOWN -> List.of();
        };
    long began = System.nanoTime();
    boolean allowed = false;
    for (String key : accepted) {
      var rule = rules.get(key);
      if (rule == null) {
        String[] parts = key.split("/");
        var pattern = state(parts[0], parts[1]);
        rule = new ParametricReactionRule<>(pattern, pattern);
        rules.put(key, rule);
      }
      if (matcher.match(world, rule).iterator().hasNext()) {
        allowed = true;
        break;
      }
    }
    return new Result(allowed, (System.nanoTime() - began) / 1e6);
  }

  private PureBigraph state(String battery, String comm) throws Exception {
    var b = pureBuilder(signature);
    var drone = b.root().child("Drone").down();
    drone.child("Battery").down().child(battery).up();
    drone.child("Communication").down().child(comm);
    return b.create();
  }
}
