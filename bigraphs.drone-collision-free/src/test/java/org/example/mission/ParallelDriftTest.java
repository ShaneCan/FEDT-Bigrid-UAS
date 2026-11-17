package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.*;
import java.util.*;
import org.junit.jupiter.api.Test;
import org.springframework.mock.env.MockEnvironment;

class ParallelDriftTest {
  static MockEnvironment environment(String profile) throws Exception {
    var p = new Properties();
    try (var in =
        ParallelDriftTest.class.getResourceAsStream("/application-" + profile + ".properties")) {
      p.load(in);
    }
    var e = new MockEnvironment().withProperty("mission.mode", "bigraph");
    p.forEach((k, v) -> e.withProperty(k.toString(), v.toString()));
    return e;
  }

  static JsonNode world(ObjectMapper j) {
    return j.valueToTree(
        Map.of(
            "backend",
            "webots",
            "drivers_ready",
            true,
            "robots",
            List.of(
                Map.of("id", 231, "initial_position", List.of(0, 2, .05)),
                Map.of("id", 232, "initial_position", List.of(.5, 2.5, .05)))));
  }

  @Test
  void bothModesPrepareIdenticalInitialReferencesAndEightSecondInput() throws Exception {
    var env = environment("drift");
    var config = MissionConfig.from(env);
    var options = RunOptions.from(env);
    var j = new ObjectMapper();
    var first = options.scenario().prepare(config, world(j), j);
    env.withProperty("mission.matching", "false");
    var second = RunOptions.from(env).scenario().prepare(config, world(j), j);
    assertEquals(first.fingerprint(), second.fingerprint());
    assertEquals(1, first.events().size());
    assertEquals(3, ((Number) first.events().getFirst().get("ramp")).doubleValue());
    assertEquals(
        8,
        first.pauses().get(0).getFirst().endAt() - first.pauses().get(0).getFirst().startAt(),
        1e-9);
    for (int id : List.of(0, 1)) {
      var curve = first.plan().trajectories().get(id);
      assertEquals(9, curve.pieces.size());
      assertEquals(0, curve.stationarySeconds());
      assertEquals(0, curve.interiorStops());
      assertEquals(4.5, curve.at(0).distance(curve.at(curve.duration())), 1e-9);
    }
    // Validate prescribed references and input; observed conflict is a result, not a test
    // prerequisite.
    for (double t = 0; t < first.plan().trajectories().get(0).duration(); t += .1) {
      var delta =
          first.plan().trajectories().get(1).at(t).sub(first.plan().trajectories().get(0).at(t));
      assertEquals(.5, delta.x(), 1e-8);
      assertEquals(.5, delta.y(), 1e-8);
    }
  }

  @Test
  void multiEventProfileUsesBothAircraftWithoutChangingExecutor() throws Exception {
    var env = environment("multievent");
    var j = new ObjectMapper();
    var p = RunOptions.from(env).scenario().prepare(MissionConfig.from(env), world(j), j);
    assertEquals(4, p.events().size());
    assertTrue(p.events().stream().anyMatch(e -> e.get("drone_id").equals(232)));
    assertEquals(2, p.pauses().get(0).size());
    assertEquals(new Scenario.Window(75, 78), p.pauses().get(0).getLast());
  }
}
