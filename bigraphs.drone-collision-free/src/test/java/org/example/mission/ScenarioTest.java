package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.*;
import java.nio.file.Path;
import java.util.*;
import org.junit.jupiter.api.Test;
import org.springframework.mock.env.MockEnvironment;

class ScenarioTest {
  static MissionConfig config() {
    return MissionConfig.from(
        new MockEnvironment()
            .withProperty("drone.count", "2")
            .withProperty("drone.targets", "0,0.5;1,0.5"));
  }

  static JsonNode world(ObjectMapper j, boolean reverse) {
    var robots =
        new ArrayList<>(
            List.of(
                Map.of("id", 231, "initial_position", List.of(0, 0, .05)),
                Map.of("id", 232, "initial_position", List.of(1, 0, .05))));
    if (reverse) Collections.reverse(robots);
    return j.valueToTree(Map.of("backend", "webots", "drivers_ready", true, "robots", robots));
  }

  static Scenario scenario() {
    return new Scenario(
        List.of(new Scenario.Event("gust", "wind", 231, null, .5, 2, 2, "0,.5,0", true, null)));
  }

  @Test
  void separatePreparationAndVisualChangesDoNotNeedSavedFiles() throws Exception {
    var j = new ObjectMapper();
    var first = scenario().prepare(config(), world(j, false), j);
    var changed = world(j, true).deepCopy();
    ((com.fasterxml.jackson.databind.node.ObjectNode) changed)
        .put("world_hash", "different visual file");
    var second = scenario().prepare(config(), changed, j);
    assertEquals(first.fingerprint(), second.fingerprint());
    assertEquals(j.valueToTree(first.input()), j.valueToTree(second.input()));
    var window = first.pauses().get(0).getFirst();
    assertEquals(
        Scenario.arcTime(first.plan().trajectories().get(0), .5), window.startAt() + 3, 1e-9);
    assertEquals(
        window.startAt(),
        Scenario.elapsed(100, 100 + window.startAt() + 2, first.windows(0, 100)),
        1e-9);
    assertEquals(
        0,
        Scenario.elapsed(
            100 + window.startAt() + 1, 100 + window.startAt() + 2, first.windows(0, 100)),
        1e-9);
    assertTrue(first.windows(1, 100).isEmpty());
  }

  @Test
  void multipleEventsMergePausesAndRejectOverlappingWind() throws Exception {
    var j = new ObjectMapper();
    var events = new ArrayList<>(scenario().events());
    events.add(new Scenario.Event("other", "wind", 232, 1.0, null, 1, 1, ".1,0,0", false, null));
    events.add(new Scenario.Event("pause", "pause", 231, 3.0, null, 0, 0, null, false, 2.0));
    var first = new Scenario(events).prepare(config(), world(j, false), j);
    Collections.reverse(events);
    assertEquals(
        first.fingerprint(),
        new Scenario(events).prepare(config(), world(j, true), j).fingerprint());
    assertEquals(1, first.pauses().get(0).size());
    events.add(new Scenario.Event("overlap", "wind", 231, 4.0, null, 1, 1, ".1,0,0", false, null));
    assertThrows(
        IllegalArgumentException.class,
        () -> new Scenario(events).prepare(config(), world(j, false), j));
  }

  @Test
  void worldIdentityAndLegacyInputsFailClearly() {
    var j = new ObjectMapper();
    var bad = world(j, false).deepCopy();
    ((com.fasterxml.jackson.databind.node.ObjectNode) bad).put("backend", "hardware");
    assertThrows(IllegalStateException.class, () -> Scenario.none().prepare(config(), bad, j));
    assertThrows(
        IllegalArgumentException.class,
        () -> Scenario.from(new MockEnvironment().withProperty("mission.plan", "old.json")));
    assertThrows(
        IllegalArgumentException.class,
        () -> new Scenario.Event("bad", "wind", 231, 0.0, null, 3, 2, "2,0,0", false, null));
  }

  @Test
  void fingerprintsAgreeAcrossIndependentJvms() throws Exception {
    String[] fingerprints = new String[2];
    for (int i = 0; i < 2; i++) {
      var child =
          new ProcessBuilder(
                  Path.of(System.getProperty("java.home"), "bin", "java").toString(),
                  "-cp",
                  System.getProperty("java.class.path"),
                  FingerprintProbe.class.getName(),
                  "" + i)
              .redirectErrorStream(true)
              .start();
      String output = new String(child.getInputStream().readAllBytes()).trim();
      assertTrue(child.waitFor(30, java.util.concurrent.TimeUnit.SECONDS));
      assertEquals(0, child.exitValue(), output);
      fingerprints[i] = output;
    }
    assertEquals(fingerprints[0], fingerprints[1]);
    assertEquals(64, fingerprints[0].length());
  }

  public static class FingerprintProbe {
    public static void main(String[] args) throws Exception {
      System.out.print(
          scenario()
              .prepare(config(), world(new ObjectMapper(), args[0].equals("1")), new ObjectMapper())
              .fingerprint());
    }
  }
}
