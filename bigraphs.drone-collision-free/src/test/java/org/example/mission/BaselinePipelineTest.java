package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.*;
import com.sun.net.httpserver.HttpServer;
import java.net.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.springframework.mock.env.MockEnvironment;

class BaselinePipelineTest {
  @TempDir Path temporary;

  @Test
  void baselineRunsTheSharedPipelineWithoutMatchingOrAuthorization() throws Exception {
    runPipeline(false, false);
  }

  @Test
  void sameGridConflictIsRecordedWhileBaselineCompletesBothMissions() throws Exception {
    runPipeline(true, false);
  }

  @Test
  void latchedConflictDoesNotTerminateTheNormalMatchingPipeline() throws Exception {
    runPipeline(true, true);
  }

  private void runPipeline(boolean observeConflict, boolean matching) throws Exception {
    var json = new ObjectMapper();
    var requests = new CopyOnWriteArrayList<String>();
    var firstLanding = new CountDownLatch(1);
    JsonNode world =
        json.valueToTree(
            Map.of(
                "backend",
                "webots",
                "drivers_ready",
                true,
                "conflict",
                false,
                "world_hash",
                "c".repeat(64),
                "robots",
                List.of(
                    Map.of("id", 231, "initial_position", List.of(0, 0, .15)),
                    Map.of("id", 232, "initial_position", List.of(1, 0, .15)))));
    var armed = new java.util.concurrent.atomic.AtomicBoolean();
    var experiment = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
    experiment.createContext(
        "/",
        exchange -> {
          String body =
              new String(
                  exchange.getRequestBody().readAllBytes(),
                  java.nio.charset.StandardCharsets.UTF_8);
          if (!body.isEmpty()) assertFalse(body.contains("matching"));
          if (exchange.getRequestURI().getPath().endsWith("/arm")) {
            var payload = json.readTree(body);
            Set<String> fields = new HashSet<>();
            payload.fieldNames().forEachRemaining(fields::add);
            assertEquals(Set.of("run_id", "epoch", "events", "grid"), fields);
            assertEquals(7, payload.path("grid").path("rows").asInt());
            armed.set(true);
          }
          var observation = world.deepCopy();
          if (observeConflict && armed.get()) {
            ((com.fasterxml.jackson.databind.node.ObjectNode) observation).put("conflict", true);
            ((com.fasterxml.jackson.databind.node.ObjectNode) observation)
                .put("physical_contact", false);
          }
          byte[] response = json.writeValueAsBytes(observation);
          exchange.sendResponseHeaders(200, response.length);
          try (var out = exchange.getResponseBody()) {
            out.write(response);
          }
        });
    experiment.start();
    try (var controllers = new MissionLandingTest.Controllers(firstLanding, requests)) {
      controllers.requirePermission = matching;
      var config =
          MissionConfig.from(
              new MockEnvironment()
                  .withProperty("drone.count", "2")
                  .withProperty("drone.control.start-port", "" + controllers.port())
                  .withProperty("drone.targets", "0,0.5;1,0.5")
                  .withProperty("obstacle.grids", "")
                  .withProperty("cdo.enabled", "false")
                  .withProperty("ros.update.enabled", "false")
                  .withProperty("mission.report", temporary.resolve("baseline.json").toString()));
      try (var runner =
          new MissionRunner(
              config,
              new RunOptions(
                  matching,
                  "http://127.0.0.1:" + experiment.getAddress().getPort(),
                  ScenarioTest.scenario()))) {
        runner.run();
      }
      var report = json.readTree(temporary.resolve("baseline.json").toFile());
      assertTrue(report.path("completed").asBoolean());
      assertFalse(report.has("hardware_health"));
      assertEquals(0, report.path("health_match_latency_ms").path("count").asInt());
      assertEquals(observeConflict, report.path("same_grid_conflict").asBoolean());
      assertEquals(
          observeConflict ? "complete_with_grid_conflict" : "complete",
          report.path("run_status").asText());
      assertEquals("same_grid_at_same_simulation_step", report.path("conflict_criterion").asText());
      if (observeConflict)
        assertFalse(report.path("experiment").path("physical_contact").asBoolean());
      assertEquals(!matching, report.path("baseline_mode").asBoolean());
      assertEquals(matching, report.path("matcher_checks").asInt() > 0);
      assertEquals(matching, requests.stream().anyMatch(value -> value.startsWith("authorize:")));
      assertTrue(requests.containsAll(List.of("land:0", "land:1")));
      assertTrue(requests.contains("pause:0"));
      assertTrue(requests.stream().noneMatch(value -> value.startsWith("pause:1:")));
      assertTrue(requests.stream().noneMatch(value -> value.startsWith("stop:")));
      assertTrue(report.path("trial_valid").asBoolean());
      assertEquals(
          observeConflict ? 1 : 0,
          report.path("events").findValues("event").stream()
              .filter(value -> value.asText().equals("grid_conflict_observed"))
              .count());
      assertFalse(report.path("input_fingerprint").asText().isBlank());
    } finally {
      experiment.stop(0);
    }
  }
}
