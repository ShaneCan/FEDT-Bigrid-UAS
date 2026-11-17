package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.springframework.mock.env.MockEnvironment;

@org.junit.jupiter.api.Timeout(45)
class HardwareMissionTest {
  @TempDir Path temporary;
  private final ObjectMapper json = new ObjectMapper();

  private JsonNode run(
      boolean matching, java.util.function.Consumer<MissionLandingTest.Controllers> setup)
      throws Exception {
    var requests = new CopyOnWriteArrayList<String>();
    try (var c = new MissionLandingTest.Controllers(new CountDownLatch(0), requests)) {
      c.backend = "hardware";
      c.requirePermission = matching;
      for (int id = 0; id < 2; id++) {
        c.phase[id] = "active";
        c.position[id] = new Vec3(id, 0, .15);
      }
      setup.accept(c);
      var provider = GridServiceTest.server(0);
      var env =
          new MockEnvironment()
              .withProperty("drone.count", "2")
              .withProperty("drone.control.start-port", "" + c.port())
              .withProperty(
                  "bigrid.service.base-url", "http://127.0.0.1:" + provider.getAddress().getPort())
              .withProperty("drone.targets", "0,0.5;1,0.5")
              .withProperty("obstacle.grids", "")
              .withProperty("cdo.enabled", "false")
              .withProperty("ros.update.enabled", "false")
              .withProperty("fault.injection.port", "0")
              .withProperty("hardware.health.startup-timeout", "0")
              .withProperty("hardware.health.required", "true")
              .withProperty("mission.report", temporary.resolve("report.json").toString());
      try (var runner =
          new MissionRunner(
              MissionConfig.from(env),
              new RunOptions(matching, "", Scenario.none(), HardwareHealth.Config.from(env)))) {
        if (c.rejectStop[0] || c.backend.equals("webots"))
          assertThrows(Exception.class, runner::run);
        else runner.run();
      } finally {
        provider.stop(0);
      }
      JsonNode report = json.readTree(temporary.resolve("report.json").toFile());
      assertEquals(
          !c.rejectStop[0] && c.backend.equals("hardware"),
          report.path("completed").asBoolean(),
          report.toString());
      assertEquals(
          matching && c.backend.equals("hardware"),
          requests.stream().anyMatch(r -> r.startsWith("authorize:")));
      assertEquals(
          matching && c.backend.equals("hardware"),
          report.path("health_match_latency_ms").path("count").asInt() > 0);
      if (!matching) assertEquals(0, report.path("matcher_checks").asInt());
      return ((com.fasterxml.jackson.databind.node.ObjectNode) report)
          .set("requests", json.valueToTree(requests));
    }
  }

  @Test
  void hardwareBaselineRunsWithoutAnExperimentAndSkipsOnlyUnhealthyGroundAircraft()
      throws Exception {
    var report =
        run(
            false,
            c -> {
              c.voltage[0] = 3.0;
            });
    assertEquals(List.of(0), json.convertValue(report.path("skipped_aircraft"), List.class));
    assertFalse(report.path("requests").toString().contains("takeoff:0"));
    assertTrue(report.path("requests").toString().contains("takeoff:1"));
    assertEquals(1, report.path("initial_input").path("initial_positions").size());
  }

  @Test
  void degradationBetweenPreflightAndTakeoffReplansWithoutStartingTheAircraft() throws Exception {
    var report =
        run(
            true,
            c ->
                c.beforeRequest =
                    (id, path) -> {
                      if (id == 0 && path.equals("/activate_idle")) c.radio[0] = 70;
                    });
    assertEquals(0, report.path("skipped_aircraft").get(0).asInt());
    assertFalse(report.path("requests").toString().contains("takeoff:0"));
  }

  @Test
  void lowBatteryAndBadRadioUseTheSharedReturnPolicyInBothEntrypoints() throws Exception {
    for (boolean matching : List.of(false, true))
      for (boolean battery : List.of(false, true)) {
        var starts = new AtomicInteger();
        var report =
            run(
                matching,
                c ->
                    c.beforeRequest =
                        (id, path) -> {
                          if (id == 0
                              && path.equals("/trajectory/full_state/start")
                              && starts.incrementAndGet() == 1) {
                            if (battery) c.voltage[0] = 3.15;
                            else c.radio[0] = 70;
                            c.completeNotBefore[0] = System.nanoTime() + 1_600_000_000L;
                          }
                        });
        assertEquals(
            battery ? "RETURN_RECHARGE" : "RETURN_HOME",
            report.path("emergency_actions").path("0").asText());
        assertTrue(report.path("requests").toString().contains("stop:0"));
        assertTrue(report.path("requests").toString().contains("land:0"));
        assertEquals(
            0,
            report.path("final_goals").path("0").asInt()
                - report.path("original_goals").path("0").asInt()
                + 1);
      }
  }

  @Test
  void emptyAndCombinedFaultLandAtActualHorizontalPosition() throws Exception {
    for (boolean matching : List.of(false, true))
      for (boolean combined : List.of(false, true)) {
        var starts = new AtomicInteger();
        var report =
            run(
                matching,
                c ->
                    c.beforeRequest =
                        (id, path) -> {
                          if (id == 0
                              && path.equals("/trajectory/full_state/start")
                              && starts.incrementAndGet() == 1) {
                            c.voltage[0] = combined ? 3.15 : 3;
                            if (combined) c.radio[0] = 70;
                            c.position[0] = new Vec3(.04, 0, .3);
                            c.completeNotBefore[0] = System.nanoTime() + 2_000_000_000L;
                          }
                        });
        assertEquals("LAND_LOCAL", report.path("emergency_actions").path("0").asText());
        var landing =
            report.path("events").findValues("event").stream()
                .filter(v -> v.asText().equals("emergency_landing_started"))
                .count();
        assertEquals(1, landing);
        assertEquals(1, starts.get());
        for (var event : report.path("events"))
          if (event.path("event").asText().equals("emergency_landing_started"))
            assertEquals(.04, event.path("position").get(0).asDouble(), 1e-9);
      }
  }

  @Test
  void emptyDuringTakeoffInterruptsBeforeAnyMissionTrajectoryForThatAircraft() throws Exception {
    var report =
        run(
            true,
            c -> {
              c.pendingTakeoff[0] = true;
              c.beforeRequest =
                  (id, path) -> {
                    if (id == 0 && path.equals("/status") && c.phase[0].equals("taking_off"))
                      c.voltage[0] = 3;
                  };
            });
    assertEquals("LAND_LOCAL", report.path("emergency_actions").path("0").asText());
    assertFalse(report.path("requests").toString().contains("upload-z:0"));
    assertTrue(report.path("requests").toString().contains("land:0"));
  }

  @Test
  void returnEscalatesToLocalLandingAndRecoveryNeverRestartsTheOriginalMission() throws Exception {
    for (boolean escalate : List.of(false, true)) {
      var starts = new AtomicInteger();
      var report =
          run(
              true,
              c ->
                  c.beforeRequest =
                      (id, path) -> {
                        if (id == 0 && path.equals("/trajectory/full_state/start")) {
                          int n = starts.incrementAndGet();
                          c.completeNotBefore[0] = System.nanoTime() + 2_100_000_000L;
                          if (n == 1) c.radio[0] = 70;
                          else {
                            c.radio[0] = 50;
                            c.voltage[0] = escalate ? 3 : 3.8;
                          }
                        }
                      });
      assertEquals(
          escalate ? "LAND_LOCAL" : "RETURN_HOME",
          report.path("emergency_actions").path("0").asText());
      assertEquals(2, starts.get());
      assertEquals("emergency_landed", report.path("aircraft_outcomes").path("0").asText());
    }
  }

  @Test
  void staleHealthStopsAndRecordsAnUnconfirmedOutcomeWithoutClaimingALanding() throws Exception {
    var starts = new AtomicInteger();
    var report =
        run(
            false,
            c ->
                c.beforeRequest =
                    (id, path) -> {
                      if (id == 0
                          && path.equals("/trajectory/full_state/start")
                          && starts.incrementAndGet() == 1) {
                        c.healthAge[0] = 4;
                        c.completeNotBefore[0] = System.nanoTime() + 2_000_000_000L;
                      }
                    });
    assertEquals("emergency_unconfirmed", report.path("aircraft_outcomes").path("0").asText());
    assertEquals("failed", report.path("run_status").asText());
    assertFalse(report.path("trial_valid").asBoolean());
    assertFalse(report.path("requests").toString().contains("land:0"));
    assertTrue(report.path("requests").toString().contains("land:1"));
  }

  @Test
  void rejectedStopPreservesTheFailureReportAndNeverClaimsEmergencyLanding() throws Exception {
    var starts = new AtomicInteger();
    var report =
        run(
            false,
            c -> {
              c.rejectStop[0] = true;
              c.beforeRequest =
                  (id, path) -> {
                    if (id == 0
                        && path.equals("/trajectory/full_state/start")
                        && starts.incrementAndGet() == 1) {
                      c.voltage[0] = 3;
                      c.completeNotBefore[0] = System.nanoTime() + 2_000_000_000L;
                    }
                  };
            });
    assertEquals("failed", report.path("run_status").asText());
    assertTrue(report.path("events").toString().contains("cleanup_stop_unconfirmed"));
    assertFalse(report.path("requests").toString().contains("land:0"));
  }

  @Test
  void blockedDescentClearsTheOtherAircraftThenResumesItsOriginalGoal() throws Exception {
    var starts0 = new AtomicInteger();
    var starts1 = new AtomicInteger();
    var report =
        run(
            true,
            c -> {
              c.phase[0] = "hovering";
              c.position[0] = new Vec3(0, 0, .9);
              c.phase[1] = "hovering";
              c.position[1] = new Vec3(0, 0, .3);
              c.beforeRequest =
                  (id, path) -> {
                    if (path.equals("/trajectory/full_state/start")) {
                      if (id == 0 && starts0.incrementAndGet() == 1) {
                        c.voltage[0] = 3;
                        c.completeNotBefore[0] = System.nanoTime() + 3_000_000_000L;
                      }
                      if (id == 1)
                        c.completeNotBefore[1] =
                            starts1.incrementAndGet() == 1
                                ? System.nanoTime() + 10_000_000_000L
                                : 0;
                    }
                  };
            });
    assertTrue(report.path("events").toString().contains("emergency_column_clearing"));
    assertEquals(report.path("original_goals").path("1"), report.path("final_goals").path("1"));
    assertEquals("emergency_landed", report.path("aircraft_outcomes").path("0").asText());
    assertEquals("mission_landed", report.path("aircraft_outcomes").path("1").asText());
    assertTrue(starts1.get() >= 3);
  }

  @Test
  void hardwareProfileRejectsWebotsBeforeAnyTakeoffOrHealthListener() throws Exception {
    for (boolean matching : List.of(false, true)) {
      var report = run(matching, c -> c.backend = "webots");
      assertFalse(report.has("hardware_health"));
      assertFalse(report.path("requests").toString().contains("takeoff:"));
    }
  }

  @Test
  void standaloneDegradationDuringTakeoffDoesNotStartTheOriginalMission() throws Exception {
    for (boolean matching : List.of(false, true)) {
      var report =
          run(
              matching,
              c -> {
                c.pendingTakeoff[0] = true;
                c.beforeRequest =
                    (id, path) -> {
                      if (id == 0 && path.equals("/status") && c.phase[0].equals("taking_off")) {
                        if (matching) c.radio[0] = 70;
                        else c.voltage[0] = 3.15;
                      }
                    };
              });
      assertEquals(
          matching ? "RETURN_HOME" : "RETURN_RECHARGE",
          report.path("emergency_actions").path("0").asText());
      assertFalse(report.path("requests").toString().contains("upload-z:0"));
      assertTrue(report.path("requests").toString().contains("land:0"));
    }
  }

  @Test
  void readyAircraftRemainMonitoredWhileAnotherIsStillTakingOff() throws Exception {
    var began = new AtomicLong();
    var starts1 = new AtomicInteger();
    var report =
        run(
            true,
            c -> {
              c.phase[0] = "hovering";
              c.position[0] = new Vec3(0, 0, .9);
              c.phase[1] = "active";
              c.position[1] = new Vec3(0, 0, .15);
              c.pendingTakeoff[1] = true;
              c.beforeRequest =
                  (id, path) -> {
                    if (id == 1 && path.equals("/begin_takeoff")) began.set(System.nanoTime());
                    if (id == 0 && path.equals("/status") && began.get() != 0) c.voltage[0] = 3;
                    if (id == 1
                        && path.startsWith("/tasks/takeoff-")
                        && System.nanoTime() - began.get() > 1_600_000_000L) {
                      c.phase[1] = "hovering";
                      c.tasks.put(c.current[1], "completed");
                    }
                    if (id == 1 && path.equals("/trajectory/full_state/start"))
                      starts1.incrementAndGet();
                  };
            });
    assertEquals("emergency_landed", report.path("aircraft_outcomes").path("0").asText());
    assertEquals("mission_landed", report.path("aircraft_outcomes").path("1").asText());
    assertTrue(report.path("events").toString().contains("emergency_column_clearing"));
    assertTrue(starts1.get() >= 2);
  }
}
