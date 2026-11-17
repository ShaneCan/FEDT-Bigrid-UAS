package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.*;
import com.sun.net.httpserver.HttpServer;
import java.net.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.springframework.mock.env.MockEnvironment;

class MissionLandingTest {
  @TempDir Path temporary;

  @Test
  void firstArrivalLandsWhileAnotherMissionIsStillRunning() throws Exception {
    checkIndependentLanding(false, false, false);
  }

  @Test
  void upperLayerArrivalDescendsAndLandsWhileAnotherMissionIsStillRunning() throws Exception {
    checkIndependentLanding(true, false, false);
  }

  @Test
  void proximityAndTrackingErrorAreMeasurementsRatherThanIndependentStops() throws Exception {
    checkIndependentLanding(false, true, false);
  }

  @Test
  void takeoffSettlingErrorPreservesTheJointPlanPreparedBeforeTakeoff() throws Exception {
    checkIndependentLanding(false, false, true);
  }

  private void checkIndependentLanding(boolean upperLayer, boolean closePositions, boolean settling)
      throws Exception {
    var firstLanding = new CountDownLatch(1);
    var requests = new CopyOnWriteArrayList<String>();
    try (var controllers = new Controllers(firstLanding, requests)) {
      controllers.closePositions = closePositions;
      if (settling)
        for (int id = 0; id < 2; id++) {
          controllers.phase[id] = "active";
          controllers.position[id] = new Vec3(controllers.position[id].x(), 0, .15);
        }
      if (closePositions) controllers.position[1] = new Vec3(.5, 0, .3);
      var provider = GridServiceTest.server(0);
      var cfg =
          MissionConfig.from(
              new MockEnvironment()
                  .withProperty("drone.count", "2")
                  .withProperty("drone.control.start-port", "" + controllers.port())
                  .withProperty(
                      "bigrid.service.base-url",
                      "http://127.0.0.1:" + provider.getAddress().getPort())
                  .withProperty("obstacle.grids", "")
                  .withProperty(
                      "drone.targets",
                      upperLayer
                          ? "0,0.5,0.9;1,0.5"
                          : closePositions ? "0,0.5;0.5,0.5" : "0,0.5;1,0.5")
                  .withProperty("cdo.enabled", "false")
                  .withProperty("ros.update.enabled", "false")
                  .withProperty("mission.report", temporary.resolve("report.json").toString()));
      try (var runner = new MissionRunner(cfg);
          var worker = Executors.newVirtualThreadPerTaskExecutor()) {
        var result =
            worker.submit(
                () -> {
                  runner.run();
                  return null;
                });
        try {
          assertTrue(
              firstLanding.await(5, TimeUnit.SECONDS),
              "The first arrival must land without waiting for the second mission");
          result.get(5, TimeUnit.SECONDS);
          assertTrue(requests.indexOf("land:0") < requests.indexOf("complete:1"));
          JsonNode report = new ObjectMapper().readTree(temporary.resolve("report.json").toFile());
          assertTrue(report.path("completed").asBoolean());
          if (settling) {
            assertTrue(requests.contains("upload-z:0:0.3"));
            assertTrue(requests.contains("upload-z:1:0.3"));
          }
          if (closePositions) {
            assertTrue(report.path("max_tracking_error_m").asDouble() > .05);
            assertTrue(report.path("minimum_distance_m").asDouble() < .19);
            assertEquals(0, report.path("match_rejections").asInt());
            assertEquals(0, report.path("replans").asInt());
          }
          assertTrue(
              report.path("events").findValues("event").stream()
                  .anyMatch(event -> event.asText().equals("landing_started")));
          assertEquals(
              upperLayer,
              report.path("events").findValues("event").stream()
                  .anyMatch(event -> event.asText().equals("descent_started")));
        } finally {
          firstLanding.countDown();
          result.cancel(true);
        }
      } finally {
        provider.stop(0);
      }
    }
  }

  /** Task protocol fixture: completion, rather than physical motion, is under test. */
  static final class Controllers implements AutoCloseable {
    final HttpServer[] servers = new HttpServer[2];
    final ObjectMapper json = new ObjectMapper();
    final CountDownLatch firstLanding;
    final List<String> requests;
    final AtomicInteger taskCounter = new AtomicInteger();
    final long epoch = System.nanoTime();
    final String[] phase = {"hovering", "hovering"}, current = {"", ""};
    final Vec3[] position = {new Vec3(0, 0, .3), new Vec3(1, 0, .3)};
    final Vec3[] target = new Vec3[2];
    final Map<String, String> tasks = new ConcurrentHashMap<>();
    final Set<Integer> authorized = new HashSet<>();
    boolean closePositions, requirePermission;

    Controllers(CountDownLatch firstLanding, List<String> requests) throws Exception {
      this.firstLanding = firstLanding;
      this.requests = requests;
      for (int attempt = 0; attempt < 30; attempt++) {
        servers[0] = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        try {
          servers[1] = HttpServer.create(new InetSocketAddress("127.0.0.1", port() + 1), 0);
          break;
        } catch (BindException unavailable) {
          servers[0].stop(0);
        }
      }
      if (servers[1] == null) throw new IllegalStateException("No consecutive HTTP fixture ports");
      for (int id = 0; id < 2; id++) {
        final int drone = id;
        servers[id].createContext(
            "/",
            exchange -> {
              byte[] input = exchange.getRequestBody().readAllBytes();
              JsonNode body = input.length > 0 ? json.readTree(input) : json.createObjectNode();
              boolean reject =
                  rejectStop[drone]
                      && exchange.getRequestURI().getPath().equals("/trajectory/stop");
              if (reject) requests.add("stop-rejected:" + drone);
              byte[] reply =
                  json.writeValueAsBytes(
                      reject
                          ? Map.of("error", "fixture stop rejected")
                          : handle(drone, exchange.getRequestURI().getPath(), body));
              exchange.sendResponseHeaders(reject ? 503 : 200, reply.length);
              try (var out = exchange.getResponseBody()) {
                out.write(reply);
              }
            });
        servers[id].start();
      }
    }

    int port() {
      return servers[0].getAddress().getPort();
    }

    String backend = "webots";
    final double[] voltage = {3.8, 3.8}, healthAge = {0, 0};
    final int[] radio = {50, 50};
    final long[] completeNotBefore = {0, 0};
    final boolean[] pendingTakeoff = {false, false}, rejectStop = {false, false};
    java.util.function.BiConsumer<Integer, String> beforeRequest = (id, path) -> {};

    synchronized Object handle(int id, String path, JsonNode body) {
      beforeRequest.accept(id, path);
      if (path.equals("/trajectory/authorize")) {
        authorized.add(id);
        requests.add("authorize:" + id);
      }
      if (path.equals("/capabilities"))
        return Map.of(
            "features",
            List.of(
                "full_state",
                "scheduled_full_state",
                "task_authorization",
                "full_state_pauses",
                "hardware_health"),
            "execution_backend",
            backend);
      if (path.equals("/status"))
        return Map.of(
            "drone_id",
            "cf" + (231 + id),
            "phase",
            phase[id],
            "task_id",
            current[id],
            "connected",
            true,
            "services_ready",
            true,
            "clock_type",
            "fixture",
            "telemetry_age",
            0,
            "sim_time",
            (System.nanoTime() - epoch) / 1e9,
            "health",
            Map.of(
                "battery_voltage",
                voltage[id],
                "rssi",
                radio[id],
                "pm_state",
                0,
                "age",
                healthAge[id],
                "received_at",
                1.0),
            "telemetry",
            Map.of(
                "position",
                closePositions && phase[id].equals("full_state")
                    ? new Vec3(id == 0 ? .24 : .26, 0, .3).array()
                    : position[id].array(),
                "velocity",
                List.of(0, 0, 0)));
      if (path.equals("/trajectory/full_state/start")) {
        if (body.has("pauses")) requests.add("pause:" + id);
        current[id] = "trajectory-" + taskCounter.incrementAndGet();
        tasks.put(current[id], "running");
        phase[id] = "full_state";
        return Map.of("task_id", current[id]);
      }
      if (path.equals("/begin_takeoff")) {
        requests.add("takeoff:" + id);
        position[id] = new Vec3(position[id].x(), position[id].y(), .293);
        phase[id] = "hovering";
        current[id] = "takeoff-" + taskCounter.incrementAndGet();
        tasks.put(current[id], pendingTakeoff[id] ? "running" : "completed");
        if (pendingTakeoff[id]) phase[id] = "taking_off";
        return Map.of("task_id", current[id]);
      }
      if (path.equals("/trajectory/upload")) {
        requests.add(
            "upload-z:" + id + ":" + body.path("pieces").get(0).path("poly_z").get(0).asDouble());
        JsonNode pieces = body.path("pieces"), last = pieces.get(pieces.size() - 1);
        double[] endpoint = new double[3];
        int axis = 0;
        for (String key : List.of("poly_x", "poly_y", "poly_z")) {
          JsonNode coefficients = last.path(key);
          double value = 0;
          for (int k = coefficients.size() - 1; k >= 0; k--)
            value = value * last.path("duration").asDouble() + coefficients.get(k).asDouble();
          endpoint[axis++] = value;
        }
        target[id] = new Vec3(endpoint[0], endpoint[1], endpoint[2]);
        return Map.of("ok", true);
      }
      if (path.startsWith("/tasks/")) {
        String task = path.substring("/tasks/".length());
        if (task.startsWith("trajectory-")
            && System.nanoTime() >= completeNotBefore[id]
            && tasks.get(task).equals("running")
            && (!requirePermission || authorized.contains(id))
            && (!closePositions || id != 0 || authorized.contains(id))
            && (id == 0 || firstLanding.getCount() == 0)) {
          tasks.put(task, "completed");
          phase[id] = "hovering";
          position[id] = target[id];
          requests.add("complete:" + id);
        }
        return Map.of("status", tasks.get(task));
      }
      if (path.equals("/begin_landing")) {
        requests.add("land:" + id);
        if (id == 0) firstLanding.countDown();
        current[id] = "landing-" + taskCounter.incrementAndGet();
        tasks.put(current[id], "completed");
        phase[id] = "landed";
        position[id] = new Vec3(position[id].x(), position[id].y(), .015);
        return Map.of("task_id", current[id]);
      }
      if (path.equals("/trajectory/stop")) {
        requests.add("stop:" + id);
        tasks.put(current[id], "cancelled");
        current[id] = "stop-" + taskCounter.incrementAndGet();
        tasks.put(current[id], "completed");
        phase[id] = "hovering";
        return Map.of("task_id", current[id]);
      }
      return Map.of("ok", true);
    }

    public void close() {
      for (var server : servers) if (server != null) server.stop(0);
    }
  }
}
