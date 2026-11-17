package org.example.mission;

import com.fasterxml.jackson.databind.*;
import java.net.URI;
import java.net.http.*;
import java.time.Duration;
import java.util.*;
import java.util.concurrent.*;

/** Every fleet batch dispatches before waiting for any response. */
public final class ControllerFleet implements AutoCloseable {
  public record Status(
      int id,
      String drone,
      String phase,
      String task,
      boolean connected,
      boolean services,
      double time,
      double age,
      Vec3 position,
      Vec3 velocity,
      JsonNode json) {}

  public final ConcurrentMap<Integer, Queue<Double>> restMs = new ConcurrentHashMap<>();
  public volatile long firstTelemetryWall, lastTelemetryWall;
  private final MissionConfig config;
  private final ObjectMapper json;
  private final ExecutorService executor = Executors.newVirtualThreadPerTaskExecutor();
  public final HttpClient http =
      HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(3)).build();

  public ControllerFleet(MissionConfig config, ObjectMapper json) {
    this.config = config;
    this.json = json;
  }

  public JsonNode request(int id, String method, String path, Object payload) throws Exception {
    var b =
        HttpRequest.newBuilder(URI.create(config.controlUrl() + ":" + (config.port() + id) + path))
            .timeout(Duration.ofSeconds(path.endsWith("upload") ? 15 : 2));
    if (method.equals("GET")) b.GET();
    else
      b.header("Content-Type", "application/json")
          .POST(HttpRequest.BodyPublishers.ofString(json.writeValueAsString(payload)));
    long begin = System.nanoTime();
    var r = http.send(b.build(), HttpResponse.BodyHandlers.ofString());
    if (method.equals("POST"))
      restMs
          .computeIfAbsent(id, k -> new ConcurrentLinkedQueue<>())
          .add((System.nanoTime() - begin) / 1e6);
    if (r.statusCode() != 200)
      throw new IllegalStateException(
          "D" + id + " " + path + " HTTP " + r.statusCode() + ": " + r.body());
    return json.readTree(r.body());
  }

  public <T> Map<Integer, T> parallel(Collection<Integer> ids, CheckedCall<T> operation)
      throws Exception {
    Map<Integer, Future<T>> futures = new TreeMap<>();
    for (int id : ids) futures.put(id, executor.submit(() -> operation.call(id)));
    Map<Integer, T> results = new TreeMap<>();
    Exception failure = null;
    for (var e : futures.entrySet())
      try {
        results.put(e.getKey(), e.getValue().get());
      } catch (ExecutionException ex) {
        if (failure == null)
          failure =
              new IllegalStateException(
                  "Fleet operation failed for D" + e.getKey() + ": " + ex.getCause().getMessage(),
                  ex.getCause());
        else failure.addSuppressed(ex.getCause());
      }
    if (failure != null) throw failure;
    return results;
  }

  @FunctionalInterface
  public interface CheckedCall<T> {
    T call(int id) throws Exception;
  }

  public Map<Integer, Status> statuses(Collection<Integer> ids) throws Exception {
    return parallel(ids, id -> status(id));
  }

  public Status status(int id) throws Exception {
    JsonNode s = request(id, "GET", "/status", null), t = s.path("telemetry");
    if (s.path("connected").asBoolean()) {
      long now = System.currentTimeMillis();
      if (firstTelemetryWall == 0) firstTelemetryWall = now;
      lastTelemetryWall = now;
    }
    return new Status(
        id,
        s.path("drone_id").asText(),
        s.path("phase").asText(),
        s.path("task_id").asText(),
        s.path("connected").asBoolean(),
        s.path("services_ready").asBoolean(),
        s.path("sim_time").asDouble(Double.NaN),
        s.path("telemetry_age").asDouble(Double.POSITIVE_INFINITY),
        vec(t.path("position")),
        vec(t.path("velocity")),
        s);
  }

  public JsonNode post(int id, String path, Object payload) throws Exception {
    return request(id, "POST", path, payload);
  }

  public JsonNode task(int id, String task) throws Exception {
    return request(id, "GET", "/tasks/" + task, null);
  }

  public static Vec3 vec(JsonNode n) {
    if (!n.isArray() || n.size() != 3)
      throw new IllegalStateException("Missing position/velocity telemetry");
    return new Vec3(n.get(0).asDouble(), n.get(1).asDouble(), n.get(2).asDouble());
  }

  public void waitTasks(Map<Integer, String> tasks, double budget) throws Exception {
    Map<Integer, Status> start = statuses(tasks.keySet());
    long wall = System.nanoTime();
    Set<Integer> pending = new TreeSet<>(tasks.keySet());
    while (!pending.isEmpty()) {
      var states = statuses(pending);
      var taskStates = parallel(pending, id -> task(id, tasks.get(id)));
      for (int id : new ArrayList<>(pending)) {
        JsonNode t = taskStates.get(id);
        String state = t.path("status").asText();
        if (state.equals("completed")) {
          if (states.get(id).velocity.norm() > .15)
            throw new IllegalStateException("Task completed while aircraft is moving");
          pending.remove(id);
        } else if (state.equals("failed") || state.equals("cancelled"))
          throw new IllegalStateException("D" + id + " task failed: " + t);
        if (states.get(id).time - start.get(id).time > budget
            || System.nanoTime() - wall > 600_000_000_000L)
          throw new TimeoutException("Task readiness timeout");
      }
      if (!pending.isEmpty()) TimeUnit.MILLISECONDS.sleep(30);
    }
  }

  public void waitHover(Collection<Integer> ids, Map<Integer, Vec3> targets) throws Exception {
    if (ids.isEmpty()) return;
    Map<Integer, Status> initial = statuses(ids);
    Map<Integer, Double> settled = new HashMap<>();
    Set<Integer> pending = new TreeSet<>(ids);
    long wall = System.nanoTime();
    while (!pending.isEmpty()) {
      var states = statuses(pending);
      for (int id : new ArrayList<>(pending)) {
        var s = states.get(id);
        boolean ready =
            s.connected
                && s.age <= .3
                && s.phase.equals("hovering")
                && s.velocity.norm() <= .03
                && (targets == null
                    || !targets.containsKey(id)
                    || s.position.distance(targets.get(id)) <= .02);
        if (ready) {
          settled.putIfAbsent(id, s.time);
          if (s.time - settled.get(id) >= .2) pending.remove(id);
        } else settled.remove(id);
        if (s.time - initial.get(id).time > 30 || System.nanoTime() - wall > 600_000_000_000L)
          throw new TimeoutException("D" + id + " did not reach a stable hover");
      }
      if (!pending.isEmpty()) TimeUnit.MILLISECONDS.sleep(30);
    }
  }

  public void close() {
    executor.shutdownNow();
    http.close();
  }
}
