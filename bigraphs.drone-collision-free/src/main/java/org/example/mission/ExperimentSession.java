package org.example.mission;

import com.fasterxml.jackson.databind.*;
import java.net.URI;
import java.net.http.*;
import java.time.Duration;
import java.util.*;

/** Owns one event schedule. Flight policy is never sent to the event executor. */
public final class ExperimentSession {
  private final HttpClient http;
  private final ObjectMapper json;
  private final String url;
  private final String runId = UUID.randomUUID().toString();
  private final Scenario.Prepared prepared;
  private double epoch = Double.NaN;
  private JsonNode observation;

  public ExperimentSession(
      String url, HttpClient http, ObjectMapper json, MissionConfig config, Scenario scenario)
      throws Exception {
    this.url = url.replaceAll("/$", "");
    this.http = http;
    this.json = json;
    observation = request("GET", "/experiment/status", null);
    prepared = scenario.prepare(config, observation, json);
  }

  private JsonNode request(String method, String path, Object body) throws Exception {
    var request = HttpRequest.newBuilder(URI.create(url + path)).timeout(Duration.ofSeconds(3));
    if (method.equals("GET")) request.GET();
    else
      request
          .header("Content-Type", "application/json")
          .POST(HttpRequest.BodyPublishers.ofString(json.writeValueAsString(body)));
    var reply = http.send(request.build(), HttpResponse.BodyHandlers.ofString());
    if (reply.statusCode() != 200)
      throw new IllegalStateException(
          "Experiment HTTP " + reply.statusCode() + ": " + reply.body());
    return json.readTree(reply.body());
  }

  public Scenario.Prepared prepared() {
    return prepared;
  }

  public boolean armed() {
    return !Double.isNaN(epoch);
  }

  public JsonNode observe() throws Exception {
    observation = request("GET", "/experiment/status", null);
    return observation;
  }

  public JsonNode observation() {
    return observation;
  }

  public void start(double commonEpoch) throws Exception {
    if (armed()) return;
    epoch = commonEpoch;
    var grid = prepared.grid();
    var winds =
        prepared.events().stream()
            .filter(e -> e.get("type").equals("wind"))
            .map(
                e ->
                    Map.of(
                        "id",
                        e.get("id"),
                        "drone_id",
                        e.get("drone_id"),
                        "offset",
                        e.get("offset"),
                        "ramp",
                        e.get("ramp"),
                        "hold",
                        e.get("hold"),
                        "velocity",
                        e.get("velocity")))
            .toList();
    request(
        "POST",
        "/experiment/arm",
        Map.of(
            "run_id",
            runId,
            "epoch",
            epoch,
            "events",
            winds,
            "grid",
            Map.of(
                "rows",
                grid.rows,
                "cols",
                grid.cols,
                "layers",
                grid.layers,
                "origin",
                grid.origin.array(),
                "step",
                grid.step.array())));
  }

  public List<Scenario.Window> taskWindows(int id, double taskStart) {
    return armed()
        ? prepared.windows(id, epoch).stream().filter(w -> w.endAt() > taskStart).toList()
        : List.of();
  }

  public Vec3 originalReference(int id, double now) {
    return prepared
        .plan()
        .trajectories()
        .get(id)
        .at(Scenario.elapsed(epoch, now, prepared.windows(id, epoch)));
  }

  public void finish() throws Exception {
    if (!armed()) return;
    try {
      observe();
    } finally {
      request("POST", "/experiment/cancel", Map.of("run_id", runId));
    }
  }
}
