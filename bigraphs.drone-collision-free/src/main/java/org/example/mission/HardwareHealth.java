package org.example.mission;

import com.fasterxml.jackson.databind.*;
import com.sun.net.httpserver.*;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.*;
import java.util.function.Consumer;
import org.springframework.core.env.Environment;

/** Hardware Status observations and local overrides; this class never commands a drone. */
public final class HardwareHealth implements AutoCloseable {
  public record Config(
      double low,
      double empty,
      int badRssi,
      double timeout,
      double startupTimeout,
      boolean injection,
      String bind,
      int port,
      Set<Integer> stations,
      boolean required) {
    public Config {
      if (!Double.isFinite(low + empty + timeout + startupTimeout)
          || empty <= 0
          || low <= empty
          || badRssi < 1
          || badRssi > 255
          || timeout <= 0
          || startupTimeout < 0
          || port < 0
          || port > 65535)
        throw new IllegalArgumentException("Invalid hardware health configuration");
      stations = Set.copyOf(stations);
    }

    public static Config defaults() {
      return from(new org.springframework.core.env.StandardEnvironment());
    }

    public static Config from(Environment e) {
      Set<Integer> stations = new TreeSet<>();
      for (String s : e.getProperty("charging.station.grids", "3").split(","))
        if (!s.isBlank()) stations.add(Integer.parseInt(s.trim()));
      return new Config(
          e.getProperty("hardware.health.battery-low", Double.class, 3.2),
          e.getProperty("hardware.health.battery-empty", Double.class, 3.1),
          e.getProperty("hardware.health.rssi-bad", Integer.class, 65),
          e.getProperty("hardware.health.status-timeout", Double.class, 3.0),
          e.getProperty("hardware.health.startup-timeout", Double.class, 10.0),
          e.getProperty("fault.injection.enabled", Boolean.class, true),
          e.getProperty("fault.injection.bind", "127.0.0.1"),
          e.getProperty("fault.injection.port", Integer.class, 8090),
          stations,
          e.getProperty("hardware.health.required", Boolean.class, false));
    }
  }

  public record View(
      String battery,
      String communication,
      Double rawVoltage,
      Integer rawRssi,
      Integer pmState,
      Double voltage,
      Integer rssi,
      double age,
      Double receivedAt,
      boolean batteryInjected,
      boolean commInjected) {
    public boolean fresh() {
      return !battery.equals("Unknown") && !communication.equals("Unknown");
    }

    public boolean eligible() {
      return battery.equals("Normal") && communication.equals("Normal");
    }
  }

  private record Reading(
      double voltage, int rssi, int pm, double age, Double receivedAt, long readAt) {}

  private final Config config;
  private final Set<Integer> ids;
  private final ObjectMapper json;
  private final Consumer<Map<String, Object>> events;
  private final Map<Integer, Reading> readings = new HashMap<>();
  private final Map<Integer, Double> voltageOverrides = new HashMap<>();
  private final Map<Integer, Integer> radioOverrides = new HashMap<>();
  private final Map<Integer, String> lastState = new HashMap<>();
  private HttpServer server;
  private ExecutorService executor;

  public HardwareHealth(
      Config config, Set<Integer> ids, ObjectMapper json, Consumer<Map<String, Object>> events) {
    this.config = config;
    this.ids = Set.copyOf(ids);
    this.json = json;
    this.events = events;
  }

  public Config config() {
    return config;
  }

  public synchronized void update(Map<Integer, ControllerFleet.Status> states) {
    for (var s : states.values()) {
      JsonNode h = s.json().path("health");
      if (h.isObject()
          && h.path("battery_voltage").isNumber()
          && h.path("rssi").isIntegralNumber()
          && h.path("pm_state").isIntegralNumber()
          && h.path("age").isNumber()) {
        double v = h.path("battery_voltage").asDouble(), age = h.path("age").asDouble();
        int r = h.path("rssi").asInt();
        if (Double.isFinite(v + age) && v > 0 && age >= 0 && r >= 0 && r <= 255)
          readings.put(
              s.id(),
              new Reading(
                  v,
                  r,
                  h.path("pm_state").asInt(),
                  age,
                  h.path("received_at").isNumber() ? h.path("received_at").asDouble() : null,
                  System.nanoTime()));
        else readings.remove(s.id());
      } else readings.remove(s.id());
      View v = view(s.id());
      String signature = v.battery() + "/" + v.communication();
      if (!signature.equals(lastState.put(s.id(), signature)))
        events.accept(
            Map.of(
                "event",
                "health_changed",
                "drone",
                s.id(),
                "health",
                v,
                "observed_at",
                java.time.Instant.now().toString()));
    }
  }

  public synchronized View view(int id) {
    if (!ids.contains(id)) throw new IllegalArgumentException("Unknown drone D" + id);
    Reading r = readings.get(id);
    double age =
        r == null ? Double.POSITIVE_INFINITY : r.age + (System.nanoTime() - r.readAt) / 1e9;
    Double voltage = voltageOverrides.getOrDefault(id, r == null ? null : r.voltage);
    Integer radio = radioOverrides.getOrDefault(id, r == null ? null : r.rssi);
    boolean known = r != null && age <= config.timeout;
    return new View(
        !known
            ? "Unknown"
            : voltage < config.empty ? "Empty" : voltage < config.low ? "Low" : "Normal",
        !known ? "Unknown" : radio >= config.badRssi ? "Bad" : "Normal",
        r == null ? null : r.voltage,
        r == null ? null : r.rssi,
        r == null ? null : r.pm,
        voltage,
        radio,
        age,
        r == null ? null : r.receivedAt,
        voltageOverrides.containsKey(id),
        radioOverrides.containsKey(id));
  }

  public synchronized Map<Integer, View> snapshot() {
    Map<Integer, View> result = new TreeMap<>();
    for (int id : ids) result.put(id, view(id));
    return result;
  }

  public synchronized void inject(int id, boolean battery, String text) {
    if (!ids.contains(id)) throw new NoSuchElementException("Unknown drone D" + id);
    String value = text.toLowerCase(Locale.ROOT);
    if (Set.of("clear", "off", "none").contains(value)) {
      if (battery) voltageOverrides.remove(id);
      else radioOverrides.remove(id);
    } else if (battery) {
      double v =
          switch (value) {
            case "normal" -> config.low + .2;
            case "low" -> (config.empty + config.low) / 2;
            case "empty" -> config.empty / 2;
            default -> Double.parseDouble(value);
          };
      if (!Double.isFinite(v) || v <= 0)
        throw new IllegalArgumentException("Battery voltage must be finite and positive");
      voltageOverrides.put(id, v);
    } else {
      int r =
          switch (value) {
            case "normal" -> Math.max(0, config.badRssi - 15);
            case "bad" -> config.badRssi;
            default -> Integer.parseInt(value);
          };
      if (r < 0 || r > 255) throw new IllegalArgumentException("RSSI must be 0..255");
      radioOverrides.put(id, r);
    }
    events.accept(
        Map.of(
            "event",
            "health_injection",
            "observed_at",
            java.time.Instant.now().toString(),
            "drone",
            id,
            "field",
            battery ? "battery" : "comm",
            "value",
            text,
            "health",
            view(id)));
  }

  public synchronized void start() throws IOException {
    if (!config.injection || server != null) return;
    server = HttpServer.create(new InetSocketAddress(config.bind, config.port), 0);
    executor = Executors.newVirtualThreadPerTaskExecutor();
    server.setExecutor(executor);
    server.createContext(
        "/inject/",
        exchange -> {
          int code = 200;
          String message;
          try {
            if (!exchange.getRequestMethod().equals("GET")) {
              code = 405;
              message = "Use GET";
            } else if (exchange.getRequestURI().getPath().equals("/inject/status")) {
              Map<String, View> views = new TreeMap<>();
              snapshot().forEach((id, view) -> views.put("D" + id, view));
              message = json.writeValueAsString(views);
            } else {
              String path = exchange.getRequestURI().getPath();
              if (!Set.of("/inject/battery", "/inject/comm").contains(path))
                throw new NoSuchElementException("Unknown endpoint");
              Map<String, String> q = query(exchange.getRequestURI().getRawQuery());
              String drone = q.get("drone"),
                  v = q.getOrDefault(path.endsWith("battery") ? "level" : "state", q.get("value"));
              if (drone == null || !drone.matches("D[0-9]+") || v == null)
                throw new IllegalArgumentException("Need drone=D<n> and level/state/value");
              int id = Integer.parseInt(drone.substring(1));
              inject(id, path.endsWith("battery"), v);
              message = json.writeValueAsString(view(id));
            }
          } catch (NoSuchElementException ex) {
            code = 404;
            message = ex.getMessage();
          } catch (IllegalArgumentException ex) {
            code = 400;
            message = ex.getMessage();
          }
          if (code != 200) message = json.writeValueAsString(Map.of("error", message));
          byte[] bytes = message.getBytes(StandardCharsets.UTF_8);
          exchange.getResponseHeaders().set("Content-Type", "application/json; charset=utf-8");
          exchange.sendResponseHeaders(code, bytes.length);
          try (var out = exchange.getResponseBody()) {
            out.write(bytes);
          }
        });
    server.start();
    System.out.println(
        "Hardware fault injection: http://"
            + config.bind
            + ":"
            + server.getAddress().getPort()
            + "/inject/status");
  }

  public synchronized int port() {
    return server == null ? -1 : server.getAddress().getPort();
  }

  private static Map<String, String> query(String raw) {
    Map<String, String> result = new HashMap<>();
    if (raw != null)
      for (String pair : raw.split("&")) {
        String[] parts = pair.split("=", 2);
        if (parts.length == 2)
          result.put(
              URLDecoder.decode(parts[0], StandardCharsets.UTF_8),
              URLDecoder.decode(parts[1], StandardCharsets.UTF_8));
      }
    return result;
  }

  public synchronized void close() {
    if (server != null) {
      server.stop(0);
      server = null;
    }
    if (executor != null) {
      executor.shutdownNow();
      executor = null;
    }
  }
}
