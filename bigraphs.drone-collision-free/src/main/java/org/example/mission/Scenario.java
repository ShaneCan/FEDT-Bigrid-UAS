package org.example.mission;

import com.fasterxml.jackson.databind.*;
import java.security.MessageDigest;
import java.util.*;
import org.springframework.boot.context.properties.bind.*;
import org.springframework.core.env.Environment;

/** Pure numeric scene preparation. No file cache, ROS or Bigraph construction. */
public record Scenario(List<Event> events) {
  public record Event(
      String id,
      String type,
      int drone,
      Double at,
      Double centerFraction,
      double ramp,
      double hold,
      String velocity,
      boolean pauseForward,
      Double duration) {
    public Event {
      if (type == null || !Set.of("wind", "pause").contains(type) || drone < 1 || drone > 255)
        throw new IllegalArgumentException("Event requires type wind/pause and a drone ID");
      if ((at == null) == (centerFraction == null)
          || at != null && (!Double.isFinite(at) || at < 0)
          || centerFraction != null
              && (!Double.isFinite(centerFraction) || centerFraction <= 0 || centerFraction >= 1))
        throw new IllegalArgumentException("Event requires at seconds OR center-fraction in (0,1)");
      if (type.equals("wind")) {
        if (duration != null)
          throw new IllegalArgumentException("Wind duration is derived from ramp and hold");
        if (!Double.isFinite(ramp + hold)
            || ramp <= 0
            || hold < 0
            || 2 * ramp + hold > 60
            || vector(velocity).norm() > 1)
          throw new IllegalArgumentException("Invalid wind duration or velocity (maximum 1 m/s)");
      } else {
        if (duration == null || !Double.isFinite(duration) || duration <= 0 || duration > 60)
          throw new IllegalArgumentException("Pause duration must be in (0,60] seconds");
        if (velocity != null || ramp != 0 || hold != 0 || pauseForward)
          throw new IllegalArgumentException(
              "Pause events use duration; wind parameters are not applicable");
      }
    }

    double seconds() {
      return type.equals("wind") ? 2 * ramp + hold : duration;
    }
  }

  public record Window(double startAt, double endAt) {
    public Window {
      if (!Double.isFinite(startAt + endAt) || startAt < 0 || endAt <= startAt)
        throw new IllegalArgumentException("Invalid pause window");
    }

    public Map<String, Double> json() {
      return Map.of("start_at", startAt, "end_at", endAt);
    }
  }

  public record Prepared(
      GridGraph grid,
      CooperativePlanner.Plan plan,
      List<Map<String, Object>> events,
      Map<Integer, List<Window>> pauses,
      Map<String, Object> input,
      String fingerprint) {
    public List<Window> windows(int id, double epoch) {
      return pauses.getOrDefault(id, List.of()).stream()
          .map(w -> new Window(epoch + w.startAt, epoch + w.endAt))
          .toList();
    }
  }

  public Scenario {
    events = List.copyOf(events);
  }

  public static Scenario none() {
    return new Scenario(List.of());
  }

  public static Scenario from(Environment env) {
    if (env.containsProperty("mission.plan")
        || env.containsProperty("experiment.drift.velocity")
        || env.containsProperty("experiment.drift.ramp")
        || env.containsProperty("experiment.drift.hold")
        || env.containsProperty("experiment.drift.pause-forward"))
      throw new IllegalArgumentException(
          "Use experiment.events; mission.plan and experiment.drift.* were removed");
    return new Scenario(
        Binder.get(env).bind("experiment.events", Bindable.listOf(Event.class)).orElse(List.of()));
  }

  public Prepared prepare(MissionConfig c, JsonNode world, ObjectMapper json) throws Exception {
    validateWorld(world, c);
    var grid =
        new GridGraph(
            c.rows(),
            c.cols(),
            c.layers(),
            c.origin(),
            c.step(),
            c.obstacles(),
            GridGraph.rectangular(c.rows(), c.cols(), c.layers(), Set.of()).links());
    Map<Integer, Vec3> initial = new TreeMap<>();
    for (var robot : world.path("robots"))
      initial.put(
          robot.path("id").asInt() - c.firstCf(),
          ControllerFleet.vec(robot.path("initial_position")));
    var goals = c.goals(grid);
    List<CooperativePlanner.Agent> agents = new ArrayList<>();
    for (var e : initial.entrySet()) {
      var p = e.getValue();
      int cell = grid.cell(p);
      agents.add(
          new CooperativePlanner.Agent(
              e.getKey(),
              cell,
              goals.get(e.getKey()),
              new Vec3(p.x(), p.y(), grid.center(cell).z())));
    }
    var planner =
        new CooperativePlanner(
            grid, c.speed(), c.acceleration(), c.margin(), c.horizon(), c.nodeLimit());
    var plan = planner.plan(agents);
    List<Map<String, Object>> resolved = new ArrayList<>();
    Map<Integer, List<Window>> pauses = new TreeMap<>(), winds = new TreeMap<>();
    Set<String> identities = new HashSet<>();
    for (int i = 0; i < events.size(); i++) {
      var event = events.get(i);
      int id = event.drone - c.firstCf();
      if (!initial.containsKey(id))
        throw new IllegalArgumentException("Event targets absent aircraft cf" + event.drone);
      String identity = event.id == null || event.id.isBlank() ? "event-" + i : event.id;
      if (!identities.add(identity))
        throw new IllegalArgumentException("Duplicate event ID: " + identity);
      var curve = plan.trajectories().get(id);
      double seconds = event.seconds();
      double at = event.at != null ? event.at : arcTime(curve, event.centerFraction) - seconds / 2;
      if (at < 0 || at + seconds > curve.duration())
        throw new IllegalArgumentException(
            "Event window must fit the original reference for cf" + event.drone);
      var window = new Window(at, at + seconds);
      Map<String, Object> data = new TreeMap<>();
      data.put("id", identity);
      data.put("type", event.type);
      data.put("drone_id", event.drone);
      data.put("offset", at);
      data.put("duration", seconds);
      if (event.type.equals("wind")) {
        data.put("ramp", event.ramp);
        data.put("hold", event.hold);
        data.put("velocity", vector(event.velocity).array());
        data.put("pause_forward", event.pauseForward);
        winds.computeIfAbsent(id, k -> new ArrayList<>()).add(window);
      }
      if (event.type.equals("pause") || event.pauseForward)
        pauses.computeIfAbsent(id, k -> new ArrayList<>()).add(window);
      resolved.add(Collections.unmodifiableMap(data));
    }
    for (var windows : winds.values()) {
      var sorted = windows.stream().sorted(Comparator.comparingDouble(Window::startAt)).toList();
      for (int i = 1; i < sorted.size(); i++)
        if (sorted.get(i).startAt < sorted.get(i - 1).endAt)
          throw new IllegalArgumentException("Overlapping wind events on one aircraft");
    }
    pauses.replaceAll((id, windows) -> merge(windows));
    resolved.sort(
        Comparator.<Map<String, Object>>comparingDouble(
                e -> ((Number) e.get("offset")).doubleValue())
            .thenComparing(e -> e.get("id").toString()));
    Map<String, Object> input = describe(c, grid, plan, initial, resolved);
    return new Prepared(
        grid, plan, List.copyOf(resolved), Map.copyOf(pauses), input, fingerprint(input, json));
  }

  public static Map<String, Object> describe(
      MissionConfig c,
      GridGraph grid,
      CooperativePlanner.Plan plan,
      Map<Integer, Vec3> initial,
      List<Map<String, Object>> events) {
    Map<String, Object> input = new TreeMap<>();
    input.put(
        "geometry",
        Map.of(
            "rows",
            c.rows(),
            "cols",
            c.cols(),
            "layers",
            c.layers(),
            "origin",
            c.origin().array(),
            "step",
            c.step().array(),
            "obstacles",
            c.obstacles().stream().sorted().toList()));
    input.put(
        "links",
        grid.links().stream()
            .sorted(
                Comparator.comparingInt(GridGraph.Edge::from)
                    .thenComparingInt(GridGraph.Edge::to)
                    .thenComparing(GridGraph.Edge::route))
            .toList());
    input.put("initial_positions", initial);
    input.put("goals", c.goals(grid));
    input.put(
        "planner",
        Map.of(
            "speed",
            c.speed(),
            "acceleration",
            c.acceleration(),
            "margin",
            c.margin(),
            "horizon",
            c.horizon(),
            "node_limit",
            c.nodeLimit(),
            "slot",
            plan.slot()));
    Map<String, Object> trajectories = new TreeMap<>();
    new TreeMap<>(plan.trajectories())
        .forEach(
            (id, curve) ->
                trajectories.put(
                    "" + id,
                    Map.of(
                        "path", curve.path, "start", curve.at(0).array(), "pieces", curve.json())));
    input.put("trajectories", trajectories);
    input.put("events", List.copyOf(events));
    return Collections.unmodifiableMap(input);
  }

  public static String fingerprint(Map<String, Object> input, ObjectMapper json) throws Exception {
    byte[] canonical =
        json.copy().enable(SerializationFeature.ORDER_MAP_ENTRIES_BY_KEYS).writeValueAsBytes(input);
    return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(canonical));
  }

  public static List<Window> merge(Collection<Window> windows) {
    List<Window> out = new ArrayList<>();
    for (var w : windows.stream().sorted(Comparator.comparingDouble(Window::startAt)).toList()) {
      if (!out.isEmpty() && w.startAt <= out.getLast().endAt) {
        var previous = out.removeLast();
        out.add(new Window(previous.startAt, Math.max(previous.endAt, w.endAt)));
      } else out.add(w);
    }
    return List.copyOf(out);
  }

  public static double elapsed(double start, double now, List<Window> windows) {
    double blocked =
        windows.stream()
            .mapToDouble(w -> Math.max(0, Math.min(now, w.endAt) - Math.max(start, w.startAt)))
            .sum();
    return Math.max(0, now - start - blocked);
  }

  public static void validateWorld(JsonNode world, MissionConfig c) {
    if (!world.path("backend").asText().equals("webots"))
      throw new IllegalStateException("Scenario events require Webots; refusing hardware");
    var actual = new TreeSet<Integer>();
    for (var robot : world.path("robots"))
      if (!actual.add(robot.path("id").asInt()))
        throw new IllegalArgumentException("Duplicate aircraft in world metadata");
    var expected = new TreeSet<Integer>();
    for (int i = 0; i < c.count(); i++) expected.add(c.firstCf() + i);
    if (!actual.equals(expected))
      throw new IllegalStateException(
          "World contains "
              + actual
              + "; scenario expects "
              + expected
              + ". Edit the world manually or select the matching scene profile; no world is"
              + " changed automatically.");
    if (!world.path("drivers_ready").asBoolean())
      throw new IllegalStateException("Scenario drivers are not ready");
  }

  private static Vec3 vector(String text) {
    if (text == null) throw new IllegalArgumentException("Wind requires velocity x,y,z");
    String[] p = text.split(",");
    if (p.length != 3) throw new IllegalArgumentException("Wind velocity requires three axes");
    return new Vec3(
        Double.parseDouble(p[0].trim()),
        Double.parseDouble(p[1].trim()),
        Double.parseDouble(p[2].trim()));
  }

  static double arcTime(PolynomialTrajectory curve, double fraction) {
    int steps = (int) Math.ceil(curve.duration() / .02);
    double[] lengths = new double[steps + 1];
    Vec3 previous = curve.at(0);
    for (int i = 1; i <= steps; i++) {
      Vec3 next = curve.at(Math.min(curve.duration(), i * .02));
      lengths[i] = lengths[i - 1] + next.distance(previous);
      previous = next;
    }
    double target = lengths[steps] * fraction;
    if (target <= 0)
      throw new IllegalArgumentException("Route fraction requires a moving reference");
    for (int i = 1; i <= steps; i++)
      if (lengths[i] >= target)
        return (i - 1 + (target - lengths[i - 1]) / (lengths[i] - lengths[i - 1])) * .02;
    throw new IllegalStateException("Arc length calculation failed");
  }
}
