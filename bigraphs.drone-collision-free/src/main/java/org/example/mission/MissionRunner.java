package org.example.mission;

import com.fasterxml.jackson.databind.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

/** Whole-mission dispatch. Monitoring remains live while affected agents replan. */
public final class MissionRunner implements AutoCloseable {
  private static final class Active {
    final int id;
    final String task;
    final PolynomialTrajectory curve;
    final double epoch;
    final List<Scenario.Window> pauses;
    final List<PolynomialTrajectory.Crossing> crossings;
    final AtomicBoolean failed = new AtomicBoolean();
    volatile boolean startRecorded;
    volatile Integer observedCell;
    volatile int nextCrossing;

    Active(int id, String task, PolynomialTrajectory curve, double epoch, GridGraph grid) {
      this(id, task, curve, epoch, grid, List.of());
    }

    Active(
        int id,
        String task,
        PolynomialTrajectory curve,
        double epoch,
        GridGraph grid,
        List<Scenario.Window> pauses) {
      this.id = id;
      this.task = task;
      this.curve = curve;
      this.epoch = epoch;
      this.pauses = pauses;
      crossings = curve.crossings(grid);
      observedCell = curve.path.getFirst();
    }

    double elapsed(double now) {
      return Scenario.elapsed(epoch, now, pauses);
    }
  }

  private record Fault(int id, String task, String reason) {}

  private record Landing(String task, double startedAt, long wallStart, double budget) {}

  private final MissionConfig config;
  private final RunOptions options;
  private ExperimentSession experiment;
  private Map<String, Object> initialInput;
  private String inputFingerprint;
  private volatile JsonNode experimentStatus;
  private final AtomicBoolean gridConflict = new AtomicBoolean();
  private final AtomicInteger matcherChecks = new AtomicInteger(),
      matchDenials = new AtomicInteger(),
      commonFaults = new AtomicInteger();
  private final AtomicInteger authorizationExpiries = new AtomicInteger();
  private final Queue<String> samples = new ConcurrentLinkedQueue<>();

  private final ObjectMapper json = new ObjectMapper();
  private final ControllerFleet fleet;
  private final Set<Integer> ids = new TreeSet<>();
  private final ConcurrentMap<Integer, Active> active = new ConcurrentHashMap<>();
  private final BlockingQueue<Fault> faults = new LinkedBlockingQueue<>();
  private final BlockingQueue<Integer> arrivals = new LinkedBlockingQueue<>();
  private final Set<Integer> awaitingDescent = new TreeSet<>(), landed = new TreeSet<>();
  private final Map<Integer, Landing> landing = new TreeMap<>();
  private final ScheduledExecutorService monitor = Executors.newSingleThreadScheduledExecutor();
  private final AtomicBoolean closed = new AtomicBoolean();
  private final AtomicLong snapshotVersion = new AtomicLong();
  private final Queue<Double> planningMs = new ConcurrentLinkedQueue<>();
  private final ConcurrentMap<Integer, Queue<Double>> matchByDrone = new ConcurrentHashMap<>();
  private final ConcurrentMap<String, ConcurrentMap<Integer, Double>> actualStarts =
      new ConcurrentHashMap<>();
  private final AtomicInteger coOccupancy = new AtomicInteger();
  private final Queue<Double> matchMs = new ConcurrentLinkedQueue<>(),
      cycleMs = new ConcurrentLinkedQueue<>();
  private final Queue<Map<String, Object>> events = new ConcurrentLinkedQueue<>();
  private volatile Map<Integer, ControllerFleet.Status> latest = Map.of();
  private volatile double minDistance = Double.POSITIVE_INFINITY;
  private final DoubleAccumulator maxTrackingError = new DoubleAccumulator(Math::max, 0);
  private GridGraph grid;
  private final Map<Integer, BigraphGate> gates = new TreeMap<>();
  private CooperativePlanner planner;
  private SnapshotStore store;
  private RosMonitor ros;
  private HardwareHealth health;
  private HealthGate healthGate;
  private final Set<Integer> participants = new TreeSet<>(),
      skipped = ConcurrentHashMap.newKeySet();
  private final Set<Integer> emergencyFailed = ConcurrentHashMap.newKeySet(),
      localLandings = ConcurrentHashMap.newKeySet();
  private final Map<Integer, Integer> homes = new TreeMap<>(), originalGoals = new TreeMap<>();
  private final ConcurrentMap<Integer, EmergencyPolicy.Action> emergencies =
      new ConcurrentHashMap<>();
  private final Map<Integer, Integer> resumeGoals = new ConcurrentHashMap<>();
  private final Queue<Integer> relocations = new ConcurrentLinkedQueue<>();
  private final Map<Integer, Double> emergencyWaiting = new HashMap<>();
  private final Queue<Double> healthMatchMs = new ConcurrentLinkedQueue<>();
  private volatile long nextHealthPoll;

  private final Map<Integer, Integer> goals = new ConcurrentHashMap<>();
  private int replans, expanded;
  private final Thread shutdownHook = new Thread(this::close, "cooperative-shutdown");

  public MissionRunner(MissionConfig config) {
    this(config, RunOptions.normal());
  }

  public MissionRunner(MissionConfig config, RunOptions options) {
    this.config = config;
    this.options = options;
    fleet = new ControllerFleet(config, json);
    for (int i = 0; i < config.count(); i++) ids.add(i);
    participants.addAll(ids);
    Runtime.getRuntime().addShutdownHook(shutdownHook);
  }

  public void run() throws Exception {
    long began = System.nanoTime();
    boolean completed = false;
    try {
      if (ids.isEmpty()) throw new IllegalArgumentException("drone.count must be positive");
      if (options.experiment()) {
        experiment =
            new ExperimentSession(
                options.experimentUrl(), fleet.http, json, config, options.scenario());
        experimentStatus = experiment.observation();
        grid = experiment.prepared().grid();
      } else {
        grid = GridService.load(config, fleet.http, json);
      }
      goals.putAll(config.goals(grid));
      originalGoals.putAll(goals);
      validateLandingColumns();
      if (options.matching())
        for (int id : ids) gates.put(id, new BigraphGate(grid, config.count()));
      planner =
          new CooperativePlanner(
              grid,
              config.speed(),
              config.acceleration(),
              config.margin(),
              config.horizon(),
              config.nodeLimit());
      var capabilities =
          fleet.parallel(
              ids,
              id -> {
                JsonNode caps = fleet.request(id, "GET", "/capabilities", null);
                for (String feature :
                    options.matching()
                        ? List.of("full_state", "scheduled_full_state", "task_authorization")
                        : List.of("full_state", "scheduled_full_state"))
                  if (!caps.path("features").toString().contains("\"" + feature + "\""))
                    throw new IllegalStateException(
                        "Controller lacks "
                            + feature
                            + "; restart cf.PyControl from updated source");
                if (options.experiment()
                    && !caps.path("execution_backend").asText().equals("webots"))
                  throw new IllegalStateException(
                      "Scenario events require Webots; refusing hardware");
                if (experiment != null
                    && !experiment.prepared().pauses().getOrDefault(id, List.of()).isEmpty()
                    && !caps.path("features").toString().contains("\"full_state_pauses\""))
                  throw new IllegalStateException(
                      "Controller lacks full_state_pauses; restart cf.PyControl");
                return caps;
              });
      latest = fleet.statuses(ids);
      validate(latest);
      initializeHardware(capabilities);
      for (var state : latest.values())
        if (!Set.of("active", "idle", "landed", "hovering").contains(state.phase()))
          throw new IllegalStateException("D" + state.id() + " is already executing another task");
      // Ground-to-layer-center takeoff is vertical inside the same cell.
      List<CooperativePlanner.Agent> projected = new ArrayList<>();
      for (var state : latest.values()) {
        int cell = grid.cell(state.position());
        homes.put(state.id(), grid.index(grid.x(cell), grid.y(cell), 0));
        if (!participants.contains(state.id())) continue;
        Vec3 hover =
            state.phase().equals("hovering")
                ? state.position()
                : new Vec3(state.position().x(), state.position().y(), grid.center(cell).z());
        var b = grid.box(cell);
        if (state.position().z() < config.origin().z()
            || !b.contains(
                new Vec3(hover.x(), hover.y(), b.lower().z() + config.margin()), config.margin())
            || !b.contains(hover, config.margin()))
          throw new IllegalStateException(
              "Takeoff/hover envelope exceeds grid cell for D" + state.id());
        projected.add(new CooperativePlanner.Agent(state.id(), cell, goals.get(state.id()), hover));
      }
      if (participants.isEmpty()) {
        completed = true;
        System.out.println("MISSION ENDED: no hardware aircraft passed preflight");
        return;
      }
      var preplan =
          experiment == null
              ? timedPlan(projected, slot -> fixedCells(participants, latest))
              : experiment.prepared().plan();
      if (experiment != null)
        for (var agent : projected)
          if (agent.position().distance(preplan.trajectories().get(agent.id()).at(0)) > .02)
            throw new IllegalStateException(
                "D" + agent.id() + " differs from its declared world start");
      if (experiment == null) {
        Map<Integer, Vec3> starts = new TreeMap<>();
        for (var agent : projected) starts.put(agent.id(), agent.position());
        initialInput = Scenario.describe(config, grid, preplan, starts, List.of());
      } else initialInput = experiment.prepared().input();
      inputFingerprint = Scenario.fingerprint(initialInput, json);
      System.out.println("INPUT_FINGERPRINT=" + inputFingerprint);
      expanded += preplan.expanded();
      for (var entry : new TreeMap<>(preplan.trajectories()).entrySet()) {
        var curve = entry.getValue();
        System.out.printf(
            Locale.ROOT,
            "PLAN D%d: duration=%.3fs planned_hold=%.3fs interior_stops=%d start=%s target=%s%n",
            entry.getKey(),
            curve.duration(),
            curve.stationarySeconds(),
            curve.interiorStops(),
            curve.at(0),
            curve.at(curve.duration()));
      }
      if (options.matching()) benchmark(projected, preplan);
      System.out.printf(
          Locale.ROOT,
          "PREFLIGHT: %d aircraft, %.3f m/s, margin %.3f m, slot %.3f s, all grid corridors"
              + " certified%n",
          ids.size(),
          config.speed(),
          config.margin(),
          planner.slot);
      if (config.rosEnabled())
        try {
          ros = new RosMonitor(config.rosHost(), config.firstCf(), json);
        } catch (Exception e) {
          System.err.println(
              "ROSBridge monitor unavailable; control uses controller ROS telemetry: "
                  + e.getMessage());
        }
      if (options.matching() && config.persist())
        store = new SnapshotStore(new BigraphGate(grid, config.count()), config.cdoUrl());
      latest = fleet.statuses(ids);
      validate(latest);
      if (health != null) health.update(latest);
      for (int id : participants) {
        var expected = preplan.trajectories().get(id).at(0);
        var state = latest.get(id);
        if (grid.cell(state.position()) != preplan.trajectories().get(id).path.getFirst()
            || Math.hypot(state.position().x() - expected.x(), state.position().y() - expected.y())
                > .02)
          throw new IllegalStateException("D" + id + " moved during preflight; refusing takeoff");
      }
      Set<Integer> takeoff = new TreeSet<>();
      for (var s : latest.values())
        if (participants.contains(s.id()) && !s.phase().equals("hovering")) takeoff.add(s.id());
      fleet.parallel(takeoff, id -> fleet.post(id, "/activate_idle", Map.of()));
      var takeoffPlan = preplan;
      Map<Integer, String> tasks =
          fleet.parallel(
              takeoff,
              id -> {
                if (health != null) {
                  health.update(Map.of(id, fleet.status(id)));
                  if (!health.view(id).eligible()
                      || !healthAllowed(id, EmergencyPolicy.Action.NORMAL, "takeoff")) {
                    skip(id, "health_changed_before_takeoff");
                    return "";
                  }
                }
                String task =
                    fleet
                        .post(
                            id,
                            "/begin_takeoff",
                            Map.of(
                                "height",
                                grid.center(grid.cell(latest.get(id).position())).z(),
                                "duration",
                                3))
                        .path("task_id")
                        .asText();
                // Record each acceptance before waiting for other aircraft responses.
                active.put(id, new Active(id, task, takeoffPlan.trajectories().get(id), 0, grid));
                return task;
              });
      tasks.values().removeIf(String::isEmpty);
      participants.removeAll(skipped);
      if (health == null) fleet.waitTasks(tasks, 45);
      else waitHardwareTakeoffs(tasks);
      Map<Integer, Vec3> takeoffTargets = new TreeMap<>();
      for (var agent : projected)
        if (takeoff.contains(agent.id())
            && participants.contains(agent.id())
            && !emergencies.containsKey(agent.id()))
          takeoffTargets.put(agent.id(), agent.position());
      if (health == null) {
        Set<Integer> hoverIds = new TreeSet<>(participants);
        hoverIds.removeAll(emergencies.keySet());
        fleet.waitHover(hoverIds, takeoffTargets);
      }
      for (int id : tasks.keySet()) active.remove(id);
      latest = fleet.statuses(ids);
      validate(latest);
      // Dispatch the plan certified before takeoff. Tiny settling errors must not
      // insert another planning slot or change the jointly agreed flight timing.
      for (int id : participants) {
        if (emergencies.containsKey(id)) continue;
        var curve = preplan.trajectories().get(id);
        Vec3 position = latest.get(id).position();
        if (position.distance(curve.at(0)) > .02
            || grid.cell(position) != curve.path.getFirst()
            || !grid.box(curve.path.getFirst()).contains(position, config.margin()))
          throw new IllegalStateException("D" + id + " did not settle at the prepared start");
      }
      monitor.scheduleWithFixedDelay(this::tick, 0, 70, TimeUnit.MILLISECONDS);
      if (health != null) advanceLocalLandings();
      Set<Integer> flightIds = new TreeSet<>(participants);
      flightIds.removeAll(active.keySet());
      flightIds.removeAll(resumeGoals.keySet());
      flightIds.removeAll(new HashSet<>(relocations));
      flightIds.removeAll(localLandings);
      flightIds.removeAll(landing.keySet());
      flightIds.removeAll(emergencyFailed);
      flightIds.removeAll(landed);
      if (!flightIds.equals(preplan.trajectories().keySet()) || !emergencies.isEmpty()) {
        preplan =
            flightIds.isEmpty()
                ? null
                : timedPlan(agents(flightIds, latest), slot -> fixedCells(flightIds, latest));
        if (preplan != null) {
          Map<Integer, Vec3> starts = new TreeMap<>();
          for (int id : flightIds) starts.put(id, latest.get(id).position());
          initialInput = Scenario.describe(config, grid, preplan, starts, List.of());
          inputFingerprint = Scenario.fingerprint(initialInput, json);
          System.out.println("INPUT_FINGERPRINT=" + inputFingerprint);
        }
      }
      var plan = preplan;
      if (plan != null) start(plan.trajectories());
      awaitMissions();
      latest = fleet.statuses(ids);
      validate(latest);
      for (var s : latest.values())
        if (!skipped.contains(s.id())
            && !emergencyFailed.contains(s.id())
            && grid.cell(s.position()) != goals.get(s.id()))
          throw new IllegalStateException("D" + s.id() + " missed target cell");
      monitor.shutdown();
      monitor.awaitTermination(5, TimeUnit.SECONDS);
      completed = true;
      System.out.println(
          "MISSION ENDED: landed="
              + landed
              + " skipped="
              + skipped
              + " unconfirmed="
              + emergencyFailed);
    } finally {
      monitor.shutdownNow();
      if (!completed)
        try {
          stopOwned(new TreeSet<>(active.keySet()));
        } catch (Exception stopFailure) {
          commonFaults.incrementAndGet();
          events.add(Map.of("event", "cleanup_stop_unconfirmed", "reason", stopFailure.toString()));
        }
      if (health != null) health.close();
      if (experiment != null)
        try {
          experiment.finish();
          experimentStatus = experiment.observation();
        } catch (Exception failure) {
          events.add(Map.of("event", "experiment_cleanup_failure", "reason", failure.toString()));
        }
      writeReport(completed, (System.nanoTime() - began) / 1e9);
    }
  }

  private void initializeHardware(Map<Integer, JsonNode> capabilities) throws Exception {
    Set<String> backends = new HashSet<>();
    for (var cap : capabilities.values()) backends.add(cap.path("execution_backend").asText());
    if (backends.size() != 1 || (!backends.contains("hardware") && !backends.contains("webots")))
      throw new IllegalStateException("A mission requires one explicit hardware or Webots backend");
    if (!backends.contains("hardware")) {
      if (options.health().required())
        throw new IllegalStateException("Hardware profile refuses a non-hardware controller");
      return;
    }
    for (var cap : capabilities.values())
      if (!cap.path("features").toString().contains("\"hardware_health\""))
        throw new IllegalStateException(
            "Controller lacks hardware_health; restart updated start.sh");
    for (int station : options.health().stations()) {
      grid.requireCell(station);
      if (!grid.legal(station, station))
        throw new IllegalArgumentException("Blocked charging station " + station);
    }
    health = new HardwareHealth(options.health(), ids, json, events::add);
    if (options.matching()) healthGate = new HealthGate();
    health.start();
    long deadline = System.nanoTime() + (long) (options.health().startupTimeout() * 1e9);
    do {
      latest = fleet.statuses(ids);
      validate(latest);
      health.update(latest);
      if (ids.stream().allMatch(id -> health.view(id).fresh())) break;
      if (System.nanoTime() >= deadline) break;
      TimeUnit.MILLISECONDS.sleep(100);
    } while (true);
    for (int id : ids) {
      var state = latest.get(id);
      if (!health.view(id).eligible()) {
        if (state.phase().equals("hovering"))
          throw new IllegalStateException(
              "Unhealthy aircraft already airborne before mission: D" + id);
        skip(id, "preflight_" + health.view(id).battery() + "_" + health.view(id).communication());
      } else if (!healthAllowed(id, EmergencyPolicy.Action.NORMAL, "preflight"))
        skip(id, "preflight_health_rule_rejected");
    }
    participants.removeAll(skipped);
    nextHealthPoll = System.nanoTime() + 1_000_000_000L;
  }

  private void skip(int id, String reason) {
    if (skipped.add(id)) {
      events.add(
          Map.of(
              "event",
              "takeoff_skipped",
              "drone",
              id,
              "reason",
              reason,
              "health",
              health.view(id)));
      System.out.println("SKIP D" + id + ": " + reason);
    }
  }

  private void pollHealth(Map<Integer, ControllerFleet.Status> states) {
    if (health == null || System.nanoTime() < nextHealthPoll) return;
    health.update(states);
    nextHealthPoll = System.nanoTime() + 1_000_000_000L;
  }

  private boolean healthAllowed(int id, EmergencyPolicy.Action action, String stage)
      throws Exception {
    if (healthGate == null) return true;
    var view = health.view(id);
    var result = healthGate.check(view, action);
    healthMatchMs.add(result.matchMs());
    // Routine permit matches are measured without duplicating the complete health frame every tick.
    if (!stage.equals("trajectory") || !result.allowed())
      events.add(
          Map.of(
              "event",
              "health_match",
              "drone",
              id,
              "action",
              action,
              "stage",
              stage,
              "allowed",
              result.allowed(),
              "health",
              view,
              "match_ms",
              result.matchMs()));
    return result.allowed();
  }

  private Set<Integer> occupiedCells(ControllerFleet.Status state) {
    Vec3 p = state.position();
    if (Set.of("active", "idle", "landed").contains(state.phase()))
      p = new Vec3(p.x(), p.y(), grid.center(grid.cell(p)).z());
    if (p.z() < grid.origin.z() + config.margin())
      p = new Vec3(p.x(), p.y(), grid.origin.z() + config.margin());
    return grid.footprint(p, config.margin());
  }

  private Set<Integer> fixedCells(
      Set<Integer> selected, Map<Integer, ControllerFleet.Status> states) {
    Set<Integer> reserved = new HashSet<>();
    for (int id : ids)
      if (!selected.contains(id)) {
        Active a = active.get(id);
        if (a != null) reserved.addAll(futureCells(a, states.get(id).time()));
        else reserved.addAll(occupiedCells(states.get(id)));
        if (localLandings.contains(id) || landing.containsKey(id))
          reserved.addAll(EmergencyPolicy.column(grid, states.get(id).position(), config.margin()));
      }
    return reserved;
  }

  private void waitHardwareTakeoffs(Map<Integer, String> tasks) throws Exception {
    Set<Integer> pending = new TreeSet<>(participants);
    Map<Integer, Double> settled = new HashMap<>();
    long began = System.nanoTime();
    while (!pending.isEmpty()) {
      latest = fleet.statuses(ids);
      validate(latest);
      pollHealth(latest);
      for (int id : new TreeSet<>(participants)) {
        if (skipped.contains(id)
            || emergencyFailed.contains(id)
            || landing.containsKey(id)
            || landed.contains(id)) continue;
        var state = latest.get(id);
        boolean protection =
            System.nanoTime() - began < 5_000_000_000L && state.phase().equals("taking_off");
        var response =
            EmergencyPolicy.required(
                health.view(id),
                emergencies.getOrDefault(id, EmergencyPolicy.Action.NORMAL),
                protection);
        if (response.action() != EmergencyPolicy.Action.NORMAL) {
          handleEmergency(id, response, false);
          pending.remove(id);
          continue;
        }
        if (!pending.contains(id)) continue;
        JsonNode task = tasks.containsKey(id) ? fleet.task(id, tasks.get(id)) : null;
        if (task != null && Set.of("failed", "cancelled").contains(task.path("status").asText()))
          throw new IllegalStateException("D" + id + " takeoff not confirmed: " + task);
        boolean ready =
            (task == null || task.path("status").asText().equals("completed"))
                && state.phase().equals("hovering")
                && state.velocity().norm() <= .03
                && Math.abs(state.position().z() - grid.center(grid.cell(state.position())).z())
                    <= .02;
        if (ready) {
          settled.putIfAbsent(id, state.time());
          if (state.time() - settled.get(id) >= .2) pending.remove(id);
        } else settled.remove(id);
      }
      advanceLocalLandings(false);
      if (System.nanoTime() - began > 60_000_000_000L)
        throw new TimeoutException("Hardware takeoff/hover confirmation timeout");
      if (!pending.isEmpty()) TimeUnit.MILLISECONDS.sleep(50);
    }
  }

  private Set<Integer> affectedByStop(int trigger, Map<Integer, ControllerFleet.Status> states) {
    Set<Integer> affected = new TreeSet<>(Set.of(trigger));
    boolean changed;
    do {
      changed = false;
      Set<Integer> stopCells = new HashSet<>();
      for (int id : affected) {
        var state = states.get(id);
        stopCells.addAll(occupiedCells(state));
        Vec3 v = state.velocity();
        double t = Math.max(.2, 1.5 * v.norm() / .5);
        Vec3 end = state.position().add(v.mul(t / 2 + .1));
        if (end.z() < grid.origin.z() + config.margin())
          end = new Vec3(end.x(), end.y(), grid.origin.z() + config.margin());
        stopCells.addAll(grid.footprint(end, config.margin()));
      }
      for (Active a : active.values())
        if (!affected.contains(a.id)
            && !Collections.disjoint(futureCells(a, states.get(a.id).time()), stopCells)) {
          affected.add(a.id);
          changed = true;
        }
    } while (changed);
    return affected;
  }

  private void handleEmergency(int id, EmergencyPolicy.Response response) throws Exception {
    handleEmergency(id, response, true);
  }

  private void handleEmergency(int id, EmergencyPolicy.Response response, boolean dispatch)
      throws Exception {
    if (response.action() == EmergencyPolicy.Action.NORMAL
        || skipped.contains(id)
        || landed.contains(id)
        || emergencyFailed.contains(id)) return;
    var previous = emergencies.get(id);
    if (previous == response.action()
        && !response.action().equals(EmergencyPolicy.Action.HEALTH_UNKNOWN)) return;
    var states = fleet.statuses(ids);
    validate(states);
    Set<Integer> affected = dispatch ? affectedByStop(id, states) : new TreeSet<>(Set.of(id));
    events.add(
        Map.of(
            "event",
            "hardware_emergency",
            "task_id",
            states.get(id).task(),
            "time",
            states.get(id).time(),
            "drone",
            id,
            "reason",
            response.reason(),
            "action",
            response.action(),
            "previous",
            previous == null ? "NORMAL" : previous.name(),
            "affected",
            List.copyOf(affected),
            "health",
            health.view(id)));
    emergencies.put(id, response.action());
    arrivals.remove(id);
    awaitingDescent.remove(id);
    resumeGoals.remove(id);
    relocations.remove(id);
    if (landing.containsKey(id))
      return; // Already descending; never replace it with a return route.
    try {
      stopOwned(affected);
    } catch (RuntimeException failure) {
      emergencyFailure(id, "Emergency stop not confirmed: " + failure.getMessage());
      throw failure;
    }
    states = fleet.statuses(ids);
    validate(states);
    latest = states;
    if (response.action() == EmergencyPolicy.Action.HEALTH_UNKNOWN) {
      emergencyFailure(id, "Health data unavailable; hold confirmed, landing not claimed");
    } else if (!healthAllowed(id, response.action(), "emergency")) {
      emergencyFailure(id, "Health rule rejected emergency action");
    } else if (response.action() == EmergencyPolicy.Action.LAND_LOCAL) {
      localLandings.add(id);
      emergencyWaiting.put(id, states.get(id).time());
    } else {
      int home = homes.get(id), current = grid.cell(states.get(id).position());
      Set<Integer> unavailable = new HashSet<>();
      for (int peer : ids)
        if (peer != id) {
          unavailable.addAll(occupiedCells(states.get(peer)));
          Active a = active.get(peer);
          if (a != null) unavailable.addAll(futureCells(a, states.get(peer).time()));
          if (emergencies.containsKey(peer)) unavailable.add(goals.get(peer));
        }
      int target =
          response.action() == EmergencyPolicy.Action.RETURN_HOME
              ? unavailable.contains(home) ? -1 : home
              : EmergencyPolicy.rechargeTarget(
                  grid, current, home, health.config().stations(), unavailable);
      if (target < 0) {
        emergencies.put(id, EmergencyPolicy.Action.LAND_LOCAL);
        localLandings.add(id);
        emergencyWaiting.put(id, states.get(id).time());
        events.add(
            Map.of(
                "event",
                "emergency_return_unavailable",
                "drone",
                id,
                "reason",
                "No unoccupied return destination"));
      } else {
        goals.put(id, target);
        if (!dispatch && target == grid.cell(states.get(id).position())) {
          localLandings.add(id);
          emergencyWaiting.put(id, states.get(id).time());
        }
        events.add(
            Map.of(
                "event",
                "emergency_target",
                "drone",
                id,
                "cell",
                target,
                "original_cell",
                originalGoals.get(id)));
      }
    }
    if (!dispatch) return;
    Set<Integer> moving = new TreeSet<>(affected);
    moving.removeAll(localLandings);
    moving.removeAll(emergencyFailed);
    moving.removeAll(skipped);
    moving.removeAll(landed);
    moving.removeAll(landing.keySet());
    if (!moving.isEmpty()) {
      try {
        start(timedPlan(agents(moving, states), slot -> fixedCells(moving, latest)).trajectories());
      } catch (IllegalArgumentException | IllegalStateException | NoSuchElementException failure) {
        if (!localLandings.contains(id) && !emergencyFailed.contains(id)) {
          emergencies.put(id, EmergencyPolicy.Action.LAND_LOCAL);
          localLandings.add(id);
          emergencyWaiting.put(id, states.get(id).time());
          events.add(
              Map.of(
                  "event",
                  "emergency_return_unavailable",
                  "drone",
                  id,
                  "reason",
                  failure.toString()));
          moving.remove(id);
        }
        // Peers are held and replanned after the descent column has been cleared.
        for (int peer : moving) relocations.offer(peer);
      }
    }
  }

  private void emergencyFailure(int id, String reason) {
    emergencyFailed.add(id);
    localLandings.remove(id);
    emergencyWaiting.remove(id);
    commonFaults.incrementAndGet();
    events.add(Map.of("event", "emergency_unconfirmed", "drone", id, "reason", reason));
    System.err.println("EMERGENCY UNCONFIRMED D" + id + ": " + reason);
  }

  private void advanceLocalLandings() throws Exception {
    advanceLocalLandings(true);
  }

  private void advanceLocalLandings(boolean allowClearing) throws Exception {
    if (localLandings.isEmpty()) return;
    var states = fleet.statuses(ids);
    validate(states);
    latest = states;
    for (int id : new TreeSet<>(localLandings)) {
      Set<Integer> column;
      try {
        column = EmergencyPolicy.column(grid, states.get(id).position(), config.margin());
      } catch (IllegalArgumentException | IllegalStateException ex) {
        emergencyFailure(id, ex.getMessage());
        continue;
      }
      boolean blocked = false;
      Set<Integer> blockers = new TreeSet<>();
      for (int peer : ids)
        if (peer != id) {
          if (!Collections.disjoint(column, occupiedCells(states.get(peer)))) {
            blocked = true;
            blockers.add(peer);
          }
          Active a = active.get(peer);
          if (allowClearing
              && a != null
              && !Collections.disjoint(column, futureCells(a, states.get(peer).time()))) {
            blocked = true;
            blockers.add(peer);
          }
        }
      if (blocked) {
        if (!allowClearing) continue;
        if (!clearLandingColumn(id, column, blockers, states)
            && states.get(id).time() - emergencyWaiting.getOrDefault(id, states.get(id).time())
                > Math.max(30, 3 * planner.slot + 5))
          emergencyFailure(
              id, "Emergency descent column remains occupied; aircraft held, landing not claimed");
        continue;
      }
      if (!health.view(id).fresh()) {
        emergencyFailure(id, "Health data lost while awaiting emergency descent");
        continue;
      }
      if (!healthAllowed(id, EmergencyPolicy.Action.LAND_LOCAL, "landing")) {
        emergencyFailure(id, "Emergency landing health rule rejected");
        continue;
      }
      if (options.matching()) {
        int cell = grid.cell(states.get(id).position());
        boolean allowed = true;
        for (int c = cell; grid.z(c) > 0; ) {
          int below = grid.index(grid.x(c), grid.y(c), grid.z(c) - 1);
          final int current = c;
          var occupants =
              snapshot(states).occupants().stream()
                  .map(o -> o.id() == id ? new BigraphGate.Occupant(id, current, true) : o)
                  .toList();
          var result =
              gates
                  .get(id)
                  .check(
                      new BigraphGate.Snapshot(snapshotVersion.incrementAndGet(), occupants),
                      id,
                      c,
                      below,
                      false);
          matchMs.add(result.matchMs());
          matcherChecks.incrementAndGet();
          if (!result.allowed()) {
            allowed = false;
            break;
          }
          c = below;
        }
        if (!allowed) {
          emergencyFailure(id, "Emergency descent Bigraph rule rejected");
          continue;
        }
      }
      var state = states.get(id);
      double duration = Math.max(3, 2 * state.position().z() / config.speed());
      String task =
          fleet
              .post(id, "/begin_landing", Map.of("height", 0, "duration", duration))
              .path("task_id")
              .asText();
      landing.put(
          id, new Landing(task, state.time(), System.nanoTime(), Math.max(45, duration + 15)));
      int c = grid.cell(state.position());
      goals.put(id, grid.index(grid.x(c), grid.y(c), 0));
      localLandings.remove(id);
      emergencyWaiting.remove(id);
      events.add(
          Map.of(
              "event",
              "emergency_landing_started",
              "drone",
              id,
              "task_id",
              task,
              "position",
              state.position().array(),
              "time",
              state.time()));
    }
  }

  private boolean clearLandingColumn(
      int emergency,
      Set<Integer> column,
      Set<Integer> blockers,
      Map<Integer, ControllerFleet.Status> states)
      throws Exception {
    Set<Integer> selected = new TreeSet<>();
    for (int peer : blockers) {
      if (skipped.contains(peer)
          || landed.contains(peer)
          || emergencyFailed.contains(peer)
          || landing.containsKey(peer)
          || localLandings.contains(peer)) return false;
      if (resumeGoals.containsKey(peer))
        return false; // An owned clearing trajectory is already running.
      selected.add(peer);
    }
    stopOwned(selected);
    for (int peer : selected) relocations.offer(peer);
    states = fleet.statuses(ids);
    validate(states);
    latest = states;
    Map<Integer, PolynomialTrajectory> curves = new TreeMap<>();
    Map<Integer, Integer> clearingTargets = new TreeMap<>();
    Set<Integer> reserved = fixedCells(selected, states);
    // The aircraft awaiting descent remains at its actual height during clearing.
    reserved.removeAll(column);
    reserved.addAll(occupiedCells(states.get(emergency)));
    for (int peer : selected) {
      var state = states.get(peer);
      int cell = grid.cell(state.position());
      if (Collections.disjoint(column, occupiedCells(state))) {
        relocations.offer(peer);
        continue;
      }
      PolynomialTrajectory chosen = null;
      int target = -1;
      for (var edge : grid.outgoing(cell)) {
        if (column.contains(edge.to())) continue;
        Set<Integer> corridor = grid.corridor(cell, edge.to());
        if (!Collections.disjoint(corridor, reserved)) continue;
        try {
          var path =
              state.position().distance(grid.center(cell)) > .001
                  ? List.of(cell, cell, edge.to())
                  : List.of(cell, edge.to());
          chosen =
              new PolynomialTrajectory(
                  grid,
                  path,
                  state.position(),
                  planner.slot,
                  config.speed(),
                  config.acceleration(),
                  config.margin());
          target = edge.to();
          break;
        } catch (IllegalArgumentException ignored) {
        }
      }
      if (chosen == null) return false;
      clearingTargets.put(peer, target);
      curves.put(peer, chosen);
      reserved.addAll(chosen.path);
    }
    if (!curves.isEmpty()) {
      planner.certify(curves);
      for (var entry : clearingTargets.entrySet()) {
        resumeGoals.put(entry.getKey(), goals.get(entry.getKey()));
        goals.put(entry.getKey(), entry.getValue());
      }
      start(curves);
      events.add(
          Map.of(
              "event",
              "emergency_column_clearing",
              "drone",
              emergency,
              "aircraft",
              curves.keySet()));
    }
    return true;
  }

  private void resumeRelocations() throws Exception {
    Set<Integer> pending = new TreeSet<>();
    for (Integer id; (id = relocations.poll()) != null; )
      if (!active.containsKey(id)
          && !emergencyFailed.contains(id)
          && !landed.contains(id)
          && !skipped.contains(id)) pending.add(id);
    if (pending.isEmpty()) return;
    var states = fleet.statuses(ids);
    validate(states);
    latest = states;
    for (int id : pending) {
      Integer goal = resumeGoals.remove(id);
      if (goal != null) goals.put(id, goal);
    }
    try {
      start(timedPlan(agents(pending, states), slot -> fixedCells(pending, latest)).trajectories());
    } catch (IllegalArgumentException | IllegalStateException | NoSuchElementException ex) {
      for (int id : pending) relocations.offer(id);
      if (localLandings.isEmpty() && landing.isEmpty()) throw ex;
    }
  }

  private void validateLandingColumns() {
    Set<Integer> columns = new HashSet<>();
    for (int goal : goals.values()) {
      int ground = grid.index(grid.x(goal), grid.y(goal), 0);
      if (!columns.add(ground))
        throw new IllegalArgumentException(
            "Automatic landing requires distinct ground cells for all goals");
      for (int c = goal; grid.z(c) > 0; ) {
        int below = grid.index(grid.x(c), grid.y(c), grid.z(c) - 1);
        if (!grid.legal(c, below))
          throw new IllegalArgumentException("Landing column has a blocked or missing DownRoute");
        c = below;
      }
    }
  }

  private CooperativePlanner.Plan timedPlan(
      List<CooperativePlanner.Agent> agents, CooperativePlanner.Reservations fixed) {
    long start = System.nanoTime();
    try {
      return planner.plan(agents, fixed);
    } finally {
      planningMs.add((System.nanoTime() - start) / 1e6);
    }
  }

  private void awaitMissions() throws Exception {
    while (landed.size() + skipped.size() + emergencyFailed.size() < ids.size()) {
      advanceLocalLandings();
      resumeRelocations();
      advanceLandings();
      Fault fault = faults.poll(100, TimeUnit.MILLISECONDS);
      if (fault == null) continue;
      Active owner = active.get(fault.id);
      if (owner != null && owner.task.equals(fault.task)) {
        if (health != null && fault.reason.startsWith("hardware_health:")) {
          var response =
              EmergencyPolicy.required(
                  health.view(fault.id),
                  emergencies.getOrDefault(fault.id, EmergencyPolicy.Action.NORMAL),
                  false);
          if (response.action() == EmergencyPolicy.Action.NORMAL) replan(fault);
          else handleEmergency(fault.id, response);
        } else replan(fault);
      }
    }
  }

  private void completeTrajectory(Active owner) {
    if (active.remove(owner.id, owner)) {
      if (resumeGoals.containsKey(owner.id)) {
        relocations.offer(owner.id);
        return;
      }
      events.add(Map.of("event", "completed", "drone", owner.id));
      arrivals.offer(owner.id);
    }
  }

  private boolean arrivalHealthChanged(int id) throws Exception {
    if (health == null) return false;
    var current = emergencies.getOrDefault(id, EmergencyPolicy.Action.NORMAL);
    var response = EmergencyPolicy.required(health.view(id), current, false);
    if (response.action() != current
        || response.action() == EmergencyPolicy.Action.HEALTH_UNKNOWN) {
      handleEmergency(id, response);
      return true;
    }
    if (!healthAllowed(id, current, "landing")) {
      emergencyFailure(id, "Landing health rule rejected");
      return true;
    }
    return false;
  }

  private void advanceLandings() throws Exception {
    if (arrivals.isEmpty() && awaitingDescent.isEmpty() && landing.isEmpty()) return;
    var states = fleet.statuses(ids);
    validate(states);
    Set<Integer> ready = new TreeSet<>();
    for (Integer id; (id = arrivals.poll()) != null; ) {
      var state = states.get(id);
      if (arrivalHealthChanged(id)) continue;
      int cell = grid.cell(state.position());
      if (cell != goals.get(id) || state.velocity().norm() > .05)
        throw new IllegalStateException("D" + id + " completed without settling at its target");
      if (grid.z(cell) == 0) ready.add(id);
      else awaitingDescent.add(id);
    }
    // No fleet arrival barrier: only the aircraft completing its own mission lands.
    fleet.parallel(
        ready,
        id -> {
          var s = states.get(id);
          if (!grid.box(grid.cell(s.position())).contains(s.position(), config.margin()))
            throw new IllegalStateException("Landing envelope exceeds target cell for D" + id);
          String task =
              fleet
                  .post(id, "/begin_landing", Map.of("height", 0, "duration", 3))
                  .path("task_id")
                  .asText();
          synchronized (landing) {
            landing.put(id, new Landing(task, s.time(), System.nanoTime(), 45));
          }
          events.add(
              Map.of(
                  "event",
                  "landing_started",
                  "drone",
                  id,
                  "time",
                  s.time(),
                  "other_trajectories",
                  active.size()));
          System.out.println("LAND D" + id + ": own target reached");
          return task;
        });
    if (!landing.isEmpty()) {
      var tasks = fleet.parallel(landing.keySet(), id -> fleet.task(id, landing.get(id).task()));
      for (int id : new ArrayList<>(landing.keySet())) {
        var task = tasks.get(id);
        String status = task.path("status").asText();
        if (status.equals("completed")) {
          if (states.get(id).phase().equals("landed")) {
            landing.remove(id);
            landed.add(id);
            events.add(Map.of("event", "landed", "drone", id, "time", states.get(id).time()));
            continue;
          }
        } else if (status.equals("failed") || status.equals("cancelled"))
          throw new IllegalStateException("D" + id + " landing failed: " + task);
        if (states.get(id).time() - landing.get(id).startedAt() > landing.get(id).budget()
            || System.nanoTime() - landing.get(id).wallStart() > 600_000_000_000L)
          throw new TimeoutException("D" + id + " landing timeout");
      }
    }
    for (int id : new ArrayList<>(awaitingDescent)) {
      if (arrivalHealthChanged(id)) continue;
      var s = states.get(id);
      int cell = grid.cell(s.position());
      Set<Integer> column = new HashSet<>();
      for (int z = 0; z <= grid.z(cell); z++) column.add(grid.index(grid.x(cell), grid.y(cell), z));
      Set<Integer> reserved = new HashSet<>();
      for (int other : ids)
        if (other != id) {
          Active owner = active.get(other);
          if (owner != null) reserved.addAll(futureCells(owner, states.get(other).time()));
          else reserved.add(grid.cell(states.get(other).position()));
        }
      // Higher-layer descent yields only to intersecting flights, never to the whole fleet.
      if (!Collections.disjoint(column, reserved)) continue;
      int ground = grid.index(grid.x(cell), grid.y(cell), 0);
      var plan =
          timedPlan(
              List.of(new CooperativePlanner.Agent(id, cell, ground, s.position())),
              slot -> reserved);
      expanded += plan.expanded();
      goals.put(id, ground);
      start(plan.trajectories());
      awaitingDescent.remove(id);
      events.add(Map.of("event", "descent_started", "drone", id, "time", s.time()));
    }
  }

  private void validate(Map<Integer, ControllerFleet.Status> states) {
    String clock = null;
    double lo = Double.POSITIVE_INFINITY, hi = -lo;
    for (var s : states.values()) {
      if (!s.connected()
          || !s.services()
          || s.age() > .3
          || s.age() < -.1
          || !Double.isFinite(s.time()))
        throw new IllegalStateException("Fresh ROS telemetry/services required for D" + s.id());
      if (!s.drone().equals("cf" + (config.firstCf() + s.id())))
        throw new IllegalStateException("HTTP port/drone identity mismatch for D" + s.id());
      String type = s.json().path("clock_type").asText();
      if (clock != null && !clock.equals(type))
        throw new IllegalStateException("Mixed fleet clocks");
      clock = type;
      lo = Math.min(lo, s.time());
      hi = Math.max(hi, s.time());
      grid.cell(s.position());
    }
    if (hi - lo > .1)
      throw new IllegalStateException("Fleet controller clocks differ by more than .1 s");
  }

  private List<CooperativePlanner.Agent> agents(
      Collection<Integer> selected, Map<Integer, ControllerFleet.Status> states) {
    return selected.stream()
        .sorted()
        .map(
            id -> {
              var s = states.get(id);
              return new CooperativePlanner.Agent(
                  id, grid.cell(s.position()), goals.get(id), s.position());
            })
        .toList();
  }

  private BigraphGate.Snapshot snapshot(Map<Integer, ControllerFleet.Status> states) {
    return new BigraphGate.Snapshot(
        snapshotVersion.incrementAndGet(),
        states.values().stream()
            .map(
                s ->
                    new BigraphGate.Occupant(
                        s.id(),
                        grid.cell(s.position()),
                        Set.of("hovering", "full_state", "trajectory", "braking")
                                .contains(s.phase())
                            && s.connected()
                            && s.age() <= .3))
            .toList());
  }

  private void benchmark(List<CooperativePlanner.Agent> agents, CooperativePlanner.Plan plan)
      throws Exception {
    var state =
        new BigraphGate.Snapshot(
            0,
            agents.stream().map(a -> new BigraphGate.Occupant(a.id(), a.start(), true)).toList());
    long start;
    int checked = 0;
    // Warm the same state and edge rules used by the live monitor, including model builders.
    for (int pass = 0; pass < 3; pass++)
      for (var e : plan.trajectories().entrySet()) {
        gates.get(e.getKey()).checkState(state, e.getKey(), e.getValue().path.getFirst());
        var crossings = e.getValue().crossings(grid);
        if (!crossings.isEmpty()) {
          var c = crossings.getFirst();
          gates.get(e.getKey()).check(state, e.getKey(), c.from(), c.to(), false);
        }
      }
    start = System.nanoTime();
    var matches =
        fleet.parallel(
            plan.trajectories().keySet(),
            id -> {
              var curve = plan.trajectories().get(id);
              var current = gates.get(id).checkState(state, id, curve.path.getFirst());
              if (!current.allowed())
                throw new IllegalStateException("Initial Bigraph state rejected for D" + id);
              var crossings = curve.crossings(grid);
              if (crossings.isEmpty()) return current;
              var c = crossings.getFirst();
              return gates.get(id).check(state, id, c.from(), c.to(), false);
            });
    checked = (int) matches.values().stream().filter(Objects::nonNull).count();
    double elapsed = (System.nanoTime() - start) / 1e6;
    if (elapsed > 200)
      throw new IllegalStateException(
          "Bigraph gate cycle too slow for .5 s permits: " + elapsed + " ms");
    System.out.printf(
        Locale.ROOT,
        "Gate preflight: %d aircraft state/edge checks in %.1f ms%n",
        checked,
        elapsed);
  }

  private synchronized void start(Map<Integer, PolynomialTrajectory> curves) throws Exception {
    fleet.parallel(
        curves.keySet(),
        id ->
            fleet.post(
                id,
                "/trajectory/upload",
                Map.of(
                    "trajectory_id",
                    0,
                    "upload_to_vehicle",
                    false,
                    "pieces",
                    curves.get(id).json())));
    var states = fleet.statuses(ids);
    validate(states);
    double epoch =
        states.values().stream().mapToDouble(ControllerFleet.Status::time).max().orElseThrow() + 2;
    if (experiment != null) experiment.start(epoch);
    Map<Integer, List<Scenario.Window>> pauses = new TreeMap<>();
    for (int id : curves.keySet())
      pauses.put(id, experiment == null ? List.of() : experiment.taskWindows(id, epoch));
    Map<Integer, String> dispatchedTasks = new ConcurrentHashMap<>();
    try {
      fleet.parallel(
          curves.keySet(),
          id -> {
            var curve = curves.get(id);
            Map<String, Object> request =
                new TreeMap<>(
                    Map.of(
                        "trajectory_id",
                        0,
                        "relative",
                        false,
                        "rate",
                        50,
                        "start_at",
                        epoch,
                        "require_authorization",
                        options.matching()));
            if (!pauses.get(id).isEmpty())
              request.put("pauses", pauses.get(id).stream().map(Scenario.Window::json).toList());
            var reply = fleet.post(id, "/trajectory/full_state/start", request);
            String task = reply.path("task_id").asText();
            dispatchedTasks.put(id, task);
            active.put(id, new Active(id, task, curve, epoch, grid, pauses.get(id)));
            return reply;
          });
    } catch (Exception failure) {
      // Scheduled execution metadata also identifies accepted requests when a reply is lost.
      for (int id : curves.keySet())
        try {
          var state = fleet.status(id);
          var task = fleet.task(id, state.task());
          if (task.path("action").asText().equals("full_state")
              && Math.abs(task.path("trajectory_epoch").asDouble(-1) - epoch) < 1e-9)
            active.put(
                id, new Active(id, state.task(), curves.get(id), epoch, grid, pauses.get(id)));
        } catch (Exception ignored) {
        }
      throw failure;
    }
    Map<String, Object> dispatched = new TreeMap<>();
    for (var entry : new TreeMap<>(curves).entrySet()) {
      var curve = entry.getValue();
      dispatched.put(
          "D" + entry.getKey(),
          Map.of(
              "task_id",
              dispatchedTasks.get(entry.getKey()),
              "grid_path",
              curve.path,
              "pieces",
              curve.json(),
              "duration",
              curve.duration(),
              "planned_hold_seconds",
              curve.stationarySeconds(),
              "interior_stops",
              curve.interiorStops(),
              "pause_windows",
              pauses.get(entry.getKey())));
      if (!pauses.get(entry.getKey()).isEmpty())
        System.out.println(
            "INPUT D" + entry.getKey() + ": absolute pause windows=" + pauses.get(entry.getKey()));
      System.out.printf(
          Locale.ROOT,
          "DISPATCH D%d: start_at=%.3f duration=%.3fs planned_hold=%.3fs interior_stops=%d%n",
          entry.getKey(),
          epoch,
          curve.duration(),
          curve.stationarySeconds(),
          curve.interiorStops());
    }
    events.add(
        Map.of("event", "start", "epoch", epoch, "drones", curves.keySet(), "plans", dispatched));
    System.out.println(
        "START aircraft="
            + curves.keySet()
            + " common_epoch="
            + epoch
            + " matching="
            + options.matching());
  }

  private void tick() {
    long start = System.nanoTime();
    try {
      var states = fleet.statuses(ids);
      validate(states);
      latest = states;
      pollHealth(states);
      var snapshot = options.matching() ? snapshot(states) : null;
      if (experiment != null) {
        experimentStatus = experiment.observe();
      }
      recordSamples(states);
      if (experimentStatus != null
          && experimentStatus.path("conflict").asBoolean()
          && gridConflict.compareAndSet(false, true)) {
        events.add(
            Map.of(
                "event",
                "grid_conflict_observed",
                "observation",
                experimentStatus.path("grid_conflict")));
        System.out.println(
            "GRID CONFLICT RECORDED: observation continues until mission completion");
      }
      if (store != null) store.submit(snapshot);
      var owners = new ArrayList<>(active.values());
      Map<Integer, Active> ownerById = new TreeMap<>();
      for (Active a : owners) ownerById.put(a.id, a);
      var tasks = fleet.parallel(ownerById.keySet(), id -> fleet.task(id, ownerById.get(id).task));
      for (int i : ids)
        for (int j : ids)
          if (i < j) {
            if (grid.cell(states.get(i).position()) == grid.cell(states.get(j).position()))
              coOccupancy.incrementAndGet();
            double d = states.get(i).position().distance(states.get(j).position());
            minDistance = Math.min(minDistance, d);
          }
      Map<Integer, Boolean> grants =
          fleet.parallel(
              ownerById.keySet(),
              id -> evaluate(ownerById.get(id), states, snapshot, tasks, owners));
      grants.values().removeIf(Objects::isNull);
      fleet.parallel(
          grants.keySet(),
          id -> {
            Active owner = ownerById.get(id);
            if (owner != active.get(id) || owner.failed.get()) return null;
            try {
              return fleet.post(
                  id,
                  "/trajectory/authorize",
                  Map.of("task_id", owner.task, "expires_at", states.get(id).time() + .5));
            } catch (Exception failure) {
              if (owner != active.get(id) || owner.failed.get()) return null;
              try {
                if (fleet.task(id, owner.task).path("status").asText().equals("completed")) {
                  completeTrajectory(owner);
                  return null;
                }
              } catch (Exception ignored) {
              }
              fail(owner, "permit_delivery_failed:" + failure.getMessage());
              return null;
            }
          });
      double cycle = (System.nanoTime() - start) / 1e6;
      cycleMs.add(cycle);
      if (cycle > 300) for (Active a : owners) fail(a, "monitor_cycle_exceeded_300ms");
    } catch (Exception e) {
      for (Active a : active.values()) fail(a, "monitor_failure:" + e.getMessage());
    }
  }

  private Boolean evaluate(
      Active a,
      Map<Integer, ControllerFleet.Status> states,
      BigraphGate.Snapshot snapshot,
      Map<Integer, JsonNode> tasks,
      List<Active> owners)
      throws Exception {
    if (a.failed.get() || active.get(a.id) != a) return null;
    var s = states.get(a.id);
    if (health != null) {
      var current = emergencies.getOrDefault(a.id, EmergencyPolicy.Action.NORMAL);
      var response = EmergencyPolicy.required(health.view(a.id), current, false);
      if (response.action() != current
          || response.action() == EmergencyPolicy.Action.HEALTH_UNKNOWN) {
        fail(a, "hardware_health:" + response.reason());
        return null;
      }
      if (!healthAllowed(a.id, current, "trajectory")) {
        fail(a, "hardware_health:health_rule_rejected");
        return null;
      }
    }
    JsonNode task = tasks.get(a.id);
    String status = task.path("status").asText();
    if (status.equals("completed")) {
      completeTrajectory(a);
      return null;
    }
    if (!status.equals("running") && !status.equals("queued")) {
      fail(
          a, "task_" + status + ":" + task.path("stop_reason").asText(task.path("error").asText()));
      return null;
    }
    if (task.has("trajectory_started_at") && !a.startRecorded) {
      a.startRecorded = true;
      var starts =
          actualStarts.computeIfAbsent(Double.toString(a.epoch), k -> new ConcurrentHashMap<>());
      starts.put(a.id, task.get("trajectory_started_at").asDouble());
      double skew =
          starts.values().stream().mapToDouble(Double::doubleValue).max().orElse(0)
              - starts.values().stream().mapToDouble(Double::doubleValue).min().orElse(0);
      if (skew > .1) {
        for (Active peer : owners) if (peer.epoch == a.epoch) fail(peer, "synchronized_start_skew");
      }
    }
    int actualCell = grid.cell(s.position());
    if (a.observedCell != null && actualCell != a.observedCell) {
      events.add(
          Map.of(
              "event",
              "grid_crossing",
              "drone",
              a.id,
              "from",
              a.observedCell,
              "to",
              actualCell,
              "time",
              s.time(),
              "task_id",
              a.task));
    }
    a.observedCell = actualCell;
    double elapsed = a.elapsed(s.time());
    double error = s.position().distance(a.curve.at(Math.max(0, elapsed)));
    maxTrackingError.accumulate(error);
    if (a.nextCrossing < a.crossings.size() && actualCell == a.crossings.get(a.nextCrossing).to())
      a.nextCrossing++;
    if (!options.matching()) return null;
    int expected =
        a.nextCrossing < a.crossings.size()
            ? a.crossings.get(a.nextCrossing).from()
            : a.curve.path.getLast();
    var stateMatch = gates.get(a.id).checkState(snapshot, a.id, expected);
    recordMatch(a, stateMatch);
    if (!stateMatch.allowed()) {
      fail(a, "bigraph_state_rejected:expected=" + expected + ":actual=" + actualCell);
      return null;
    }
    if (a.nextCrossing < a.crossings.size()) {
      var edge = a.crossings.get(a.nextCrossing);
      if (edge.at() - elapsed <= 3) {
        var move = gates.get(a.id).check(snapshot, a.id, edge.from(), edge.to(), false);
        recordMatch(a, move);
        if (!move.allowed()) {
          fail(a, "bigraph_edge_rejected:" + edge.from() + "->" + edge.to());
          return null;
        }
      }
    }
    return true;
  }

  private void recordMatch(Active owner, BigraphGate.Result result) {
    matcherChecks.incrementAndGet();
    matchMs.add(result.matchMs());
    matchByDrone
        .computeIfAbsent(owner.id, key -> new ConcurrentLinkedQueue<>())
        .add(result.matchMs());
    if (!result.allowed()) {
      matchDenials.incrementAndGet();
      events.add(
          Map.of(
              "event",
              "match_rejected",
              "drone",
              owner.id,
              "task_id",
              owner.task,
              "snapshot",
              result.snapshotVersion(),
              "time",
              latest.get(owner.id).time()));
    }
  }

  private void recordSamples(Map<Integer, ControllerFleet.Status> states) {
    for (var state : states.values()) {
      Active owner = active.get(state.id());
      String referenceError =
          owner == null || owner.failed.get()
              ? ""
              : Double.toString(
                  state.position().distance(owner.curve.at(owner.elapsed(state.time()))));
      String originalCell = "", originalError = "";
      if (experiment != null && experiment.armed()) {
        Vec3 original = experiment.originalReference(state.id(), state.time());
        originalCell = Integer.toString(grid.cell(original));
        originalError = Double.toString(state.position().distance(original));
      }
      JsonNode bias =
          experimentStatus == null
              ? json.createArrayNode().add(0).add(0).add(0)
              : experimentStatus
                  .path("frames")
                  .path("" + (config.firstCf() + state.id()))
                  .path("bias");
      samples.add(
          String.format(
              Locale.ROOT,
              "%.6f,D%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%s,%s,%s,%s,%s,%.9f,%.9f,%.9f",
              state.time(),
              state.id(),
              state.position().x(),
              state.position().y(),
              state.position().z(),
              state.velocity().x(),
              state.velocity().y(),
              state.velocity().z(),
              grid.cell(state.position()),
              state.phase(),
              referenceError,
              state.task(),
              originalCell,
              originalError,
              bias.path(0).asDouble(),
              bias.path(1).asDouble(),
              bias.path(2).asDouble()));
    }
  }

  private void fail(Active a, String reason) {
    if (a != null && active.get(a.id) == a && a.failed.compareAndSet(false, true)) {
      faults.offer(new Fault(a.id, a.task, reason));
      String source =
          reason.startsWith("bigraph_")
              ? "bigraph"
              : reason.startsWith("hardware_health:") ? "hardware_health" : "task_fault";
      if (source.equals("task_fault")) commonFaults.incrementAndGet();
      events.add(
          Map.of(
              "event",
              "stop_requested",
              "drone",
              a.id,
              "task_id",
              a.task,
              "source",
              source,
              "reason",
              reason));
      System.err.println("STOP REQUEST D" + a.id + " source=" + source + ": " + reason);
    }
  }

  private void replan(Fault fault) throws Exception {
    if (++replans > 8)
      throw new IllegalStateException(
          "Repeated runtime gate failures; aircraft held for inspection");
    var states = fleet.statuses(ids);
    validate(states);
    Set<Integer> affected = affectedByStop(fault.id, states);
    events.add(
        Map.of("event", "replan", "reason", fault.reason, "affected", List.copyOf(affected)));
    stopOwned(affected);
    for (int attempt = 0; attempt < 2; attempt++) {
      states = fleet.statuses(ids);
      validate(states);
      Map<String, Object> held = new TreeMap<>();
      for (int id : affected) {
        var state = states.get(id);
        held.put(
            "D" + id,
            Map.of(
                "position",
                state.position().array(),
                "velocity",
                state.velocity().array(),
                "time",
                state.time()));
      }
      events.add(Map.of("event", "held_before_replan", "aircraft", held));
      Set<Integer> reserved = new HashSet<>();
      for (int id : ids)
        if (!affected.contains(id)) {
          Active a = active.get(id);
          if (a != null) reserved.addAll(futureCells(a, states.get(id).time()));
          else reserved.add(grid.cell(states.get(id).position()));
        }
      try {
        Set<Integer> fixed = Set.copyOf(reserved);
        var plan = timedPlan(agents(affected, states), s -> fixed);
        expanded += plan.expanded();
        start(plan.trajectories());
        return;
      } catch (IllegalStateException | NoSuchElementException e) {
        if (attempt == 1) throw e;
        affected.addAll(active.keySet());
        stopOwned(affected);
      }
    }
  }

  private Set<Integer> futureCells(Active a, double now) {
    Set<Integer> cells = new HashSet<>();
    int first = Math.max(0, (int) (a.elapsed(now) / a.curve.slot));
    for (int t = first; t < a.curve.path.size(); t++) {
      int c = CooperativePlanner.at(a.curve.path, t);
      cells.addAll(grid.corridor(c, CooperativePlanner.at(a.curve.path, t + 1)));
    }
    if (cells.isEmpty()) cells.add(a.curve.path.getLast());
    if (a.failed.get() && latest.containsKey(a.id))
      cells.addAll(grid.footprint(latest.get(a.id).position(), config.margin()));
    return cells;
  }

  private void stopOwned(Set<Integer> selected) {
    if (selected.isEmpty()) return;
    try {
      Map<Integer, String> stops =
          fleet.parallel(
              selected,
              id -> {
                Active a = active.get(id);
                if (a == null) return "";
                a.failed.set(true);
                var s = fleet.status(id);
                if (s.task().equals(a.task))
                  return fleet
                      .post(id, "/trajectory/stop", Map.of("task_id", a.task))
                      .path("task_id")
                      .asText();
                JsonNode old = fleet.task(id, a.task);
                String stop = old.path("stop_task_id").asText();
                if (!stop.isEmpty() && s.task().equals(stop)) {
                  if (old.path("stop_reason").asText().startsWith("authorization_")) {
                    authorizationExpiries.incrementAndGet();
                    events.add(
                        Map.of(
                            "event",
                            "authorization_stop_observed",
                            "drone",
                            id,
                            "task_id",
                            a.task,
                            "reason",
                            old.path("stop_reason").asText()));
                  }
                  return stop;
                }
                if (old.path("status").asText().equals("completed") && s.phase().equals("hovering"))
                  return "";
                throw new IllegalStateException(
                    "D" + id + " ownership changed; refusing unrelated stop");
              });
      Map<Integer, String> pending = new TreeMap<>(stops);
      pending.values().removeIf(String::isEmpty);
      fleet.waitTasks(pending, 30);
      fleet.waitHover(pending.keySet(), null);
      for (int id : selected) active.remove(id);
    } catch (Exception e) {
      System.err.println("Stop confirmation failed (local permits will expire): " + e.getMessage());
      throw new IllegalStateException(e);
    }
  }

  private static Map<String, Object> stats(Collection<Double> values) {
    var sorted = values.stream().sorted().toList();
    if (sorted.isEmpty()) return Map.of("count", 0);
    int n = sorted.size();
    return Map.of(
        "count",
        n,
        "mean_ms",
        sorted.stream().mapToDouble(Double::doubleValue).average().orElse(0),
        "p50_ms",
        sorted.get((n - 1) / 2),
        "p95_ms",
        sorted.get(Math.min(n - 1, (int) Math.ceil(.95 * n) - 1)),
        "max_ms",
        sorted.getLast());
  }

  private void writeReport(boolean complete, double elapsed) {
    try {
      Map<String, Object> report = new LinkedHashMap<>();
      report.put(
          "run_status",
          !complete || !emergencyFailed.isEmpty()
              ? "failed"
              : !emergencies.isEmpty() || !skipped.isEmpty()
                  ? "complete_with_health_exits"
                  : gridConflict.get() ? "complete_with_grid_conflict" : "complete");
      report.put("generated_at", java.time.Instant.now().toString());
      report.put("baseline_mode", !options.matching());
      report.put("matcher_checks", matcherChecks.get());
      report.put("match_rejections", matchDenials.get());
      report.put("task_faults", commonFaults.get());
      report.put("authorization_stops", authorizationExpiries.get());
      report.put(
          "trial_valid", commonFaults.get() == 0 && authorizationExpiries.get() == 0 && complete);
      if (initialInput != null) {
        report.put("input_fingerprint", inputFingerprint);
        report.put("initial_input", initialInput);
      }
      if (experimentStatus != null) {
        report.put("experiment", experimentStatus);
        JsonNode minimum = experimentStatus.path("minimum_distance");
        report.put("minimum_distance_webots_m", minimum.isNumber() ? minimum.asDouble() : null);
      }
      report.put("conflict_criterion", "same_grid_at_same_simulation_step");
      report.put(
          "same_grid_conflict", experiment != null ? gridConflict.get() : coOccupancy.get() > 0);
      report.put(
          "grid_observation",
          experiment != null ? "webots_simulation_steps" : "http_telemetry_samples");
      report.put("co_occupancy_samples", coOccupancy.get());
      report.put("completion_time_sec", elapsed);
      report.put("first_ros_pose_ms", fleet.firstTelemetryWall);
      report.put("last_ros_pose_ms", fleet.lastTelemetryWall);
      report.put("planning_latency_ms", Map.of("overall", stats(planningMs), "by_drone", Map.of()));
      Map<String, Object> perMatch = new TreeMap<>();
      matchByDrone.forEach((id, values) -> perMatch.put("D" + id, stats(values)));
      report.put("gate_match_latency_ms", Map.of("overall", stats(matchMs), "by_drone", perMatch));
      Map<String, Object> perRest = new TreeMap<>();
      fleet.restMs.forEach((id, values) -> perRest.put("D" + id, stats(values)));
      report.put(
          "rest_latency_ms",
          Map.of(
              "overall",
              stats(fleet.restMs.values().stream().flatMap(Collection::stream).toList()),
              "by_drone",
              perRest));
      Map<String, Double> skews = new TreeMap<>();
      actualStarts.forEach(
          (epoch, starts) ->
              skews.put(
                  epoch,
                  starts.values().stream().mapToDouble(Double::doubleValue).max().orElse(0)
                      - starts.values().stream().mapToDouble(Double::doubleValue).min().orElse(0)));
      report.put("start_skew_seconds", skews);
      report.put(
          "mode",
          options.matching()
              ? "cooperative_full_state_bigraph"
              : "cooperative_full_state_baseline");
      report.put("completed", complete);
      report.put("mission_aircraft", List.copyOf(participants));
      report.put("skipped_aircraft", new TreeSet<>(skipped));
      report.put("emergency_failed_aircraft", new TreeSet<>(emergencyFailed));
      report.put("emergency_actions", new TreeMap<>(emergencies));
      Map<Integer, String> outcomes = new TreeMap<>();
      for (int id : ids)
        outcomes.put(
            id,
            skipped.contains(id)
                ? "skipped_preflight"
                : emergencyFailed.contains(id)
                    ? "emergency_unconfirmed"
                    : landed.contains(id)
                        ? emergencies.containsKey(id) ? "emergency_landed" : "mission_landed"
                        : "not_completed");
      report.put("aircraft_outcomes", outcomes);
      report.put(
          "mission_success",
          complete && skipped.isEmpty() && emergencies.isEmpty() && emergencyFailed.isEmpty());
      report.put("original_goals", originalGoals);
      report.put("final_goals", new TreeMap<>(goals));
      report.put("health_match_latency_ms", stats(healthMatchMs));
      if (health != null) report.put("hardware_health", health.snapshot());
      report.put("duration_seconds", elapsed);
      report.put("drone_count", config.count());
      report.put("replans", replans);
      report.put("cbs_expanded", expanded);
      report.put("minimum_distance_m", Double.isFinite(minDistance) ? minDistance : null);
      report.put("max_tracking_error_m", maxTrackingError.get());

      report.put("gate_cycle_ms", stats(cycleMs));
      report.put("events", List.copyOf(events));
      if (store != null) report.put("cdo_error", store.lastError);
      Path output = Path.of(config.report()).toAbsolutePath();
      Files.createDirectories(output.getParent());
      Path csv =
          output.resolveSibling(
              output.getFileName().toString().replaceAll("\\.json$", "") + ".csv");
      List<String> rows = new ArrayList<>();
      rows.add(
          "sim_time,drone,x,y,z,vx,vy,vz,cell,phase,reference_error,task_id,original_plan_cell,original_reference_error,bias_x,bias_y,bias_z");
      rows.addAll(samples);
      Files.write(csv, rows);
      report.put("samples_csv", csv.getFileName().toString());
      json.writerWithDefaultPrettyPrinter().writeValue(output.toFile(), report);
    } catch (Exception e) {
      System.err.println("Cannot write mission report: " + e.getMessage());
    }
  }

  public void close() {
    if (!closed.compareAndSet(false, true)) return;
    monitor.shutdownNow();
    try {
      stopOwned(new TreeSet<>(active.keySet()));
    } catch (Exception ignored) {
    }
    if (health != null) health.close();
    if (store != null) store.close();
    if (ros != null) ros.close();
    fleet.close();
    try {
      Runtime.getRuntime().removeShutdownHook(shutdownHook);
    } catch (IllegalStateException ignored) {
    }
  }
}
