package org.example.mission;

import java.util.*;
import org.springframework.core.env.Environment;

public record MissionConfig(
    int rows,
    int cols,
    int layers,
    Vec3 origin,
    Vec3 step,
    int count,
    int firstCf,
    int port,
    String controlUrl,
    String gridUrl,
    String cdoUrl,
    boolean persist,
    String rosHost,
    boolean rosEnabled,
    double speed,
    double acceleration,
    double margin,
    int horizon,
    int nodeLimit,
    Set<Integer> obstacles,
    String targets,
    String report) {
  public MissionConfig {
    if (rows < 1
        || cols < 1
        || layers < 1
        || (long) rows * cols * layers > 4096
        || count < 1
        || firstCf < 1
        || firstCf + count > 256
        || port < 1
        || port + count > 65536)
      throw new IllegalArgumentException("Invalid fleet or grid dimensions");
    if (!Double.isFinite(speed + acceleration + margin)
        || speed <= 0
        || acceleration <= 0
        || margin < .06
        || margin > .3
        || horizon < 1
        || nodeLimit < 1)
      throw new IllegalArgumentException("Invalid mission safety or search limits");
  }

  public static MissionConfig from(Environment e) {
    Set<Integer> blocked = new HashSet<>();
    for (String s : e.getProperty("obstacle.grids", "18,67").split(","))
      if (!s.isBlank()) blocked.add(Integer.parseInt(s.trim()));
    return new MissionConfig(
        i(e, "bigrid.service.rows", 7),
        i(e, "bigrid.service.cols", 7),
        i(e, "bigrid.service.layers", 3),
        new Vec3(
            d(e, "bigrid.service.origin.x", -1.5),
            d(e, "bigrid.service.origin.y", -1.5),
            d(e, "bigrid.service.origin.z", 0)),
        new Vec3(
            d(e, "bigrid.service.step.x", .5),
            d(e, "bigrid.service.step.y", .5),
            d(e, "bigrid.service.layer.height", .6)),
        i(e, "drone.count", 12),
        i(e, "drone.first-cf", 231),
        i(e, "drone.control.start-port", 5000),
        e.getProperty("drone.control.base-url", "http://127.0.0.1"),
        e.getProperty("bigrid.service.base-url", "http://localhost:8080"),
        e.getProperty("cdo.connection", "cdo://localhost:2036/repo1"),
        e.getProperty("cdo.enabled", Boolean.class, true),
        e.getProperty("ros.bridge.host", "localhost"),
        e.getProperty("ros.update.enabled", Boolean.class, true),
        d(e, "mission.max-speed", .15),
        d(e, "mission.max-acceleration", .30),
        d(e, "mission.safety-margin", .10),
        i(e, "mission.horizon", 96),
        i(e, "mission.cbs-node-limit", 3000),
        Set.copyOf(blocked),
        e.getProperty(
            "drone.targets",
            "-1,-1.5;1,-1.5;1.5,-1;1.5,1;1,1.5;-1,1.5;-1.5,1;-1.5,-1;-1,-1;1,-1;1,1;-1,1"),
        e.getProperty("mission.report", "mission-report-cooperative.json"));
  }

  private static int i(Environment e, String key, int value) {
    return e.getProperty(key, Integer.class, value);
  }

  private static double d(Environment e, String key, double value) {
    return e.getProperty(key, Double.class, value);
  }

  public Map<Integer, Integer> goals(GridGraph grid) {
    String[] list = targets.split(";");
    if (list.length < count)
      throw new IllegalArgumentException("drone.targets needs one entry per aircraft");
    Map<Integer, Integer> result = new TreeMap<>();
    for (int id = 0; id < count; id++) {
      String[] p = list[id].split(",");
      if (p.length < 2 || p.length > 3)
        throw new IllegalArgumentException("Target format: x,y[,z]");
      Vec3 v =
          new Vec3(
              Double.parseDouble(p[0]),
              Double.parseDouble(p[1]),
              p.length == 3 ? Double.parseDouble(p[2]) : origin.z() + step.z() / 2);
      int cell = grid.cell(v);
      if (grid.center(cell).distance(v) > .001)
        throw new IllegalArgumentException("Targets must be grid centers: " + v);
      result.put(id, cell);
    }
    return Map.copyOf(result);
  }
}
