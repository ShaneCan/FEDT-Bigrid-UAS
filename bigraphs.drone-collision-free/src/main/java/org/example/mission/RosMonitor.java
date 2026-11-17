package org.example.mission;

import com.fasterxml.jackson.databind.*;
import edu.wpi.rail.jrosbridge.*;
import java.util.concurrent.*;

/** Optional independent ROSBridge observation; command ownership stays with HTTP. */
public final class RosMonitor implements AutoCloseable {
  public record Observation(Vec3 position, long receivedNanos) {}

  public final ConcurrentMap<Integer, Observation> observations = new ConcurrentHashMap<>();
  private final Ros ros;

  public RosMonitor(String host, int firstCf, ObjectMapper mapper) {
    ros = new Ros(host, 9090);
    if (!ros.connect())
      throw new IllegalStateException("Cannot connect to ROSBridge at " + host + ":9090");
    new Topic(ros, "/cf_positions_path", "nav_msgs/Path")
        .subscribe(
            message -> {
              try {
                var m = mapper.readTree(message.toJsonObject().toString());
                for (var pose : m.path("poses")) {
                  String frame = pose.path("header").path("frame_id").asText();
                  if (!frame.startsWith("cf")) continue;
                  var p = pose.path("pose").path("position");
                  observations.put(
                      Integer.parseInt(frame.substring(2)) - firstCf,
                      new Observation(
                          new Vec3(
                              p.path("x").asDouble(),
                              p.path("y").asDouble(),
                              p.path("z").asDouble()),
                          System.nanoTime()));
                }
              } catch (Exception e) {
                System.err.println("Invalid ROSBridge position frame: " + e.getMessage());
              }
            });
  }

  public void close() {
    ros.disconnect();
  }
}
