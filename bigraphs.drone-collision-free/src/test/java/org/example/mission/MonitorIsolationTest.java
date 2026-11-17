package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Map;
import java.util.concurrent.*;
import org.junit.jupiter.api.Test;
import org.springframework.mock.env.MockEnvironment;

class MonitorIsolationTest {
  @Test
  void webotsHealthPollingDoesNotWaitForConcurrentTrajectoryDispatch() throws Exception {
    try (var runner = new MissionRunner(MissionConfig.from(new MockEnvironment()));
        var worker = Executors.newVirtualThreadPerTaskExecutor()) {
      var poll = MissionRunner.class.getDeclaredMethod("pollHealth", Map.class);
      poll.setAccessible(true);
      // start() holds this monitor while HTTP upload/start requests are in flight.
      synchronized (runner) {
        var complete = worker.submit(() -> poll.invoke(runner, Map.of()));
        assertNull(complete.get(2, TimeUnit.SECONDS));
      }
    }
  }
}
