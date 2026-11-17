package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.sun.net.httpserver.HttpServer;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.springframework.mock.env.MockEnvironment;

class ControllerFleetTest {
  @Test
  void fleetWorkStartsConcurrently() throws Exception {
    var config = MissionConfig.from(new MockEnvironment());
    var both = new CountDownLatch(2);
    try (var fleet = new ControllerFleet(config, new ObjectMapper())) {
      var values =
          fleet.parallel(
              List.of(10, 2),
              id -> {
                both.countDown();
                assertTrue(both.await(1, TimeUnit.SECONDS));
                return id;
              });
      assertEquals(List.of(2, 10), new ArrayList<>(values.keySet()));
    }
  }

  @Test
  void controllerTaskCompletionDoesNotReplaceStablePoseReadiness() throws Exception {
    var calls = new AtomicInteger();
    var server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
    server.createContext(
        "/status",
        exchange -> {
          int n = calls.incrementAndGet();
          String z = n < 4 ? "0.235" : "0.3";
          String value =
              "{\"drone_id\":\"cf231\",\"phase\":\"hovering\",\"connected\":true,\"services_ready\":true,\"sim_time\":"
                  + (n * .1)
                  + ",\"telemetry_age\":0,\"telemetry\":{\"position\":[0,0,"
                  + z
                  + "],\"velocity\":[0,0,0]}}";
          byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
          exchange.sendResponseHeaders(200, bytes.length);
          try (var out = exchange.getResponseBody()) {
            out.write(bytes);
          }
        });
    server.start();
    try (var fleet =
        new ControllerFleet(
            MissionConfig.from(
                new MockEnvironment()
                    .withProperty(
                        "drone.control.start-port",
                        Integer.toString(server.getAddress().getPort()))),
            new ObjectMapper())) {
      fleet.waitHover(List.of(0), Map.of(0, new Vec3(0, 0, .3)));
      assertTrue(calls.get() >= 6);
    } finally {
      server.stop(0);
    }
  }
}
