package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.*;
import java.net.URI;
import java.net.http.*;
import java.util.*;
import java.util.concurrent.*;
import org.junit.jupiter.api.Test;
import org.springframework.mock.env.MockEnvironment;

class HardwareHealthTest {
  private final ObjectMapper json = new ObjectMapper();

  private HardwareHealth health() {
    return new HardwareHealth(
        HardwareHealth.Config.from(new MockEnvironment().withProperty("fault.injection.port", "0")),
        Set.of(0, 1),
        json,
        e -> {});
  }

  private void update(HardwareHealth h, int id, double voltage, int radio, double age) {
    JsonNode node =
        json.valueToTree(
            Map.of(
                "health",
                Map.of(
                    "battery_voltage",
                    voltage,
                    "rssi",
                    radio,
                    "pm_state",
                    0,
                    "age",
                    age,
                    "received_at",
                    123.0)));
    h.update(
        Map.of(
            id,
            new ControllerFleet.Status(
                id,
                "cf" + (231 + id),
                "hovering",
                "task",
                true,
                true,
                123,
                0,
                new Vec3(0, 0, .3),
                Vec3.ZERO,
                node)));
  }

  @Test
  void unknownCannotBecomeHealthyThroughInjectionAndRawAgeSurvivesClear() {
    try (var h = health()) {
      h.inject(0, true, "Normal");
      h.inject(0, false, "Normal");
      assertFalse(h.view(0).eligible());
      update(h, 0, 3.0, 70, 0);
      assertTrue(h.view(0).eligible());
      h.inject(0, true, "clear");
      h.inject(0, false, "off");
      assertEquals("Empty", h.view(0).battery());
      assertEquals("Bad", h.view(0).communication());
      update(h, 0, 3.8, 50, 4);
      h.inject(0, true, "Normal");
      assertFalse(h.view(0).fresh());
      assertTrue(h.view(0).age() >= 4);
    }
  }

  @Test
  void thresholdsAndDynamicNamedOverridesAgree() {
    try (var h = health()) {
      update(h, 0, 3.2, 64, 0);
      assertTrue(h.view(0).eligible());
      update(h, 0, 3.1, 65, 0);
      assertEquals("Low", h.view(0).battery());
      assertEquals("Bad", h.view(0).communication());
      update(h, 0, 3.099, 50, 0);
      assertEquals("Empty", h.view(0).battery());
      for (String state : List.of("Low", "Empty", "Normal")) {
        h.inject(0, true, state);
        assertEquals(state, h.view(0).battery());
      }
      assertThrows(IllegalArgumentException.class, () -> h.inject(0, true, "NaN"));
      assertThrows(IllegalArgumentException.class, () -> h.inject(0, false, "256"));
      assertThrows(NoSuchElementException.class, () -> h.inject(5, true, "Low"));
    }
    var cfg =
        HardwareHealth.Config.from(
            new MockEnvironment()
                .withProperty("hardware.health.battery-low", "4")
                .withProperty("hardware.health.battery-empty", "3.8")
                .withProperty("hardware.health.rssi-bad", "90"));
    try (var h = new HardwareHealth(cfg, Set.of(0), json, e -> {})) {
      update(h, 0, 4.1, 30, 0);
      h.inject(0, true, "Low");
      h.inject(0, false, "Bad");
      assertEquals("Low", h.view(0).battery());
      assertEquals("Bad", h.view(0).communication());
    }
  }

  @Test
  void localHttpInjectionAndCleanup() throws Exception {
    var h = health();
    h.start();
    int port = h.port();
    try (var client = HttpClient.newHttpClient()) {
      for (var entry :
          Map.of(
                  "/inject/battery?drone=D0&level=Low",
                  200,
                  "/inject/comm?drone=D9&state=Bad",
                  404,
                  "/inject/battery?drone=D0&value=NaN",
                  400,
                  "/inject/status",
                  200)
              .entrySet()) {
        var r =
            client.send(
                HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + entry.getKey()))
                    .GET()
                    .build(),
                HttpResponse.BodyHandlers.ofString());
        assertEquals(entry.getValue().intValue(), r.statusCode());
      }
    } finally {
      h.close();
    }
    try (var socket =
        new java.net.ServerSocket(port, 0, java.net.InetAddress.getLoopbackAddress())) {
      assertEquals(port, socket.getLocalPort());
    }
  }

  @Test
  void emergencyPriorityProtectionRecoveryAndChargingSelection() {
    try (var h = health()) {
      update(h, 0, 3.15, 50, 0);
      assertEquals(
          EmergencyPolicy.Action.RETURN_RECHARGE,
          EmergencyPolicy.required(h.view(0), EmergencyPolicy.Action.NORMAL, false).action());
      assertEquals(
          EmergencyPolicy.Action.NORMAL,
          EmergencyPolicy.required(h.view(0), EmergencyPolicy.Action.NORMAL, true).action());
      update(h, 0, 3.15, 70, 0);
      assertEquals(
          EmergencyPolicy.Action.LAND_LOCAL,
          EmergencyPolicy.required(h.view(0), EmergencyPolicy.Action.RETURN_HOME, true).action());
      update(h, 0, 3.8, 50, 0);
      assertEquals(
          EmergencyPolicy.Action.RETURN_HOME,
          EmergencyPolicy.required(h.view(0), EmergencyPolicy.Action.RETURN_HOME, false).action());
      update(h, 0, 3, 50, 0);
      assertEquals(
          EmergencyPolicy.Action.LAND_LOCAL,
          EmergencyPolicy.required(h.view(0), EmergencyPolicy.Action.RETURN_RECHARGE, true)
              .action());
      update(h, 0, 3, 50, 4);
      assertEquals(
          EmergencyPolicy.Action.HEALTH_UNKNOWN,
          EmergencyPolicy.required(h.view(0), EmergencyPolicy.Action.NORMAL, false).action());
    }
    var grid = GridGraph.rectangular(7, 7, 3, Set.of());
    assertEquals(1, EmergencyPolicy.rechargeTarget(grid, 2, 0, Set.of(1, 3), Set.of(3)));
    assertEquals(0, EmergencyPolicy.rechargeTarget(grid, 2, 0, Set.of(1, 3), Set.of(1, 3)));
  }

  @Test
  void occupiedHomeDoesNotHideAnAvailableChargingDestination() {
    var grid = GridGraph.rectangular(7, 7, 3, Set.of());
    assertEquals(6, EmergencyPolicy.rechargeTarget(grid, 2, 0, Set.of(6), Set.of(0)));
    assertEquals(-1, EmergencyPolicy.rechargeTarget(grid, 2, 0, Set.of(6), Set.of(0, 6)));
  }

  @Test
  void realHealthMatcherAllowsEmergencyButRejectsNormalLowFlight() throws Exception {
    try (var h = health()) {
      update(h, 0, 3.15, 50, 0);
      var gate = new HealthGate();
      assertFalse(gate.check(h.view(0), EmergencyPolicy.Action.NORMAL).allowed());
      assertTrue(gate.check(h.view(0), EmergencyPolicy.Action.RETURN_RECHARGE).allowed());
      update(h, 0, 3, 70, 0);
      assertTrue(gate.check(h.view(0), EmergencyPolicy.Action.LAND_LOCAL).allowed());
      assertFalse(gate.check(h.view(0), EmergencyPolicy.Action.RETURN_HOME).allowed());
    }
  }

  @Test
  void occupiedInjectionPortFailsWithoutClosingTheExistingListener() throws Exception {
    try (var owner = new java.net.ServerSocket(0, 0, java.net.InetAddress.getLoopbackAddress())) {
      var cfg =
          HardwareHealth.Config.from(
              new MockEnvironment()
                  .withProperty("fault.injection.port", "" + owner.getLocalPort()));
      try (var h = new HardwareHealth(cfg, Set.of(0), json, e -> {})) {
        assertThrows(java.net.BindException.class, h::start);
      }
      assertFalse(owner.isClosed());
    }
  }
}
