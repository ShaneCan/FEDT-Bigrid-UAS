package org.example;

import com.sun.net.httpserver.HttpServer;
import org.junit.jupiter.api.Test;

import java.net.InetSocketAddress;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.test.util.ReflectionTestUtils;
import java.util.ArrayList;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

/** No ROS, CDO or aircraft required. Exercises the integrated 3D HTTP control path. */
class Application3DControlTest {
    @Test
    void controllerCompletionGatesNavigation() throws Exception {
        AtomicReference<String> state = new AtomicReference<>("{\"phase\":\"taking_off\",\"connected\":true}");
        AtomicReference<String> task = new AtomicReference<>("{\"status\":\"running\"}");
        AtomicReference<Integer> statusCode = new AtomicReference<>(200);
        HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        server.createContext("/", exchange -> {
            String body = exchange.getRequestURI().getPath().startsWith("/tasks/") ? task.get() : state.get();
            byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
            exchange.sendResponseHeaders(statusCode.get(), bytes.length);
            try (var out = exchange.getResponseBody()) { out.write(bytes); }
        });
        server.start();
        try {
            int port = server.getAddress().getPort();
            Application3D sync = new Application3D();
            ReflectionTestUtils.setField(sync, "droneControlBaseUrl", "http://127.0.0.1");
            sync.acceptControllerResponse(port, "{\"task_id\":\"takeoff-1\"}");
            check(!sync.controllerSettled(port), "Takeoff still running");
            state.set("{\"phase\":\"hovering\",\"connected\":true}");
            check(!sync.controllerSettled(port), "Hovering alone cannot complete a running task");
            task.set("{\"status\":\"completed\"}");
            check(sync.controllerSettled(port), "Settled takeoff is ready");
            sync.acceptControllerResponse(port, "{\"task_id\":\"nav-1\"}");
            state.set("{\"phase\":\"navigating\",\"connected\":true}");
            task.set("{\"status\":\"running\"}");
            check(!sync.controllerSettled(port), "Entering a grid cannot complete navigation");
            task.set("{\"status\":\"failed\",\"error\":\"Goal timeout\"}");
            check(!sync.controllerSettled(port), "Failed task must not trigger replanning");
            check(sync.controllerWaitReason(port).contains("Goal timeout"), "Expose task failure reason");
            task.set("{\"status\":\"cancelled\"}");
            check(!sync.controllerSettled(port), "Externally cancelled task requires explicit recovery");
            sync.acceptControllerResponse(port, "{\"message\":\"holding\"}");
            state.set("{\"phase\":\"hovering\",\"connected\":false}");
            check(!sync.controllerSettled(port), "Stale telemetry blocks movement");
            state.set("{\"phase\":\"fault\",\"error\":\"ROS unavailable\"}");
            check(!sync.controllerSettled(port), "Fault blocks movement");
            check(sync.controllerWaitReason(port).contains("ROS unavailable"), "Expose phase error");
            state.set("{\"state\":\"hovering\"}");
            check(sync.controllerSettled(port), "Legacy status without task API remains supported");
            statusCode.set(503);
            boolean failed = false;
            try { sync.controllerSettled(port); } catch (java.io.IOException e) { failed = e.getMessage().contains("503"); }
            check(failed, "Unavailable HTTP status is never ready");
        } finally { server.stop(0); }
    }

    @Test
    void initialWorldHeightIsNotTakeoff() {
        check(!Application3D.hasLiftedOff(false, 0.135, 0.135), "No takeoff command");
        check(!Application3D.hasLiftedOff(true, 0.135, 0.135), "No actual rise");
        check(Application3D.hasLiftedOff(true, 0.135, 0.30), "Actual rise after command");
        check(Application3D.hasLiftedOff(true, 0.0, 0.15), "Ground-level real drone");
    }

    private static void check(boolean value, String message) {
        if (!value) throw new AssertionError(message);
    }

    @Test
    void takeoffSendsConfiguredHeightWithoutAnAscentCommand() throws Exception {
        List<String> commands = new ArrayList<>();
        List<String> payloads = new ArrayList<>();
        AtomicReference<String> taskState = new AtomicReference<>("running");
        HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        server.createContext("/", exchange -> {
            String path = exchange.getRequestURI().getPath();
            String response;
            if ("POST".equals(exchange.getRequestMethod())) {
                commands.add(path);
                payloads.add(new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8));
                response = "{\"task_id\":\"motion-1\"}";
            } else if (path.startsWith("/tasks/")) {
                response = "{\"status\":\"" + taskState.get() + "\"}";
            } else {
                response = "{\"phase\":\"hovering\",\"connected\":true}";
            }
            byte[] bytes = response.getBytes(StandardCharsets.UTF_8);
            exchange.sendResponseHeaders(200, bytes.length);
            try (var out = exchange.getResponseBody()) { out.write(bytes); }
        });
        server.start();
        try {
            Application3D app = new Application3D();
            int port = server.getAddress().getPort();
            ReflectionTestUtils.setField(app, "droneControlBaseUrl", "http://127.0.0.1");
            ReflectionTestUtils.setField(app, "droneControlStartPort", port);
            assertEquals(Boolean.TRUE, ReflectionTestUtils.invokeMethod(app, "beginTakeoff", "D0", port));
            assertEquals(0.9, new ObjectMapper().readTree(payloads.get(0)).path("height").asDouble());
            assertEquals(Boolean.FALSE, ReflectionTestUtils.invokeMethod(app, "navigateTo", "D0", port, 1.0, 2.0, 0.9));
            assertEquals(List.of("/begin_takeoff"), commands);
            taskState.set("completed");
            assertEquals(Boolean.TRUE, ReflectionTestUtils.invokeMethod(app, "navigateTo", "D0", port, 1.0, 2.0, 0.9));
            assertEquals(List.of("/begin_takeoff", "/navigate/1.0/2.0/0.9"), commands);
            ReflectionTestUtils.setField(app, "takeoffHeight", 1.2);
            assertEquals(Boolean.TRUE, ReflectionTestUtils.invokeMethod(app, "beginTakeoff", "D0", port));
            assertEquals(1.2, new ObjectMapper().readTree(payloads.get(2)).path("height").asDouble());
            int accepted = commands.size();
            for (double invalid : new double[]{0, -0.1, Double.NaN, Double.POSITIVE_INFINITY}) {
                ReflectionTestUtils.setField(app, "takeoffHeight", invalid);
                assertThrows(IllegalArgumentException.class,
                        () -> ReflectionTestUtils.invokeMethod(app, "beginTakeoff", "D0", port));
            }
            assertEquals(accepted, commands.size(), "Invalid heights must not reach the controller");
        } finally { server.stop(0); }
    }
}
