"""Upstream ROS contracts without ROS, radios or aircraft."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace, ModuleType
import unittest
from unittest.mock import MagicMock, Mock, patch

CF = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CF / "src"))
from drone_controller import ControlError, RosTransport


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


launcher = load("hardware_launcher", CF / "bin/launch.py")
services = load("hardware_services", CF.parents[1] /
                "simulation/ds-crazyflies-ext/simulation.ds-crazyflies/tools/services.py")


class UpstreamTransportTest(unittest.TestCase):
    def transport(self, require_report=False):
        transport = RosTransport.__new__(RosTransport)
        transport.backend = "hardware"
        transport.drone_id = "cf231"
        transport.require_report = require_report
        transport.node = Mock()
        transport.node.get_service_names_and_types.return_value = [
            (f"/cf231/{name}", []) for name in (
                "takeoff", "land", "go_to", "notify_setpoints_stop",
                "upload_trajectory", "start_trajectory")]
        transport.node.count_publishers.return_value = 0
        transport.node.count_subscribers.return_value = 1
        transport._sensors = {}
        transport._reports = {}
        transport._report_count = 0
        transport._report_ready = MagicMock()
        transport._report_ready.wait_for.return_value = False
        transport._clients = {}
        transport.on_telemetry = Mock()
        return transport

    def test_upstream_readiness_does_not_require_custom_reports(self):
        transport = self.transport()
        self.assertTrue(transport.services_ready())
        transport.node.count_publishers.assert_not_called()

    def test_requested_reports_are_still_required(self):
        transport = self.transport(require_report=True)
        self.assertFalse(transport.services_ready())
        transport.node.count_publishers.return_value = 1
        self.assertTrue(transport.services_ready())

    def test_missing_stream_subscriber_or_service_blocks_readiness(self):
        transport = self.transport()
        transport.node.count_subscribers.side_effect = lambda name: int(not name.endswith("cmd_full_state"))
        self.assertFalse(transport.services_ready())
        transport.node.count_subscribers.side_effect = None
        transport.node.get_service_names_and_types.return_value = []
        self.assertFalse(transport.services_ready())

    def test_world_velocity_is_not_rotated_by_aircraft_yaw(self):
        transport = self.transport()
        vec = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
        msg = SimpleNamespace(
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=vec(1, 2, .3),
                orientation=SimpleNamespace(x=0, y=0, z=math.sqrt(.5), w=math.sqrt(.5)))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=vec(.15, 0, 0))),
            header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=0)))
        transport._odom(msg)
        self.assertEqual(transport.on_telemetry.call_args.kwargs["velocity"], [.15, 0, 0])
        self.assertEqual(transport.on_telemetry.call_args.kwargs["source_time"], 12)

    def command(self, transport, name, response=None, exception=None):
        future = Mock()
        future.add_done_callback.side_effect = lambda callback: callback(future)
        future.result.side_effect = exception
        future.result.return_value = response if response is not None else SimpleNamespace()
        client = Mock()
        client.wait_for_service.return_value = True
        client.srv_type.Request = SimpleNamespace
        client.call_async.return_value = future
        transport._clients[name] = client
        utilities = ModuleType("rosidl_runtime_py.utilities")
        utilities.get_service = Mock()
        fields = ModuleType("rosidl_runtime_py.set_message")
        fields.set_message_fields = Mock()
        with patch.dict(sys.modules, {
            "rosidl_runtime_py": ModuleType("rosidl_runtime_py"),
            "rosidl_runtime_py.utilities": utilities,
            "rosidl_runtime_py.set_message": fields,
        }):
            transport._command(name)

    def test_empty_upstream_takeoff_and_upload_responses_do_not_wait_for_reports(self):
        for name in ("takeoff", "upload_trajectory"):
            with self.subTest(name=name):
                transport = self.transport()
                self.command(transport, name)
                transport._report_ready.wait_for.assert_not_called()

    def test_optional_report_timeout_remains_an_error(self):
        with self.assertRaisesRegex(TimeoutError, "Missing execution acknowledgement"):
            self.command(self.transport(require_report=True), "takeoff")

    def test_ros_rejection_and_rpc_errors_are_not_swallowed(self):
        with self.assertRaisesRegex(ControlError, "rejected"):
            self.command(self.transport(), "takeoff", SimpleNamespace(success=False, message="rejected"))
        with self.assertRaisesRegex(RuntimeError, "RPC error"):
            self.command(self.transport(), "land", exception=RuntimeError("RPC error"))


class HardwareLaunchTest(unittest.TestCase):
    def test_hardware_requires_external_workspace_and_configuration(self):
        with patch.dict(launcher.os.environ, {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "CF_ROS_SETUP.*CF_CRAZYFLIES_CONFIG"):
                launcher.local_ros(SimpleNamespace(), [])

    def test_hardware_uses_external_config_and_only_sources_its_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            setup = Path(directory) / "setup.bash"
            config = Path(directory) / "crazyflies.yaml"
            setup.write_text("", encoding="utf-8")
            config.write_text("robots: {}", encoding="utf-8")
            yaml = ModuleType("yaml")
            yaml.safe_load = lambda text: {"robots": {"cf231": {"enabled": True}, "cf232": {"enabled": False}}}
            processes = Mock()
            with patch.dict(launcher.os.environ, {
                "CF_ROS_SETUP": str(setup), "CF_CRAZYFLIES_CONFIG": str(config)}, clear=True), \
                    patch.dict(sys.modules, {"yaml": yaml, "services": services}), \
                    patch.object(launcher.subprocess, "run", return_value=SimpleNamespace(stdout=b"ROS_DISTRO=humble\0")) as run, \
                    patch.object(launcher, "Processes", return_value=processes), \
                    patch.object(launcher, "check_ports"), \
                    patch.object(services, "start_services") as start, \
                    patch.object(services, "await_services"):
                launcher.local_ros(SimpleNamespace(), [])
            self.assertEqual(run.call_args.args[0][2], 'source "$1" && env -0')
            self.assertEqual(start.call_args.args[1]["required_drone_ids"], [231])
            self.assertFalse(start.call_args.kwargs["simulated"])
            processes.close.assert_called_once()

    def test_only_simulator_requests_custom_command_reports(self):
        with tempfile.TemporaryDirectory() as directory:
            cf = Path(directory) / "cf"
            (cf / "webview").mkdir(parents=True)
            (cf / "src").mkdir()
            (cf / "webview/webview.html").write_text("<html></html>")
            config = dict(drone_ids=[231], bind_host="127.0.0.1", http_port=5000,
                          web_port=8080, ws_port=8765, wall_port=9000)
            for simulated in (False, True):
                with self.subTest(simulated=simulated):
                    processes = SimpleNamespace(directory=Path(directory) / str(simulated), spawn=Mock())
                    with patch.object(services, "CF", cf):
                        services.start_services(processes, config, simulated=simulated)
                    command = processes.spawn.call_args_list[0].args[1]
                    self.assertEqual("--require-command-report" in command, simulated)
                    self.assertEqual(processes.spawn.call_count, 2)


if __name__ == "__main__":
    unittest.main()
