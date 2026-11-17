#!/usr/bin/env python3
"""One container: Webots, native ROS, compatibility bridge and HTTP services."""
import fcntl
import json
import os
from pathlib import Path
import socket
import shutil
import subprocess
import sys
import time
import urllib.request

from config import HERE, RUNTIME, load, ports
from services import start_services, await_services

sys.path.insert(0, "/workspace/controller/cf.PyControl/bin")
from launch import Processes, check_ports, select_display


def main():
    config = load("/ds/crazywebotsworld/worlds/crazyflie.wbt")
    headless = os.environ.get("CF_HEADLESS", "0") == "1"
    if not headless:
        os.environ["DISPLAY"] = select_display(os.environ.get("DISPLAY", ""), Path("/host-x11"))
    RUNTIME.mkdir(parents=True, exist_ok=True)
    lock = (RUNTIME / "supervisor.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("This simulator already has a supervisor") from None
    ready = RUNTIME / "ready.json"
    ready.unlink(missing_ok=True)
    check_ports(ports(config) + [config["webots_port"]])
    # The lock ensures no live supervisor owns sockets left by a previous crash.
    ipc = RUNTIME / "ipc"
    ipc.mkdir(exist_ok=True)
    for endpoint in ipc.glob("cf*.sock"):
        if endpoint.is_socket():
            endpoint.unlink()
    processes = Processes(RUNTIME)
    config.update(headless=headless)
    (RUNTIME / "config.json").write_text(json.dumps(config, indent=2)+"\n")
    os.environ.update(USER="ds3", ROS_DOMAIN_ID=str(config["native_domain"]),
                      CF_NATIVE_DOMAIN=str(config["native_domain"]), CF_COMPAT_DOMAIN=str(config["compat_domain"]))
    try:
        env = os.environ.copy()
        if Path("/dev/dxg").exists() and Path("/usr/lib/wsl/lib/libd3d12.so").exists():
            env["LD_LIBRARY_PATH"] = "/usr/lib/wsl/lib:" + env.get("LD_LIBRARY_PATH", "")
            print("Webots graphics: WSLg D3D12; adapter=" + env.get("MESA_D3D12_DEFAULT_ADAPTER_NAME", "default"), flush=True)
        x11 = Path("/tmp/.X11-unix")
        if not headless and not x11.exists():
            x11.symlink_to("/host-x11", target_is_directory=True)
        if not headless and shutil.which("glxinfo"):
            graphics = subprocess.run(["glxinfo", "-B"], env=env, capture_output=True, text=True, timeout=10)
            (RUNTIME / "graphics.log").write_text(graphics.stdout + graphics.stderr)
            for line in graphics.stdout.splitlines():
                if "OpenGL renderer string:" in line or "Accelerated:" in line:
                    print(line.strip(), flush=True)
        if headless:
            # Keep Xvfb sockets private; never write into the host display mount.
            if x11.is_symlink():
                x11.unlink()
            env["DISPLAY"] = ":99"
            processes.spawn("xvfb", ["Xvfb", ":99", "-screen", "0", "1280x720x24", "-nolisten", "tcp"])
            deadline = time.monotonic()+8
            while not Path("/tmp/.X11-unix/X99").exists():
                processes.check()
                if time.monotonic() > deadline:
                    raise TimeoutError("Private Xvfb not ready")
                time.sleep(.1)
        processes.spawn("webots", ["webots", f"--port={config['webots_port']}", "--batch", "--mode=realtime",
            "--stdout", "--stderr", *( ["--no-rendering"] if headless else [] ),
            "/ds/crazywebotsworld/worlds/crazyflie.wbt"], env=env)
        sockets = Path(f"/tmp/webots/ds3/{config['webots_port']}/ipc")
        deadline = time.monotonic()+90
        while not sockets.exists():
            processes.check()
            if time.monotonic() > deadline:
                raise TimeoutError("Webots controller sockets unavailable; inspect webots.log")
            time.sleep(.2)
        # The framework launch uses the same patched gateway as manual ROS use.
        processes.spawn("gateway", ["ros2", "launch", "crazyflies", "framework.launch.py", "backend:=webots",
                                  f"webots_port:={config['webots_port']}"])
        import rclpy
        from rclpy.node import Node
        from crazyflie_interfaces.srv import AddCrazyflie
        rclpy.init()
        node = Node("webots_attach_configured_drones")
        try:
            client = node.create_client(AddCrazyflie, "/crazyflie_webots_gateway/add_crazyflie")
            if not client.wait_for_service(timeout_sec=20):
                raise RuntimeError("Webots gateway unavailable")
            for drone in config["drone_ids"]:
                processes.check()
                request = AddCrazyflie.Request()
                request.uri = f"webots://{drone}"
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                if not future.done() or not future.result().success:
                    raise RuntimeError(f"Could not attach cf{drone}")
        finally:
            node.destroy_node()
            rclpy.shutdown()
        ids = ",".join(f"cf{i}" for i in config["drone_ids"])
        source = Path("/workspace/controller/cf.PyControl/src")
        processes.spawn("swarm", ["python3", str(source / "swarm_services.py"), "--drone-ids", ids])
        processes.spawn("experiment", ["python3", str(HERE / "tools/experiment.py")])
        start_services(processes, config)
        processes.spawn("compatibility", ["/usr/local/bin/ds-env", "compat", "python3",
            str(HERE / "compat/bridge.py"), "--ipc-dir", str(ipc), "--drone-ids", ids])
        await_services(processes, config)
        deadline = time.monotonic()+20
        while True:
            processes.check()
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{config['experiment_port']}/experiment/status", timeout=.5) as response:
                    if json.load(response).get("drivers_ready"):
                        break
            except (OSError, ValueError):
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("Updated Webots grid-observation/disturbance drivers unavailable; rebuild the existing image")
            time.sleep(.1)
        # Every controller requires fresh native telemetry.
        sys.path.insert(0, str(HERE / "compat"))
        from bridge import ipc as snapshot
        deadline = time.monotonic()+30
        while True:
            processes.check()
            try:
                if all(snapshot(ipc / f"cf{i}.sock", {"action": "snapshot"})["connected"] for i in config["drone_ids"]):
                    break
            except (OSError, RuntimeError):
                pass
            if time.monotonic() > deadline:
                raise TimeoutError("Native telemetry is not ready")
            time.sleep(.2)
        (RUNTIME / "supervisor.json").write_text(json.dumps({name: p.pid for name, p in processes.children}, indent=2))
        temporary = ready.with_suffix(".tmp")
        temporary.write_text(json.dumps(dict(config, supervisor_pid=os.getpid()), indent=2)+"\n")
        temporary.replace(ready)
        print(f"READY — drones={ids}; HTTP={config['http_port']}; Webots R2025a / Humble", flush=True)
        processes.wait()
    except InterruptedError:
        pass
    finally:
        ready.unlink(missing_ok=True)
        processes.close()
        lock.close()


if __name__ == "__main__":
    main()
