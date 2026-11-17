"""Start the same per-drone HTTP and diagram services in Docker or local ROS."""
import json
import os
from pathlib import Path
import shutil
import sys
import time
import urllib.request

if Path("/workspace/controller/cf.PyControl").exists():
    CF = Path("/workspace/controller/cf.PyControl")
else:
    CF = Path(__file__).resolve().parents[4] / "controller/cf.PyControl"


def diagram_services():
    result = [("web_port", Path(__file__).with_name("web.py"), "web")]
    wall = CF / "src/wall_following/wall_following_webview_server.py"
    if wall.is_file() and (CF / "src/wall_following/webview").is_dir():
        result.append(("wall_port", wall, "wall"))
    return result


def start_services(processes, config, simulated=True, environment=None, extra=()):
    web = processes.directory / "webview"
    shutil.copytree(CF / "webview", web, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("img", "*.log", "__pycache__"))
    if any(key == "wall_port" for key, _, _ in diagram_services()):
        shutil.copytree(CF / "src/wall_following/webview", web / "wall_following", dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns("img", "*.log", "__pycache__"))
    (web / "runtime-config.js").write_text("window.CF_RUNTIME = " + json.dumps(config) + ";\n")
    env = dict(environment or os.environ, CF_WEB_ROOT=str(web))
    for i, drone in enumerate(config["drone_ids"]):
        name = f"cf{drone}"
        current = [sys.executable, str(CF / "src/server.py"), "--drone_id", name,
                   "--host", config["bind_host"], "--web-root", str(web),
                   "--backend", "webots" if simulated else "hardware"]
        if simulated:
            current += ["--use-sim-time", "--require-command-report", "--ipc-dir", str(processes.directory / "ipc")]
        else:
            current += ["--use-wall-time"]
        command = current + ["--port", str(config["http_port"]+i), "--wsendpoint",
                             "--wshost", config["bind_host"], "--wsport", str(config["ws_port"]+i), *extra]
        processes.spawn(name, command, env=env, cwd=CF / "src")
        for key, entry, prefix in diagram_services():
            command = [sys.executable, str(entry), "--directory", str(web),
                       "--port", str(config[key]+i), "--host", config["bind_host"]]
            processes.spawn(f"{prefix}-{name}", command, env=env)


def await_services(processes, config, require_telemetry=True, timeout=100):
    pending = set(range(len(config["drone_ids"])))
    last = {}
    deadline = time.monotonic()+timeout
    while pending:
        processes.check()
        for i in list(pending):
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{config['http_port']+i}/status", timeout=.3) as response:
                    state = json.load(response)
                required = config.get("required_drone_ids", config["drone_ids"])
                last[config["drone_ids"][i]] = dict(connected=state.get("connected"), services_ready=state.get("services_ready"))
                if require_telemetry and config["drone_ids"][i] in required and (
                        not state.get("connected") or not state.get("services_ready", False)):
                    continue
                for key, _, _ in diagram_services():
                    with urllib.request.urlopen(f"http://127.0.0.1:{config[key]+i}/webview.html", timeout=.3) as response:
                        assert response.status == 200
                pending.remove(i)
            except (OSError, ValueError, AssertionError):
                pass
        if time.monotonic() > deadline:
            raise TimeoutError(f"Services not ready: {last}. For hardware, verify the upstream server, logging, radio and ROS domain.")
        processes.stop.wait(.2)
