#!/usr/bin/env python3
"""Offline checks, optionally followed by the actual owned Webots simulator."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from config import HERE, OWNER, RUNTIME

REPO = HERE.parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", action="store_true", help="Sends commands to this project's simulator")
    args = parser.parse_args()
    output = RUNTIME / "tests"
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
    def run(command, **kwargs):
        print("RUN", *map(str, command), flush=True)
        subprocess.run(list(map(str, command)), check=True, **kwargs)
    run([sys.executable, "-m", "pytest", "controller/cf.PyControl/tests", "-p", "no:cacheprovider", "-q",
         f"--junitxml={output / 'unit.xml'}"], cwd=REPO, env=environment)
    for name in ("motion", "disturbance"):
        binary = output / ("test-" + name)
        run(["g++", "-std=c++17", "-Wall", "-Wextra", "-pedantic", "-I/usr/include/eigen3",
             "-I"+str(HERE / "src/crazyflie_webots/src/crazyflie_webots/include"),
             HERE / ("tests/test_" + name + ".cpp"), "-o", binary])
        run([binary])
    if not args.runtime:
        return
    spec = importlib.util.spec_from_file_location("cf_launcher", REPO / "controller/cf.PyControl/bin/launch.py")
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    docker = launcher.docker_command()
    info = launcher.inspect(docker)
    if not info or not info["State"]["Running"]:
        raise RuntimeError("Start start_dssim_docker.sh --headless first")
    config = json.loads((RUNTIME / "ready.json").read_text())
    if not config["headless"]:
        raise RuntimeError("Runtime tests require private headless Xvfb")
    source = dict(image_id=info["Image"], world_sha256=hashlib.sha256((HERE / "crazyflie.wbt").read_bytes()).hexdigest(),
                  recorded_unix=time.time(), ros_native=config["native_domain"], ros_public=config["compat_domain"])
    (output / "source.json").write_text(json.dumps(source, indent=2)+"\n")
    run([sys.executable, HERE / "tests/verify_runtime.py"], env=environment)
    prefix = docker + ["exec", launcher.CONTAINER]
    inside = "/ds/ds-crazyflies"
    for overlay, suffix, extra in (("native", "native-ros", []), ("compat", "compat-ros", ["--compat-ros"])):
        run(prefix + ["/usr/local/bin/ds-env", overlay, "python3", inside+"/tests/verify_ros.py",
                      "--output", inside+f"/.runtime/tests/{suffix}-results.json", *extra])
    run(docker + ["exec", "-e", "DISPLAY=:99", launcher.CONTAINER,
                  "/usr/local/bin/ds-env", "native", "python3", inside+"/tests/verify_pause.py"])
    run(prefix + ["/usr/local/bin/ds-env", "compat", "python3", inside+"/tests/verify_consumers.py"])
    print("Simulator suites passed. External business workflows remain separately recorded.", flush=True)


if __name__ == "__main__":
    main()
