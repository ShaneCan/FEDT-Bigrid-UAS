"""Compose readiness: owned supervisor plus every configured public endpoint."""
import json
import os
from pathlib import Path
import urllib.request


def main():
    ready = json.loads(Path("/ds/ds-crazyflies/.runtime/ready.json").read_text())
    os.kill(ready["supervisor_pid"], 0)
    for i, _ in enumerate(ready["drone_ids"]):
        with urllib.request.urlopen(f"http://127.0.0.1:{ready['http_port']+i}/status", timeout=.25) as response:
            state = json.load(response)
        if not state.get("connected"):
            raise RuntimeError("Telemetry unavailable")
    with urllib.request.urlopen(f"http://127.0.0.1:{ready['experiment_port']}/experiment/status", timeout=.5) as response:
        if not json.load(response).get("drivers_ready"):
            raise RuntimeError("Webots experiment observer unavailable")


if __name__ == "__main__":
    main()
