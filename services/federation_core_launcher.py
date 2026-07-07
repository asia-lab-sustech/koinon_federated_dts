import argparse
import os
import signal
import subprocess
import sys
import time
from typing import List


def _script_path(name: str) -> str:
    return os.path.join(os.path.dirname(__file__), name)


def _start(cmd: List[str]) -> subprocess.Popen:
    return subprocess.Popen(cmd)


def parse_args():
    ap = argparse.ArgumentParser(description="Launch split federation core services")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--log-dir", default="", help="Optional directory to persist each service JSONL log")
    ap.add_argument("--with-metrics", action="store_true", default=True)
    ap.add_argument("--without-metrics", dest="with_metrics", action="store_false")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    py = sys.executable
    procs: List[subprocess.Popen] = []

    log_dir = str(args.log_dir or "").strip()
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)

    def _log_arg(name: str) -> List[str]:
        if not log_dir:
            return []
        return ["--log-jsonl", os.path.join(log_dir, f"{name}.jsonl")]

    membership_cmd = [
        py,
        _script_path("federation_membership_service.py"),
        "--mqtt-host", str(args.mqtt_host),
        "--mqtt-port", str(args.mqtt_port),
    ] + _log_arg("membership")

    catalog_cmd = [
        py,
        _script_path("federation_catalog_service.py"),
        "--mqtt-host", str(args.mqtt_host),
        "--mqtt-port", str(args.mqtt_port),
    ] + _log_arg("catalog")

    discovery_cmd = [
        py,
        _script_path("federation_discovery_service.py"),
        "--mqtt-host", str(args.mqtt_host),
        "--mqtt-port", str(args.mqtt_port),
    ] + _log_arg("discovery")

    print("[LAUNCHER] starting membership service")
    procs.append(_start(membership_cmd))
    time.sleep(0.2)

    print("[LAUNCHER] starting catalog service")
    procs.append(_start(catalog_cmd))
    time.sleep(0.2)

    print("[LAUNCHER] starting discovery service")
    procs.append(_start(discovery_cmd))
    time.sleep(0.2)

    if bool(args.with_metrics):
        metrics_cmd = [
            py,
            _script_path("federation_metrics_service.py"),
            "--mqtt-host", str(args.mqtt_host),
            "--mqtt-port", str(args.mqtt_port),
        ] + _log_arg("metrics")
        print("[LAUNCHER] starting metrics service")
        procs.append(_start(metrics_cmd))

    stop_flag = {"stop": False}

    def _stop(_sig, _frm):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while not stop_flag["stop"]:
            alive = [p for p in procs if p.poll() is None]
            if not alive:
                print("[LAUNCHER] all services exited")
                break
            time.sleep(0.5)
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
        time.sleep(0.4)
        for p in procs:
            if p.poll() is None:
                p.kill()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
