#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Start FNM sidecars for EV + intersections.")
    ap.add_argument("--python-bin", default=sys.executable or "python3")
    ap.add_argument("--fnm-script", required=True, help="path to federation_node_manager.py")
    ap.add_argument("--ev-config", default="", help="optional EV FNM config yaml")
    ap.add_argument("--intersection-config-dir", required=True, help="directory with fnm_intersection_*.yml")
    ap.add_argument("--intersection-config-glob", default="fnm_intersection_*.yml")
    ap.add_argument("--only-tls", default="", help="comma-separated TLS ids filter")
    ap.add_argument("--exclude-tls", default="", help="comma-separated TLS ids exclusion")
    ap.add_argument("--tick-sec", type=float, default=0.1, help="FNM tick period")
    ap.add_argument("--stagger-sec", type=float, default=0.15, help="delay between process starts")
    ap.add_argument("--log-dir", required=True, help="directory for sidecar logs")
    ap.add_argument(
        "--topic-namespace",
        default="",
        help="optional MQTT topic namespace passed to each FNM process for run isolation",
    )
    ap.add_argument("--data-base-dir", default="", help="optional base dir override for node data manager")
    ap.add_argument("--data-run-id", default="", help="optional run id override for node data manager")
    ap.add_argument(
        "--data-base-dir-default-under-logdir",
        action="store_true",
        default=True,
        help="when --data-base-dir is empty, default to <log-dir>/federation_traces",
    )
    ap.add_argument(
        "--data-persist-raw-messages",
        choices=["auto", "on", "off"],
        default="auto",
        help="override raw message persistence in node data manager",
    )
    ap.add_argument(
        "--ev-preflight-enable",
        action="store_true",
        default=False,
        help="wait for EV sidecar state-pull readiness before proceeding",
    )
    ap.add_argument("--ev-preflight-timeout-sec", type=float, default=60.0)
    ap.add_argument("--ev-preflight-poll-sec", type=float, default=0.25)
    ap.add_argument(
        "--ev-preflight-min-state-ok",
        type=int,
        default=1,
        help="minimum fnm.adapter.state_pull.ok events required",
    )
    ap.add_argument(
        "--ev-preflight-min-req-published",
        type=int,
        default=0,
        help="minimum req_published value seen in fnm.adapter.state_pull.ok",
    )
    ap.add_argument(
        "--ev-preflight-require-nearest-tls",
        action="store_true",
        default=False,
        help="require nearest_tls non-empty in at least one state_pull.ok event",
    )
    ap.add_argument(
        "--ev-preflight-max-errors",
        type=int,
        default=0,
        help="if >0, fail preflight when state_pull.error reaches this count",
    )
    ap.add_argument("--auto-generate-intersection-configs", action="store_true", default=False)
    ap.add_argument("--generator-script", default="", help="optional path to generate_fnm_intersection_configs.py")
    ap.add_argument("--generator-template", default="", help="template config for generator")
    ap.add_argument("--generator-route-file", default="", help="route file for generator")
    ap.add_argument("--generator-ev-id", default="emergency1", help="EV id for generator")
    ap.add_argument("--generator-net-file", default="", help="network file for generator")
    ap.add_argument("--generator-clean-out-dir", action="store_true", default=False)
    ap.add_argument("--dry-run", action="store_true", default=False)
    return ap.parse_args()


def _stem_to_tls(stem: str) -> str:
    s = str(stem)
    p = "fnm_intersection_"
    if s.startswith(p):
        return s[len(p):]
    return s


def _split_csv(s: str) -> List[str]:
    return [x.strip() for x in str(s or "").split(",") if x.strip()]


def _build_proc_cmd(
    py: str,
    fnm_script: str,
    cfg: str,
    tick_sec: float,
    jsonl_path: str,
    *,
    data_base_dir: str = "",
    data_run_id: str = "",
    data_persist_raw_messages: str = "auto",
    topic_namespace: str = "",
) -> List[str]:
    cmd = [
        str(py),
        str(fnm_script),
        "--config",
        str(cfg),
        "--tick-sec",
        str(float(tick_sec)),
        "--log-jsonl",
        str(jsonl_path),
    ]
    if str(data_base_dir or "").strip():
        cmd += ["--data-base-dir", str(data_base_dir)]
    if str(data_run_id or "").strip():
        cmd += ["--data-run-id", str(data_run_id)]
    if str(data_persist_raw_messages or "auto") in {"on", "off"}:
        cmd += ["--data-persist-raw-messages", str(data_persist_raw_messages)]
    if str(topic_namespace or "").strip():
        cmd += ["--topic-namespace", str(topic_namespace).strip()]
    return cmd


def _wait_ev_preflight(
    *,
    jsonl_path: str,
    timeout_sec: float,
    poll_sec: float,
    min_state_ok: int,
    min_req_published: int,
    require_nearest_tls: bool,
    max_errors: int,
    watched_procs: List[subprocess.Popen],
) -> bool:
    t0 = float(time.time())
    min_state_ok = max(1, int(min_state_ok))
    min_req_published = max(0, int(min_req_published))
    max_errors = max(0, int(max_errors))
    next_pos = 0
    ok_count = 0
    err_count = 0
    max_req = 0
    nearest_seen = False

    def _alive() -> bool:
        return any((p.poll() is None) for p in watched_procs)

    while (float(time.time()) - t0) <= float(timeout_sec):
        if not _alive():
            print("EV_PRECHECK_FAIL reason=process_exited")
            return False
        if os.path.exists(jsonl_path):
            try:
                with open(jsonl_path, "r", encoding="utf-8") as f:
                    f.seek(int(next_pos))
                    for line in f:
                        line = str(line or "").strip()
                        if not line:
                            continue
                        try:
                            obj = dict(json.loads(line))
                        except Exception:
                            continue
                        evt = str(obj.get("event", ""))
                        if evt == "fnm.adapter.state_pull.ok":
                            ok_count += 1
                            max_req = max(max_req, int(obj.get("req_published", 0) or 0))
                            if str(obj.get("nearest_tls", "") or "").strip():
                                nearest_seen = True
                        elif evt == "fnm.adapter.state_pull.error":
                            err_count += 1
                    next_pos = int(f.tell())
            except Exception:
                pass
        if max_errors > 0 and err_count >= max_errors:
            print(
                "EV_PRECHECK_FAIL "
                f"reason=too_many_errors errors={err_count} max_errors={max_errors} "
                f"ok={ok_count} max_req={max_req} nearest_seen={1 if nearest_seen else 0}"
            )
            return False
        pass_ok = (ok_count >= min_state_ok) and (max_req >= min_req_published)
        if require_nearest_tls:
            pass_ok = bool(pass_ok and nearest_seen)
        if pass_ok:
            print(
                "EV_PRECHECK_OK "
                f"ok={ok_count} max_req={max_req} nearest_seen={1 if nearest_seen else 0} "
                f"errors={err_count} elapsed_s={(float(time.time())-t0):.2f}"
            )
            return True
        time.sleep(max(0.05, float(poll_sec)))

    print(
        "EV_PRECHECK_FAIL "
        f"reason=timeout timeout_s={timeout_sec:.1f} "
        f"ok={ok_count} max_req={max_req} nearest_seen={1 if nearest_seen else 0} errors={err_count}"
    )
    return False


def main() -> None:
    args = _parse_args()
    log_dir = Path(args.log_dir).resolve()
    log_dir.mkdir(parents=True, exist_ok=True)

    only = set(_split_csv(args.only_tls))
    exclude = set(_split_csv(args.exclude_tls))

    fnm_script = Path(args.fnm_script).resolve()
    cfg_dir = Path(args.intersection_config_dir).resolve()

    if bool(args.auto_generate_intersection_configs):
        gen_script = str(args.generator_script or (Path(__file__).resolve().parent / "generate_fnm_intersection_configs.py"))
        gen_cmd = [
            str(args.python_bin),
            str(gen_script),
            "--template",
            str(args.generator_template),
            "--out-dir",
            str(cfg_dir),
            "--route-file",
            str(args.generator_route_file),
            "--ev-id",
            str(args.generator_ev_id),
            "--net-file",
            str(args.generator_net_file),
        ]
        if bool(args.generator_clean_out_dir):
            gen_cmd.append("--clean-out-dir")
        if args.dry_run:
            print("DRY_RUN", " ".join(gen_cmd))
        else:
            print("GEN", " ".join(gen_cmd))
            subprocess.check_call(gen_cmd)

    inter_cfgs = sorted(list(cfg_dir.glob(str(args.intersection_config_glob))))

    selected_inter: List[Path] = []
    for p in inter_cfgs:
        tls = _stem_to_tls(p.stem)
        if only and tls not in only:
            continue
        if tls in exclude:
            continue
        selected_inter.append(p)

    proc_specs: List[Dict[str, str]] = []
    data_base_dir_effective = str(args.data_base_dir or "").strip()
    if (not data_base_dir_effective) and bool(args.data_base_dir_default_under_logdir):
        data_base_dir_effective = str((log_dir / "federation_traces").resolve())
        os.makedirs(data_base_dir_effective, exist_ok=True)
    if str(args.ev_config).strip():
        ev_cfg = Path(args.ev_config).resolve()
        proc_specs.append(
            {
                "name": "fnm_ev",
                "cfg": str(ev_cfg),
                "jsonl": str(log_dir / "fnm_ev.jsonl"),
                "stdout": str(log_dir / "fnm_ev.stdout.log"),
            }
        )

    for p in selected_inter:
        tls = _stem_to_tls(p.stem)
        proc_specs.append(
            {
                "name": f"fnm_{tls}",
                "cfg": str(p),
                "jsonl": str(log_dir / f"fnm_{tls}.jsonl"),
                "stdout": str(log_dir / f"fnm_{tls}.stdout.log"),
            }
        )

    if not proc_specs:
        raise SystemExit("No sidecars selected. Check config dir and filters.")

    if args.dry_run:
        for spec in proc_specs:
            cmd = _build_proc_cmd(
                args.python_bin,
                str(fnm_script),
                spec["cfg"],
                float(args.tick_sec),
                spec["jsonl"],
                data_base_dir=str(data_base_dir_effective),
                data_run_id=str(args.data_run_id or ""),
                data_persist_raw_messages=str(args.data_persist_raw_messages or "auto"),
                topic_namespace=str(args.topic_namespace or ""),
            )
            print("DRY_RUN", " ".join(cmd))
        return

    procs: List[subprocess.Popen] = []

    def _stop_all(*_args) -> None:
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass

    signal.signal(signal.SIGINT, _stop_all)
    signal.signal(signal.SIGTERM, _stop_all)

    for spec in proc_specs:
        cmd = _build_proc_cmd(
            args.python_bin,
            str(fnm_script),
            spec["cfg"],
            float(args.tick_sec),
            spec["jsonl"],
            data_base_dir=str(data_base_dir_effective),
            data_run_id=str(args.data_run_id or ""),
            data_persist_raw_messages=str(args.data_persist_raw_messages or "auto"),
            topic_namespace=str(args.topic_namespace or ""),
        )
        fp = open(spec["stdout"], "a", encoding="utf-8")
        p = subprocess.Popen(cmd, stdout=fp, stderr=subprocess.STDOUT)
        procs.append(p)
        print(f"START name={spec['name']} pid={p.pid} cfg={spec['cfg']} jsonl={spec['jsonl']}")
        time.sleep(max(0.0, float(args.stagger_sec)))

    if bool(args.ev_preflight_enable) and str(args.ev_config).strip():
        ev_spec = next((s for s in proc_specs if s.get("name") == "fnm_ev"), None)
        ev_jsonl = str((ev_spec or {}).get("jsonl", "") or "")
        if ev_jsonl:
            ok = _wait_ev_preflight(
                jsonl_path=ev_jsonl,
                timeout_sec=float(args.ev_preflight_timeout_sec),
                poll_sec=float(args.ev_preflight_poll_sec),
                min_state_ok=int(args.ev_preflight_min_state_ok),
                min_req_published=int(args.ev_preflight_min_req_published),
                require_nearest_tls=bool(args.ev_preflight_require_nearest_tls),
                max_errors=int(args.ev_preflight_max_errors),
                watched_procs=procs,
            )
            if not ok:
                _stop_all()
                for p in procs:
                    try:
                        p.wait(timeout=2.0)
                    except Exception:
                        pass
                raise SystemExit(2)

    try:
        while True:
            alive = 0
            for p in procs:
                if p.poll() is None:
                    alive += 1
            if alive == 0:
                break
            time.sleep(0.5)
    finally:
        _stop_all()
        for p in procs:
            try:
                p.wait(timeout=2.0)
            except Exception:
                pass
        print("FNM sidecars stopped.")


if __name__ == "__main__":
    main()
