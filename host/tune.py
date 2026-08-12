"""Record one trial or run one human-confirmed assisted-tuning step."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import re
import sys
import tomllib

from .transport import run_ble_program
from .scoring import score_csv
from .search import (
    SEARCH_SCHEMA_VERSION,
    current_candidate,
    migrate_search_state,
    new_search_state,
    record_result,
)


ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = ROOT / "results"
PROGRAM = ROOT / "hub" / "balancer.py"
OFFICIAL_REFERENCE_PROGRAM = ROOT / "hub" / "official_reference.py"
CONFIG = ROOT / "config.toml"
ASSIST_STATE = RESULTS_DIR / "assist-state.json"
STAGED_SEARCH_STATE = RESULTS_DIR / "staged-search-state.json"

TERMINAL_EVENTS = ("TRIAL_COMPLETE", "FALLEN", "ABORTED", "ERROR")
UNSAFE_SURVIVAL_S = 0.5
UNSAFE_SATURATION_FRACTION = 0.25


def is_unsafe_trial(terminal_event: str | None, metrics: dict | None) -> bool:
    """Distinguish an ordinary tuning fall from a repeatedly unsafe outcome."""
    if terminal_event != "FALLEN" or metrics is None:
        return False
    return (
        float(metrics["survival_s"]) < UNSAFE_SURVIVAL_S
        or float(metrics["saturation_fraction"]) >= UNSAFE_SATURATION_FRACTION
    )


def is_valid_tuning_start(metrics: dict, tolerance_deg: float) -> bool:
    """Reject a released pose that drifted outside the stable arming envelope."""
    return float(metrics["initial_abs_angle_error_deg"]) <= tolerance_deg


class TrialRecorder:
    """Preserve all output while extracting valid telemetry rows."""

    def __init__(self, session_dir: Path):
        self.raw_path = session_dir / "hub-output.log"
        self.telemetry_path = session_dir / "telemetry.csv"
        self.raw_file = self.raw_path.open("w", encoding="utf-8")
        self.telemetry_file = self.telemetry_path.open("w", encoding="utf-8", newline="")
        self.writer = csv.writer(self.telemetry_file)
        self.header: list[str] | None = None
        self.telemetry_rows = 0
        self.events: list[str] = []
        self.terminal_event: str | None = None

    def handle_line(self, raw_line: str) -> None:
        self.raw_file.write(raw_line)
        self.raw_file.flush()

        line = raw_line.strip("\r\n")
        if line.startswith("timestamp_ms,"):
            self.header = next(csv.reader([line]))
            self.writer.writerow(self.header)
            self.telemetry_file.flush()
            return

        event_name = line.split(",", 1)[0]
        if event_name in (
            "READY", "READY_AUTO", "READY_OFFICIAL", "WAITING_AUTO", "WAITING_OFFICIAL",
            "UPRIGHT_STABLE", "COUNTDOWN",
            "COUNTDOWN_CANCELLED", "START_REJECTED", "ARMED", "ARMED_AUTO",
            "TRIAL_STARTED", "TRIAL_COMPLETE", "FALLEN", "ABORTED", "ERROR",
            "TIMING", "STOPPED",
        ):
            self.events.append(line)
            if event_name in TERMINAL_EVENTS:
                self.terminal_event = event_name
            return

        if self.header is None or not line or not line[0].isdigit():
            return

        row = next(csv.reader([line]))
        if len(row) != len(self.header):
            return
        self.writer.writerow(row)
        self.telemetry_rows += 1
        self.telemetry_file.flush()

    def close(self) -> None:
        self.raw_file.close()
        self.telemetry_file.close()


def create_session_dir(now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now().astimezone()).strftime("%Y-%m-%d_%H%M%S")
    session_dir = RESULTS_DIR / timestamp
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def render_trial_files(
    session_dir: Path,
    kp: float,
    kd: float,
    *,
    ki: float | None = None,
    target_angle_deg: float | None = None,
    mode: str | None = None,
    kw: float | None = None,
    kx: float | None = None,
    auto_arm: bool = False,
) -> tuple[Path, dict]:
    """Create exact session-local program/config artifacts for proposed gains."""
    program_text = PROGRAM.read_text(encoding="utf-8")
    base_config = tomllib.loads(CONFIG.read_text(encoding="utf-8"))
    ki = base_config["controller"]["ki"] if ki is None else ki
    target_angle_deg = (
        base_config["controller"]["target_angle_deg"]
        if target_angle_deg is None
        else target_angle_deg
    )
    mode = base_config["controller"]["mode"] if mode is None else mode
    kw = base_config["controller"]["kw"] if kw is None else kw
    kx = base_config["controller"]["kx"] if kx is None else kx
    program_text, mode_count = re.subn(
        r'^CONTROLLER_MODE = "(?:P|PD|PID|REFERENCE)"$',
        f'CONTROLLER_MODE = "{mode}"',
        program_text,
        flags=re.MULTILINE,
    )
    program_text, target_count = re.subn(
        r"^TARGET_ANGLE_DEG = [-+0-9.eE]+$",
        f"TARGET_ANGLE_DEG = {target_angle_deg}",
        program_text,
        flags=re.MULTILINE,
    )
    program_text, kp_count = re.subn(
        r"^KP = [-+0-9.eE]+$", f"KP = {kp}", program_text, flags=re.MULTILINE
    )
    program_text, kd_count = re.subn(
        r"^KD = [-+0-9.eE]+$", f"KD = {kd}", program_text, flags=re.MULTILINE
    )
    program_text, kw_count = re.subn(
        r"^KW = [-+0-9.eE]+$", f"KW = {kw}", program_text, flags=re.MULTILINE
    )
    program_text, kx_count = re.subn(
        r"^KX = [-+0-9.eE]+$", f"KX = {kx}", program_text, flags=re.MULTILINE
    )
    program_text, gyro_filter_count = re.subn(
        r"^GYRO_FILTER_ALPHA = [-+0-9.eE]+$",
        f"GYRO_FILTER_ALPHA = {base_config['controller']['gyro_filter_alpha']}",
        program_text,
        flags=re.MULTILINE,
    )
    program_text, ki_count = re.subn(
        r"^KI = [-+0-9.eE]+$", f"KI = {ki}", program_text, flags=re.MULTILINE
    )
    program_text, auto_count = re.subn(
        r"^AUTO_ARM = (?:True|False)$",
        f"AUTO_ARM = {auto_arm}",
        program_text,
        flags=re.MULTILINE,
    )
    program_text, prediction_min_count = re.subn(
        r"^PREDICTED_FALL_MIN_ERROR_DEG = [-+0-9.eE]+$",
        f"PREDICTED_FALL_MIN_ERROR_DEG = {base_config['safety']['predicted_fall_min_error_deg']}",
        program_text,
        flags=re.MULTILINE,
    )
    if (
        mode_count, target_count, kp_count, ki_count, kd_count, auto_count,
        prediction_min_count, gyro_filter_count, kw_count, kx_count,
    ) != (1, 1, 1, 1, 1, 1, 1, 1, 1, 1):
        raise ValueError("could not locate unique rendered controller constants")
    program_path = session_dir / "hub-program.py"
    program_path.write_text(program_text, encoding="utf-8")

    config_text = CONFIG.read_text(encoding="utf-8")
    config_text, kp_count = re.subn(
        r"^kp = [-+0-9.eE]+$", f"kp = {kp}", config_text, flags=re.MULTILINE
    )
    config_text, kd_count = re.subn(
        r"^kd = [-+0-9.eE]+$", f"kd = {kd}", config_text, flags=re.MULTILINE
    )
    config_text, kw_count = re.subn(
        r"^kw = [-+0-9.eE]+$", f"kw = {kw}", config_text, flags=re.MULTILINE
    )
    config_text, kx_count = re.subn(
        r"^kx = [-+0-9.eE]+$", f"kx = {kx}", config_text, flags=re.MULTILINE
    )
    config_text, ki_count = re.subn(
        r"^ki = [-+0-9.eE]+$", f"ki = {ki}", config_text, flags=re.MULTILINE
    )
    config_text, target_count = re.subn(
        r"^target_angle_deg = [-+0-9.eE]+$",
        f"target_angle_deg = {target_angle_deg}",
        config_text,
        flags=re.MULTILINE,
    )
    config_text, mode_count = re.subn(
        r'^mode = "(?:P|PD|PID|REFERENCE)"$', f'mode = "{mode}"', config_text, flags=re.MULTILINE
    )
    if (mode_count, target_count, kp_count, ki_count, kd_count, kw_count, kx_count) != (1, 1, 1, 1, 1, 1, 1):
        raise ValueError("could not locate unique rendered config values")
    (session_dir / "config.toml").write_text(config_text, encoding="utf-8")
    return program_path, tomllib.loads(config_text)


def run_trial(
    hub_name: str | None,
    kp: float | None = None,
    kd: float | None = None,
    *,
    ki: float | None = None,
    target_angle_deg: float | None = None,
    mode: str | None = None,
    kw: float | None = None,
    kx: float | None = None,
    session_dir: Path | None = None,
    auto_arm: bool = False,
):
    started = datetime.now().astimezone()
    if session_dir is None:
        session_dir = create_session_dir(started)
    else:
        session_dir.mkdir(parents=True, exist_ok=False)
    base_config = tomllib.loads(CONFIG.read_text(encoding="utf-8"))
    kp = base_config["controller"]["kp"] if kp is None else kp
    kd = base_config["controller"]["kd"] if kd is None else kd
    program, config = render_trial_files(
        session_dir,
        kp,
        kd,
        ki=ki,
        target_angle_deg=target_angle_deg,
        mode=mode,
        kw=kw,
        kx=kx,
        auto_arm=auto_arm,
    )
    recorder = TrialRecorder(session_dir)

    metadata = {
        "schema_version": 1,
        "started_at": started.isoformat(),
        "ended_at": None,
        "program": str(program.relative_to(ROOT)),
        "source_program": str(PROGRAM.relative_to(ROOT)),
        "hub_name": hub_name,
        "auto_arm": auto_arm,
        "config": config,
        "returncode": None,
        "terminal_event": None,
        "telemetry_rows": 0,
        "events": [],
    }
    write_json(session_dir / "session.json", metadata)
    print(f"SESSION,{session_dir}", flush=True)

    try:
        returncode = run_ble_program(program, recorder.handle_line, hub_name)
    finally:
        recorder.close()
        metrics = None
        if recorder.telemetry_rows:
            metrics = score_csv(
                recorder.telemetry_path,
                recorder.events,
                trial_duration_s=config["safety"]["trial_duration_s"],
                output_limit=config["controller"]["output_limit"],
            )
            write_json(session_dir / "metrics.json", metrics)
        metadata.update(
            ended_at=datetime.now().astimezone().isoformat(),
            returncode=locals().get("returncode"),
            terminal_event=recorder.terminal_event,
            telemetry_rows=recorder.telemetry_rows,
            events=recorder.events,
            metrics=metrics,
        )
        write_json(session_dir / "session.json", metadata)
        print(f"SAVED,{session_dir}", flush=True)

    if returncode != 0:
        print(f"Host transport exited with status {returncode}.", file=sys.stderr)
    if recorder.telemetry_rows == 0:
        print("No telemetry rows were captured.", file=sys.stderr)
        return returncode or 2, session_dir, None
    return returncode, session_dir, metrics


def propose_candidate(config: dict) -> tuple[float, float]:
    """Propose one conservative bounded D refinement around the current best."""
    kp = float(config["controller"]["kp"])
    kd = float(config["controller"]["kd"])
    return kp, round(min(0.5, kd + 0.05), 3)


def best_previous_score(exclude: Path | None = None) -> float | None:
    scores = []
    for metrics_path in RESULTS_DIR.glob("**/metrics.json"):
        if exclude is not None and metrics_path.parent == exclude:
            continue
        try:
            scores.append(float(json.loads(metrics_path.read_text())["score"]))
        except (KeyError, ValueError, json.JSONDecodeError):
            continue
    return max(scores) if scores else None


def choose_next_kd(base_kd: float, trial_kd: float, score: float, prior_best: float | None):
    if prior_best is None or score > prior_best:
        return round(min(0.5, trial_kd + 0.05), 3), (
            "score improved; continue the same bounded D direction"
        )
    return round((base_kd + trial_kd) / 2.0, 3), (
        "score did not beat the prior best; refine between both values"
    )


def assist(hub_name: str | None) -> int:
    config = tomllib.loads(CONFIG.read_text(encoding="utf-8"))
    base_kp = float(config["controller"]["kp"])
    base_kd = float(config["controller"]["kd"])
    if ASSIST_STATE.exists():
        pending = json.loads(ASSIST_STATE.read_text(encoding="utf-8"))
        kp = float(pending["kp"])
        kd = float(pending["kd"])
        proposal_source = "saved next proposal"
    else:
        kp, kd = propose_candidate(config)
        proposal_source = "new bounded proposal"
    print("PROPOSAL")
    print(f"  current: Kp={base_kp}, Kd={base_kd}")
    print(f"  trial:   Kp={kp}, Kd={kd}")
    print(f"  source:  {proposal_source}")
    print("  bounds:  1 <= Kp <= 20, 0 <= Kd <= 0.5")
    confirmation = input("Type Y to launch exactly one physical trial: ")
    if confirmation != "Y":
        print("CANCELLED,no trial launched")
        return 0

    returncode, session_dir, metrics = run_trial(hub_name, kp, kd)
    if metrics is None:
        return returncode or 2

    previous_best = best_previous_score(exclude=session_dir)
    score = float(metrics["score"])
    next_kd, reason = choose_next_kd(base_kd, kd, score, previous_best)
    write_json(
        ASSIST_STATE,
        {
            "kp": kp,
            "kd": next_kd,
            "reason": reason,
            "based_on_session": str(session_dir.relative_to(ROOT)),
            "score": score,
            "prior_best_score": previous_best,
        },
    )
    print("NEXT_PROPOSAL_NOT_RUN")
    print(f"  Kp={kp}, Kd={next_kd}")
    print(f"  reason: {reason}")
    return returncode


def best_recorded_candidate() -> dict | None:
    best = None
    for metrics_path in RESULTS_DIR.glob("**/metrics.json"):
        session_path = metrics_path.parent / "session.json"
        if not session_path.exists():
            continue
        try:
            metrics = json.loads(metrics_path.read_text())
            session = json.loads(session_path.read_text())
            candidate = {
                "kp": session["config"]["controller"]["kp"],
                "kd": session["config"]["controller"]["kd"],
                "score": metrics["score"],
                "survival_s": metrics["survival_s"],
                "session": str(metrics_path.parent.relative_to(ROOT)),
            }
        except (KeyError, ValueError, json.JSONDecodeError):
            continue
        if best is None or candidate["score"] > best["score"]:
            best = candidate
    return best


def _state_recommendation(state: dict) -> dict | None:
    if state.get("recommendation"):
        recommendation = dict(state["recommendation"])
        matching = [
            entry for entry in state["history"]
            if entry["candidate"] == state["recommendation"]
        ]
    else:
        matching = state["history"]
        if not matching:
            return None
        recommendation = dict(max(matching, key=lambda item: item["metrics"]["score"])["candidate"])
    if matching:
        evidence = max(matching, key=lambda item: item["metrics"]["score"])
        recommendation["evidence"] = {
            "score": evidence["metrics"]["score"],
            "survival_s": evidence["metrics"]["survival_s"],
            "rms_angle_error_deg": evidence["metrics"]["rms_angle_error_deg"],
            "session": evidence["session"],
        }
    recommendation["stage"] = state["stage"]
    recommendation["decision"] = state["decision"]
    return recommendation


def semi_auto(hub_name: str | None, max_trials: int, reset_search: bool) -> int:
    config = tomllib.loads(CONFIG.read_text(encoding="utf-8"))
    if STAGED_SEARCH_STATE.exists() and not reset_search:
        staged_state = json.loads(STAGED_SEARCH_STATE.read_text(encoding="utf-8"))
        original_schema = staged_state.get("schema_version")
        staged_state = migrate_search_state(staged_state, config)
        if original_schema == SEARCH_SCHEMA_VERSION:
            state_source = "continued saved staged search"
        else:
            write_json(STAGED_SEARCH_STATE, staged_state)
            state_source = "migrated saved evidence to confirmation methodology"
    else:
        staged_state = new_search_state(config)
        write_json(STAGED_SEARCH_STATE, staged_state)
        state_source = "new staged search"

    pending_candidate = current_candidate(staged_state)
    if pending_candidate is None:
        if staged_state.get("blocked"):
            print("SEARCH_BLOCKED," + staged_state["decision"])
            return 2
        print("SEARCH_ALREADY_COMPLETE")
        print(json.dumps(_state_recommendation(staged_state), indent=2))
        return 0

    print("SEMI_AUTOMATIC_STAGED_SEARCH")
    print(f"  source: {state_source}")
    print(f"  stage: {staged_state['stage']}")
    print(f"  decision: {staged_state['decision']}")
    print("  first candidate: mode={mode}, Kp={kp}, Ki={ki}, Kd={kd}, target={target_angle_deg}".format(
        **pending_candidate
    ))
    print(f"  maximum trials: {max_trials}")
    print("  motors remain off until upright is stable and the hub finishes 3-2-1")
    print("  CENTER aborts a running trial; Bluetooth button is firmware emergency stop")
    confirmation = input("Type Y to authorize this bounded multi-trial session: ")
    if confirmation != "Y":
        print("CANCELLED,no trial launched")
        return 0

    search_started = datetime.now().astimezone()
    search_dir = RESULTS_DIR / search_started.strftime("search_%Y-%m-%d_%H%M%S")
    search_dir.mkdir(parents=True, exist_ok=False)
    trials = []
    consecutive_unsafe_trials = 0
    stop_reason = "maximum_trials"
    returncode = 0

    for trial_number in range(1, max_trials + 1):
        pending_candidate = current_candidate(staged_state)
        if pending_candidate is None:
            stop_reason = "methodology_complete"
            break
        print(
            "CANDIDATE,{trial},stage={stage},mode={mode},kp={kp},ki={ki},kd={kd},target={target_angle_deg}".format(
                trial=trial_number, stage=staged_state["stage"], **pending_candidate
            )
        )
        trial_dir = search_dir / f"trial_{trial_number:03d}"
        returncode, session_dir, metrics = run_trial(
            hub_name,
            pending_candidate["kp"],
            pending_candidate["kd"],
            ki=pending_candidate["ki"],
            target_angle_deg=pending_candidate["target_angle_deg"],
            mode=pending_candidate["mode"],
            kw=pending_candidate.get("kw", 0.0),
            kx=pending_candidate.get("kx", 0.0),
            session_dir=trial_dir,
            auto_arm=True,
        )
        session = json.loads((session_dir / "session.json").read_text())
        trial_record = {
            "trial": trial_number,
            "stage": staged_state["stage"],
            "candidate": pending_candidate,
            "session": str(session_dir.relative_to(ROOT)),
            "terminal_event": session["terminal_event"],
            "metrics": metrics,
            "accepted_for_tuning": None,
        }
        trials.append(trial_record)

        if returncode != 0 or metrics is None or session["terminal_event"] in ("ERROR", "ABORTED"):
            stop_reason = "transport_or_manual_abort"
            break

        start_tolerance = float(config["safety"]["start_angle_tolerance_deg"])
        if not is_valid_tuning_start(metrics, start_tolerance):
            trial_record["accepted_for_tuning"] = False
            print(
                "TRIAL_REJECTED,start_error={:.3f},limit={:.3f},candidate_will_retry".format(
                    metrics["initial_abs_angle_error_deg"], start_tolerance
                )
            )
            continue

        trial_record["accepted_for_tuning"] = True

        record_result(
            staged_state,
            metrics,
            str(session_dir.relative_to(ROOT)),
        )
        write_json(STAGED_SEARCH_STATE, staged_state)

        if is_unsafe_trial(session["terminal_event"], metrics):
            consecutive_unsafe_trials += 1
        else:
            consecutive_unsafe_trials = 0
        if consecutive_unsafe_trials >= 3:
            stop_reason = "three_consecutive_unsafe_trials"
            break
        next_candidate = current_candidate(staged_state)
        if next_candidate is None:
            stop_reason = (
                "methodology_blocked_no_recovery"
                if staged_state.get("blocked")
                else "methodology_complete"
            )
            break
        print(
            "RECOMPUTED,stage={stage},mode={mode},kp={kp},ki={ki},kd={kd},target={target_angle_deg},reason={reason}".format(
                stage=staged_state["stage"], reason=staged_state["decision"], **next_candidate
            )
        )

    recommendation = _state_recommendation(staged_state)
    search = {
        "schema_version": 1,
        "started_at": search_started.isoformat(),
        "ended_at": datetime.now().astimezone().isoformat(),
        "max_trials": max_trials,
        "stop_reason": stop_reason,
        "trials": trials,
        "staged_state": staged_state,
        "recommendation": recommendation,
    }
    write_json(search_dir / "search.json", search)
    write_json(search_dir / "recommendation.json", recommendation)
    print(f"SEARCH_COMPLETE,reason={stop_reason},directory={search_dir}")
    if recommendation:
        print("RECOMMENDATION,mode={mode},kp={kp},ki={ki},kd={kd},target={target_angle_deg}".format(
            **recommendation
        ))
    return returncode


def run_reference(hub_name: str | None) -> int:
    """Run one isolated translation of the visual-program PID controller."""
    print("REFERENCE_CONTROLLER")
    print("  target=88.95, Kp=5.5, Ki=2.1, Kd=4.0")
    print("  discrete integral step=0.25, derivative=error-last_error")
    print("  final scale=2.5 percent, mapped to regulated motor speed")
    confirmation = input("Type Y to authorize one catch-ready reference trial: ")
    if confirmation != "Y":
        print("CANCELLED,no trial launched")
        return 0
    return run_trial(
        hub_name,
        5.5,
        4.0,
        ki=2.1,
        target_angle_deg=88.95,
        mode="REFERENCE",
        kw=0.0,
        kx=0.0,
        auto_arm=True,
    )[0]


def run_official_reference(hub_name: str | None) -> int:
    """Run one safety-wrapped adaptation of the official Pybricks balancer."""
    print("OFFICIAL_PYBRICKS_REFERENCE")
    print("  5 ms loop, integrated gyro angle, 300 ms wheel-speed window")
    print("  raw duty with rate + angle + position + speed feedback")
    confirmation = input("Type Y to authorize one catch-ready official reference trial: ")
    if confirmation != "Y":
        print("CANCELLED,no trial launched")
        return 0
    session_dir = create_session_dir()
    program_path = session_dir / "hub-program.py"
    program_path.write_text(OFFICIAL_REFERENCE_PROGRAM.read_text(), encoding="utf-8")
    recorder = TrialRecorder(session_dir)
    try:
        returncode = run_ble_program(program_path, recorder.handle_line, hub_name)
    finally:
        recorder.close()
    print(f"SAVED,{session_dir}")
    return returncode


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    run_parser = subparsers.add_parser("run", help="run and record one physical trial")
    run_parser.add_argument("--name", help="Pybricks Bluetooth hub name")
    run_parser.add_argument("--kp", type=float, help="session-only Kp override")
    run_parser.add_argument("--kd", type=float, help="session-only Kd override")
    assist_parser = subparsers.add_parser(
        "assist", help="propose, confirm, run, and score exactly one candidate"
    )
    assist_parser.add_argument("--name", help="Pybricks Bluetooth hub name")
    auto_parser = subparsers.add_parser(
        "semi-auto", help="run a bounded search with automatic upright re-arming"
    )
    auto_parser.add_argument("--name", help="Pybricks Bluetooth hub name")
    auto_parser.add_argument("--max-trials", type=int, default=5, choices=range(1, 11))
    auto_parser.add_argument(
        "--reset-search",
        action="store_true",
        help="start the four-stage methodology from P-only coarse search",
    )
    reference_parser = subparsers.add_parser(
        "reference", help="run one isolated visual-program PID translation"
    )
    reference_parser.add_argument("--name", help="Pybricks Bluetooth hub name")
    official_parser = subparsers.add_parser(
        "official-reference", help="run the adapted official Pybricks balancer"
    )
    official_parser.add_argument("--name", help="Pybricks Bluetooth hub name")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "run":
        return run_trial(args.name, args.kp, args.kd)[0]
    if args.command == "assist":
        return assist(args.name)
    if args.command == "semi-auto":
        return semi_auto(args.name, args.max_trials, args.reset_search)
    if args.command == "reference":
        return run_reference(args.name)
    if args.command == "official-reference":
        return run_official_reference(args.name)
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
