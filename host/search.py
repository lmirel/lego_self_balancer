"""Persisted, understandable four-stage gain-search methodology."""

from __future__ import annotations

from copy import deepcopy
from statistics import median


SEARCH_SCHEMA_VERSION = 20
CONFIRMATION_TRIALS = 3
P_COARSE = (160.0, 200.0, 240.0, 320.0)
KD_COARSE = (0.0, 0.5, 1.0, 2.0)
TARGET_OFFSETS = (-0.4, -0.2, 0.0, 0.2, 0.4)
EARLY_TARGET_OFFSETS = (-0.4, -0.2, 0.0)
KI_CANDIDATES = (0.01, 0.02, 0.05)
KW_COARSE = (0.0, 0.1, 0.2, 0.4)
KX_COARSE = (0.0, 0.1, 0.25, 0.5)
BIAS_THRESHOLD_DEG = 0.5
MIN_SURVIVAL_FOR_TARGET_S = 3.0


def candidate(mode, kp, ki, kd, target, kw=0.0, kx=0.0):
    return {
        "mode": mode,
        "kp": round(float(kp), 4),
        "ki": round(float(ki), 4),
        "kd": round(float(kd), 4),
        "kw": round(float(kw), 4),
        "kx": round(float(kx), 4),
        "target_angle_deg": round(float(target), 4),
    }


def new_search_state(config: dict) -> dict:
    target = float(config["controller"]["target_angle_deg"])
    return {
        "schema_version": SEARCH_SCHEMA_VERSION,
        "stage": "p_coarse",
        "base_target_angle_deg": target,
        "queue": [candidate("P", kp, 0, 0, target) for kp in P_COARSE],
        "history": [],
        "selected": {},
        "complete": False,
        "blocked": False,
        "decision": "Tune Kp first with Ki=Kd=0.",
    }


def current_candidate(state: dict) -> dict | None:
    return state["queue"][0] if state["queue"] else None


def _results(state: dict, stages: tuple[str, ...]) -> list[dict]:
    return [entry for entry in state["history"] if entry["stage"] in stages]


def _best(entries: list[dict]) -> dict:
    groups = _groups(entries)
    winning_key = max(groups, key=lambda key: median(_scores(groups[key])))
    winning = groups[winning_key]
    median_score = median(_scores(winning))
    return min(winning, key=lambda entry: abs(entry["metrics"]["score"] - median_score))


def _candidate_key(value: dict) -> tuple:
    return tuple((field, value[field]) for field in sorted(value))


def _groups(entries: list[dict]) -> dict[tuple, list[dict]]:
    groups: dict[tuple, list[dict]] = {}
    for entry in entries:
        groups.setdefault(_candidate_key(entry["candidate"]), []).append(entry)
    return groups


def _scores(entries: list[dict]) -> list[float]:
    return [float(entry["metrics"]["score"]) for entry in entries]


def _top_candidates(entries: list[dict], count: int = 2) -> list[dict]:
    groups = _groups(entries)
    ranked = sorted(
        groups.values(),
        key=lambda group: median(_scores(group)),
        reverse=True,
    )
    return [dict(group[0]["candidate"]) for group in ranked[:count]]


def _confirmation_queue(entries: list[dict]) -> list[dict]:
    groups = _groups(entries)
    queue = []
    for value in _top_candidates(entries):
        completed = len(groups[_candidate_key(value)])
        queue.extend(dict(value) for _ in range(max(0, CONFIRMATION_TRIALS - completed)))
    return queue


def _confirmed_entries(state: dict, exploration_stages: tuple[str, ...], confirmation_stage: str) -> list[dict]:
    """Return evidence only for candidates admitted to the top-two confirmation."""
    exploration = _results(state, exploration_stages)
    confirmation = _results(state, (confirmation_stage,))
    admitted = {_candidate_key(entry["candidate"]) for entry in confirmation}
    for key, group in _groups(exploration).items():
        if len(group) >= CONFIRMATION_TRIALS:
            admitted.add(key)
    return [
        entry for entry in _results(state, exploration_stages + (confirmation_stage,))
        if _candidate_key(entry["candidate"]) in admitted
    ]


def _confirmed_winner(
    state: dict, exploration_stages: tuple[str, ...], confirmation_stage: str, parameter: str
) -> dict | None:
    entries = _confirmed_entries(state, exploration_stages, confirmation_stage)
    best = _best(entries)
    winner_key = _candidate_key(best["candidate"])
    winner_entries = [
        entry for entry in entries
        if _candidate_key(entry["candidate"]) == winner_key
    ]
    recoveries = sum(
        bool(entry["metrics"].get("recovered_toward_upright", False))
        for entry in winner_entries
    )
    if recoveries < 2:
        state["blocked"] = True
        state["queue"] = []
        state["decision"] = (
            f"Stop before locking {parameter}: the leading candidate recovered "
            f"toward upright in only {recoveries}/3 confirmation trials."
        )
        return None
    return best


def _refinement_values(entries: list[dict], field: str) -> list[float]:
    values = sorted({float(entry["candidate"][field]) for entry in entries})
    best_value = float(_best(entries)["candidate"][field])
    index = values.index(best_value)
    neighbors = []
    if index > 0:
        neighbors.append(values[index - 1])
    if index + 1 < len(values):
        neighbors.append(values[index + 1])
    return [round((best_value + neighbor) / 2.0, 4) for neighbor in neighbors]


def _set_queue(state: dict, stage: str, queue: list[dict], decision: str) -> None:
    state["stage"] = stage
    state["queue"] = queue
    state["decision"] = decision


def _seed_pd_baseline(state: dict, p_entries: list[dict], kp: float, target: float) -> None:
    """Reuse confirmed P trials as the mathematically identical Kd=0 baseline."""
    baseline_candidate = candidate("PD", kp, 0, 0, target)
    for entry in p_entries:
        if entry["candidate"]["kp"] == kp:
            state["history"].append({
                "stage": "pd_baseline",
                "candidate": dict(baseline_candidate),
                "metrics": dict(entry["metrics"]),
                "session": entry["session"],
            })


def _advance_stage(state: dict) -> None:
    stage = state["stage"]
    target = state["base_target_angle_deg"]

    if stage == "pd_sign_test":
        entries = _results(state, ("pd_sign_baseline", "pd_sign_test"))
        best = _best(entries)
        state["selected"]["kd"] = best["candidate"]["kd"]
        state["blocked"] = True
        state["queue"] = []
        state["decision"] = (
            "Complete Kd sign diagnostic; retain the best of negative, zero, and positive evidence."
        )
        return
    elif stage == "wheel_position_coarse":
        entries = _results(state, ("wheel_position_baseline", "wheel_position_coarse"))
        _set_queue(
            state,
            "wheel_position_confirm",
            _confirmation_queue(entries),
            "Confirm the top two wheel-position candidates with three trials each.",
        )
    elif stage == "wheel_position_confirm":
        best = _confirmed_winner(
            state,
            ("wheel_position_baseline", "wheel_position_coarse"),
            "wheel_position_confirm",
            "Kx",
        )
        if best is None:
            return
        state["selected"]["kx"] = best["candidate"]["kx"]
        state["blocked"] = True
        state["queue"] = []
        state["decision"] = "Lock wheel-position feedback; reassess survival before more terms."
        return
    elif stage == "wheel_speed_coarse":
        entries = _results(state, ("wheel_speed_baseline", "wheel_speed_coarse"))
        _set_queue(
            state,
            "wheel_speed_confirm",
            _confirmation_queue(entries),
            "Confirm the top two wheel-speed damping candidates with three trials each.",
        )
    elif stage == "wheel_speed_confirm":
        best = _confirmed_winner(
            state,
            ("wheel_speed_baseline", "wheel_speed_coarse"),
            "wheel_speed_confirm",
            "Kw",
        )
        if best is None:
            return
        selected = state["selected"]
        selected["kw"] = best["candidate"]["kw"]
        state["blocked"] = True
        state["queue"] = []
        state["decision"] = "Lock wheel-speed damping; reassess survival before adding more terms."
        return
    elif stage == "early_target_coarse":
        entries = _results(state, ("early_target_coarse",))
        _set_queue(
            state,
            "early_target_confirm",
            _confirmation_queue(entries),
            "Confirm the top two early target candidates with three trials each.",
        )
    elif stage == "early_target_confirm":
        best = _confirmed_winner(
            state, ("early_target_coarse",), "early_target_confirm", "early target"
        )
        if best is None:
            return
        selected = state["selected"]
        target = best["candidate"]["target_angle_deg"]
        selected["target_angle_deg"] = target
        selected.pop("kd", None)
        kp = selected["kp"]
        _set_queue(
            state,
            "pd_coarse",
            [candidate("PD", kp, 0, kd, target) for kd in KD_COARSE if kd != 0],
            "Lock the early target, then retune filtered Kd against its P-only baseline.",
        )
        confirmed = _confirmed_entries(
            state, ("early_target_coarse",), "early_target_confirm"
        )
        _seed_pd_baseline(state, confirmed, kp, target)
    elif stage == "p_coarse":
        coarse = _results(state, ("p_coarse",))
        values = _refinement_values(coarse, "kp")
        _set_queue(
            state,
            "p_refine",
            [candidate("P", kp, 0, 0, target) for kp in values],
            "Refine Kp between the best coarse value and its neighbor(s).",
        )
    elif stage == "p_refine":
        entries = _results(state, ("p_coarse", "p_refine"))
        _set_queue(
            state,
            "p_confirm",
            _confirmation_queue(entries),
            "Confirm the top two Kp candidates with three trials each; compare medians.",
        )
    elif stage == "p_confirm":
        p_entries = _confirmed_entries(state, ("p_coarse", "p_refine"), "p_confirm")
        best = _confirmed_winner(state, ("p_coarse", "p_refine"), "p_confirm", "Kp")
        if best is None:
            return
        kp = best["candidate"]["kp"]
        state["selected"]["kp"] = kp
        _seed_pd_baseline(state, p_entries, kp, target)
        _set_queue(
            state,
            "pd_coarse",
            [candidate("PD", kp, 0, kd, target) for kd in KD_COARSE if kd != 0],
            "Freeze Kp and search measured-gyro Kd with Ki=0.",
        )
    elif stage == "pd_coarse":
        coarse = _results(state, ("pd_baseline", "pd_coarse"))
        kp = state["selected"]["kp"]
        values = _refinement_values(coarse, "kd")
        _set_queue(
            state,
            "pd_refine",
            [candidate("PD", kp, 0, kd, target) for kd in values],
            "Refine Kd around the best coarse PD result.",
        )
    elif stage == "pd_refine":
        entries = _results(state, ("pd_baseline", "pd_coarse", "pd_refine"))
        _set_queue(
            state,
            "pd_confirm",
            _confirmation_queue(entries),
            "Confirm the top two Kd candidates with three trials each; compare medians.",
        )
    elif stage == "pd_confirm":
        best = _confirmed_winner(
            state, ("pd_baseline", "pd_coarse", "pd_refine"), "pd_confirm", "Kd"
        )
        if best is None:
            return
        kp = state["selected"]["kp"]
        kd = best["candidate"]["kd"]
        state["selected"]["kd"] = kd
        confirmed = _confirmed_entries(
            state, ("pd_baseline", "pd_coarse", "pd_refine"), "pd_confirm"
        )
        winner_key = _candidate_key(best["candidate"])
        median_survival = median(
            float(entry["metrics"]["survival_s"])
            for entry in confirmed
            if _candidate_key(entry["candidate"]) == winner_key
        )
        if median_survival < MIN_SURVIVAL_FOR_TARGET_S:
            state["blocked"] = True
            state["queue"] = []
            state["decision"] = (
                f"Stop before target tuning: confirmed Kd={kd} has median survival "
                f"{median_survival:.3f}s, below the {MIN_SURVIVAL_FOR_TARGET_S:.3f}s gate."
            )
            return
        _set_queue(
            state,
            "target_coarse",
            [candidate("PD", kp, 0, kd, target + offset) for offset in TARGET_OFFSETS],
            "Freeze PD and search small target-angle offsets.",
        )
    elif stage == "target_coarse":
        coarse = _results(state, ("target_coarse",))
        kp, kd = state["selected"]["kp"], state["selected"]["kd"]
        values = _refinement_values(coarse, "target_angle_deg")
        _set_queue(
            state,
            "target_refine",
            [candidate("PD", kp, 0, kd, value) for value in values],
            "Refine target angle around the best coarse offset.",
        )
    elif stage == "target_refine":
        entries = _results(state, ("target_coarse", "target_refine"))
        _set_queue(
            state,
            "target_confirm",
            _confirmation_queue(entries),
            "Confirm the top two target angles with three trials each; compare medians.",
        )
    elif stage == "target_confirm":
        best = _confirmed_winner(
            state, ("target_coarse", "target_refine"), "target_confirm", "target angle"
        )
        if best is None:
            return
        target_entries = _confirmed_entries(
            state, ("target_coarse", "target_refine"), "target_confirm"
        )
        selected = state["selected"]
        selected["target_angle_deg"] = best["candidate"]["target_angle_deg"]
        winner_key = _candidate_key(best["candidate"])
        winner_entries = [
            entry for entry in target_entries
            if _candidate_key(entry["candidate"]) == winner_key
        ]
        bias = abs(median(
            float(entry["metrics"]["mean_angle_error_deg"])
            for entry in winner_entries
        ))
        if bias <= BIAS_THRESHOLD_DEG:
            selected["ki"] = 0.0
            state["complete"] = True
            state["stage"] = "complete"
            state["queue"] = []
            state["decision"] = (
                f"Skip Ki: residual mean bias {bias:.3f} deg is within "
                f"the {BIAS_THRESHOLD_DEG:.3f} deg threshold."
            )
            state["recommendation"] = candidate(
                "PD", selected["kp"], 0, selected["kd"], selected["target_angle_deg"]
            )
        else:
            _set_queue(
                state,
                "i_optional",
                [
                    candidate(
                        "PID",
                        selected["kp"],
                        ki,
                        selected["kd"],
                        selected["target_angle_deg"],
                    )
                    for ki in KI_CANDIDATES
                ],
                f"Residual mean bias is {bias:.3f} deg; test small anti-windup Ki values.",
            )
    elif stage == "i_optional":
        entries = _results(state, ("i_optional",))
        _set_queue(
            state,
            "i_confirm",
            _confirmation_queue(entries),
            "Confirm the top two Ki candidates with three trials each; compare medians.",
        )
    elif stage == "i_confirm":
        best = _confirmed_winner(state, ("i_optional",), "i_confirm", "Ki")
        if best is None:
            return
        selected = state["selected"]
        selected["ki"] = best["candidate"]["ki"]
        state["complete"] = True
        state["stage"] = "complete"
        state["queue"] = []
        state["decision"] = "Select the best bounded anti-windup Ki candidate."
        state["recommendation"] = best["candidate"]
    else:
        raise ValueError(f"unknown search stage: {stage}")

    # An edge-stage winner can have no neighbor on a degenerate queue.
    if not state["complete"] and not state.get("blocked") and not state["queue"]:
        _advance_stage(state)


def migrate_search_state(state: dict, config: dict) -> dict:
    """Preserve v2 evidence while inserting the new Kp confirmation gate."""
    if state.get("schema_version") == SEARCH_SCHEMA_VERSION:
        return state
    if state.get("schema_version") == 19:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        if not any(
            entry["stage"] == "pd_sign_test" and entry["candidate"]["kd"] == 1.0
            for entry in migrated.get("history", [])
        ):
            session = "results/search_2026-08-12_104400/trial_001"
            metrics_path = __import__("pathlib").Path(session) / "metrics.json"
            if metrics_path.exists():
                import json
                migrated["history"].append({
                    "stage": "pd_sign_test",
                    "candidate": candidate("PD", 160, 0, 1.0, migrated["base_target_angle_deg"], 0, 0),
                    "metrics": json.loads(metrics_path.read_text()),
                    "session": session,
                })
        migrated["queue"] = []
        _advance_stage(migrated)
        return migrated
    if state.get("schema_version") == 18:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        target = migrated["base_target_angle_deg"]
        baseline_entries = [
            entry for entry in migrated.get("history", [])
            if entry["stage"] == "p_coarse"
            and entry["candidate"]["kp"] == 160.0
        ]
        migrated["history"] = []
        for entry in baseline_entries:
            migrated["history"].append({
                "stage": "pd_sign_baseline",
                "candidate": candidate("PD", 160, 0, 0, target, 0, 0),
                "metrics": dict(entry["metrics"]),
                "session": entry["session"],
            })
        migrated["selected"] = {"kp": 160.0, "target_angle_deg": target}
        migrated["blocked"] = False
        migrated["complete"] = False
        _set_queue(
            migrated,
            "pd_sign_test",
            [
                candidate("PD", 160, 0, -1.0, target, 0, 0),
                candidate("PD", 160, 0, 1.0, target, 0, 0),
            ],
            "Diagnose filtered gyro-rate sign with equal negative and positive Kd.",
        )
        return migrated
    if state.get("schema_version") == 17:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        target = migrated["base_target_angle_deg"]
        baseline_entries = [
            entry for entry in migrated.get("history", [])
            if entry["stage"] == "wheel_position_baseline"
        ]
        migrated["history"] = []
        for entry in baseline_entries:
            migrated["history"].append({
                "stage": "p_coarse",
                "candidate": candidate("P", 160, 0, 0, target, 0, 0),
                "metrics": dict(entry["metrics"]),
                "session": entry["session"],
            })
        migrated["selected"] = {}
        migrated["blocked"] = False
        migrated["complete"] = False
        _set_queue(
            migrated,
            "p_coarse",
            [candidate("P", kp, 0, 0, target, 0, 0) for kp in P_COARSE if kp != 160],
            "Extend regulated-speed P search above the prior upper boundary.",
        )
        return migrated
    if state.get("schema_version") == 16:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        selected = migrated.get("selected", {})
        kp = selected.get("kp", 160.0)
        target = selected.get("target_angle_deg", migrated["base_target_angle_deg"])
        migrated["selected"] = {
            "kp": kp, "kd": 0.0, "kw": 0.0, "target_angle_deg": target
        }
        migrated["blocked"] = False
        migrated["complete"] = False
        baseline_entries = [
            entry for entry in migrated.get("history", [])
            if entry["stage"] == "wheel_speed_baseline"
        ]
        for entry in baseline_entries:
            migrated["history"].append({
                "stage": "wheel_position_baseline",
                "candidate": candidate("P", kp, 0, 0, target, 0, 0),
                "metrics": dict(entry["metrics"]),
                "session": entry["session"],
            })
        _set_queue(
            migrated,
            "wheel_position_coarse",
            [candidate("P", kp, 0, 0, target, 0, kx) for kx in KX_COARSE if kx != 0],
            "Tune gentle average wheel-position return with angle gains fixed.",
        )
        return migrated
    if state.get("schema_version") == 15:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        kp = migrated.get("selected", {}).get("kp", 160.0)
        target = migrated.get("selected", {}).get(
            "target_angle_deg", migrated["base_target_angle_deg"]
        )
        migrated["selected"] = {"kp": kp, "kd": 0.0, "target_angle_deg": target}
        migrated["blocked"] = False
        migrated["complete"] = False
        p_entries = [
            entry for entry in migrated.get("history", [])
            if entry["stage"] in ("p_coarse", "p_refine", "p_confirm")
            and entry["candidate"]["kp"] == kp
            and entry["candidate"]["target_angle_deg"] == target
        ]
        for entry in p_entries:
            migrated["history"].append({
                "stage": "wheel_speed_baseline",
                "candidate": candidate("P", kp, 0, 0, target, 0),
                "metrics": dict(entry["metrics"]),
                "session": entry["session"],
            })
        _set_queue(
            migrated,
            "wheel_speed_coarse",
            [candidate("P", kp, 0, 0, target, kw) for kw in KW_COARSE if kw != 0],
            "Tune compact average wheel-speed damping with the angle controller fixed.",
        )
        return migrated
    if state.get("schema_version") == 14:
        migrated = deepcopy(state)
        migrated["schema_version"] = 15
        kp = migrated.get("selected", {}).get("kp")
        if kp is None:
            return new_search_state(config)
        base_target = migrated["base_target_angle_deg"]
        migrated["selected"] = {
            "kp": kp,
            "kd": 0.0,
            "target_angle_deg": base_target,
        }
        migrated["queue"] = []
        migrated["blocked"] = True
        migrated["complete"] = False
        migrated["stage"] = "early_target_confirm"
        migrated["decision"] = (
            "Keep the confirmed target baseline at 87.14 deg: neither early "
            "target candidate beat its median survival. Stop before repeating Kd."
        )
        return migrate_search_state(migrated, config)
    if state.get("schema_version") == 13:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        kp = migrated.get("selected", {}).get("kp")
        if kp is None:
            return new_search_state(config)
        base_target = migrated["base_target_angle_deg"]
        migrated["blocked"] = False
        migrated["complete"] = False
        _set_queue(
            migrated,
            "early_target_confirm",
            [
                candidate("P", kp, 0, 0, base_target + offset)
                for offset in (0.2, 0.2, 0.4, 0.4)
            ],
            "Confirm upward target candidates selected by survival, RMS error, and bias.",
        )
        return migrated
    if state.get("schema_version") == 12:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        kp = migrated.get("selected", {}).get("kp")
        if kp is None:
            return new_search_state(config)
        base_target = migrated["base_target_angle_deg"]
        migrated["blocked"] = False
        migrated["complete"] = False
        _set_queue(
            migrated,
            "early_target_coarse",
            [
                candidate("P", kp, 0, 0, base_target + 0.2),
                candidate("P", kp, 0, 0, base_target + 0.4),
            ],
            "Complete the early target diagnostic on the previously missing upward side.",
        )
        return migrated
    if state.get("schema_version") == 11:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        kp = migrated.get("selected", {}).get("kp")
        if kp is None:
            return new_search_state(config)
        for entry in migrated.get("history", []):
            if entry["stage"].startswith(("pd_", "target_", "early_target_")):
                entry["stage"] = "pre_early_target"
        base_target = migrated["base_target_angle_deg"]
        migrated["selected"] = {"kp": kp, "kd": 0.0}
        migrated["blocked"] = False
        migrated["complete"] = False
        _set_queue(
            migrated,
            "early_target_coarse",
            [candidate("P", kp, 0, 0, base_target + offset) for offset in EARLY_TARGET_OFFSETS],
            "Diagnose persistent backward bias before further Kd tuning.",
        )
        return migrated
    # Regulated motor speed changes controller output from percent duty to
    # degrees/second. All earlier gain evidence is dimensionally incompatible.
    if state.get("schema_version") == 10:
        return new_search_state(config)
    if state.get("schema_version") == 9:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        kp = migrated.get("selected", {}).get("kp")
        if kp is None:
            return new_search_state(config)
        for entry in migrated.get("history", []):
            if entry["stage"].startswith("pd_"):
                entry["stage"] = "pd_prefilter"
        migrated["selected"] = {"kp": kp}
        migrated["blocked"] = False
        migrated["complete"] = False
        target = migrated["base_target_angle_deg"]
        p_entries = _confirmed_entries(
            migrated, ("p_coarse", "p_refine"), "p_confirm"
        )
        _seed_pd_baseline(migrated, p_entries, kp, target)
        _set_queue(
            migrated,
            "pd_coarse",
            [candidate("PD", kp, 0, kd, target) for kd in KD_COARSE if kd != 0],
            "Preserve confirmed Kp and retune low Kd using the filtered gyro signal.",
        )
        return migrated
    # Controller schema 9 changes the physical motor-output mapping. Gain
    # evidence recorded under schemas 6--8 is not comparable and must not be
    # carried into the new local Kp search.
    if state.get("schema_version") in (6, 7, 8):
        return new_search_state(config)
    if state.get("schema_version") == 7:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        kp = migrated.get("selected", {}).get("kp")
        if kp is None:
            return new_search_state(config)
        p_entries = _confirmed_entries(
            migrated, ("p_coarse", "p_refine"), "p_confirm"
        )
        _seed_pd_baseline(migrated, p_entries, kp, migrated["base_target_angle_deg"])
        entries = _results(
            migrated, ("pd_baseline", "pd_coarse", "pd_refine", "pd_confirm")
        )
        _set_queue(
            migrated,
            "pd_confirm",
            _confirmation_queue(entries),
            "Compare low Kd candidates against the three-trial confirmed P-only baseline.",
        )
        if not migrated["queue"]:
            _advance_stage(migrated)
        return migrated

    if state.get("schema_version") == 6:
        migrated = deepcopy(state)
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        kp = migrated.get("selected", {}).get("kp")
        if kp is None:
            return new_search_state(config)
        for entry in migrated.get("history", []):
            if entry["stage"].startswith("pd_"):
                entry["stage"] = "pd_pre_low_range"
        target = migrated["base_target_angle_deg"]
        migrated["blocked"] = False
        migrated["complete"] = False
        _set_queue(
            migrated,
            "pd_coarse",
            [candidate("PD", kp, 0, kd, target) for kd in KD_COARSE],
            "Retune Kd over a lower range including Kd=0 baseline; require D to beat P-only.",
        )
        return migrated

    if state.get("schema_version") not in (2, 3):
        return new_search_state(config)

    migrated = deepcopy(state)
    if state.get("schema_version") == 3:
        migrated["schema_version"] = SEARCH_SCHEMA_VERSION
        p_confirmed = _confirmed_entries(
            migrated, ("p_coarse", "p_refine"), "p_confirm"
        )
        if not p_confirmed:
            return new_search_state(config)
        kp = _best(p_confirmed)["candidate"]["kp"]
        if migrated.get("selected", {}).get("kp") == kp:
            return migrated

        for entry in migrated.get("history", []):
            if entry["stage"].startswith(("pd_", "target_", "i_")):
                entry["stage"] = "post_p_preconfirmation"
        migrated["selected"] = {"kp": kp}
        target = migrated["base_target_angle_deg"]
        _set_queue(
            migrated,
            "pd_coarse",
            [candidate("PD", kp, 0, kd, target) for kd in KD_COARSE],
            "Repaired confirmation eligibility; lock confirmed Kp and restart Kd search.",
        )
        return migrated

    migrated["schema_version"] = SEARCH_SCHEMA_VERSION
    migrated["complete"] = False
    migrated.pop("recommendation", None)
    migrated["selected"] = {}
    for entry in migrated.get("history", []):
        if entry["stage"].startswith("pd_"):
            entry["stage"] = "pd_preconfirmation"
    p_entries = _results(migrated, ("p_coarse", "p_refine"))
    if not p_entries:
        return new_search_state(config)
    _set_queue(
        migrated,
        "p_confirm",
        _confirmation_queue(p_entries),
        "Methodology updated: confirm the top two Kp candidates before locking Kp.",
    )
    return migrated


def record_result(state: dict, metrics: dict, session: str) -> None:
    if state["complete"] or not state["queue"]:
        raise ValueError("search has no pending candidate")
    tested = state["queue"].pop(0)
    state["history"].append(
        {
            "stage": state["stage"],
            "candidate": tested,
            "metrics": metrics,
            "session": session,
        }
    )
    if not state["queue"]:
        _advance_stage(state)
