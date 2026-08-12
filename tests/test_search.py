from host.search import (
    CONFIRMATION_TRIALS,
    KD_COARSE,
    KW_COARSE,
    KX_COARSE,
    _best,
    current_candidate,
    migrate_search_state,
    new_search_state,
    record_result,
)


def config():
    return {"controller": {"target_angle_deg": 87.14}}


def metrics(score, mean_error=0.1, recovered=True, survival=5.0):
    return {
        "score": score,
        "survival_s": survival,
        "rms_angle_error_deg": 1.0,
        "mean_angle_error_deg": mean_error,
        "recovered_toward_upright": recovered,
    }


def run_methodology(residual_bias, recovered=True):
    state = new_search_state(config())
    count = 0
    while not state["complete"] and not state.get("blocked"):
        trial = current_candidate(state)
        assert trial is not None
        stage = state["stage"]
        if stage.startswith("p_"):
            score = 500 - abs(trial["kp"] - 200.0)
        elif stage.startswith("pd_"):
            score = 510 - abs(trial["kd"] - 1.0) * 10
        elif stage.startswith("target_"):
            score = 500 - abs(trial["target_angle_deg"] - 87.34) * 100
        else:
            score = 500 - abs(trial["ki"] - 0.02) * 100
        record_result(state, metrics(score, residual_bias, recovered), f"trial-{count}")
        count += 1
        assert count < 50
    return state


def test_four_stage_search_skips_unneeded_integral():
    state = run_methodology(residual_bias=0.2)

    assert state["recommendation"]["mode"] == "PD"
    assert state["recommendation"]["kp"] == 200.0
    assert state["recommendation"]["kd"] == 1.0
    assert state["recommendation"]["ki"] == 0.0
    assert "Skip Ki" in state["decision"]
    assert_confirmed(state, "kp", ("p_coarse", "p_refine", "p_confirm"))
    assert_confirmed(state, "kd", ("pd_coarse", "pd_refine", "pd_confirm"))
    assert_confirmed(
        state,
        "target_angle_deg",
        ("target_coarse", "target_refine", "target_confirm"),
    )


def test_four_stage_search_adds_small_integral_only_for_bias():
    state = run_methodology(residual_bias=0.8)

    assert state["recommendation"]["mode"] == "PID"
    assert state["recommendation"]["ki"] == 0.02
    assert any(entry["stage"] == "i_optional" for entry in state["history"])
    assert_confirmed(state, "ki", ("i_optional", "i_confirm"))


def test_search_stops_before_locking_a_parameter_without_recovery():
    state = run_methodology(residual_bias=0.2, recovered=False)

    assert state["blocked"] is True
    assert state["stage"] == "p_confirm"
    assert state["selected"] == {}
    assert "recovered toward upright in only 0/3" in state["decision"]


def test_search_stops_before_target_tuning_when_pd_survival_is_too_short():
    state = new_search_state(config())
    while state["stage"] != "pd_confirm":
        trial = current_candidate(state)
        score = (
                500 - abs(trial["kp"] - 200.0)
            if state["stage"].startswith("p_")
            else 500 - trial["kd"] * 100
        )
        record_result(state, metrics(score, survival=1.5), "short")
    while state["queue"]:
        trial = current_candidate(state)
        record_result(
            state,
            metrics(500 - trial["kd"] * 100, survival=1.5),
            "short-confirm",
        )

    assert state["blocked"] is True
    assert state["selected"]["kd"] == 0.0
    assert "below the 3.000s gate" in state["decision"]


def assert_confirmed(state, field, stages):
    selected = state["selected"][field]
    matching = [
        entry for entry in state["history"]
        if entry["stage"] in stages and entry["candidate"][field] == selected
    ]
    assert len(matching) == CONFIRMATION_TRIALS


def test_selection_uses_median_instead_of_single_outlier():
    entries = []
    for kp, scores in ((8.0, (100, 100, 1000)), (10.0, (200, 200, 200))):
        for score in scores:
            entries.append({
                "candidate": {"kp": kp},
                "metrics": metrics(score),
                "stage": "p_confirm",
                "session": "test",
            })
    assert _best(entries)["candidate"]["kp"] == 10.0


def test_v2_migration_preserves_evidence_and_returns_to_kp_confirmation():
    state = new_search_state(config())
    state["schema_version"] = 2
    state["stage"] = "pd_coarse"
    state["selected"] = {"kp": 10.0}
    state["queue"] = []
    state["history"] = [
        {
            "stage": "p_coarse",
            "candidate": {"mode": "P", "kp": kp, "ki": 0.0, "kd": 0.0,
                          "target_angle_deg": 87.14},
            "metrics": metrics(score),
            "session": f"p-{kp}",
        }
        for kp, score in ((8.0, 100), (10.0, 200), (12.0, 80), (15.0, 150))
    ]
    state["history"].append({
        "stage": "pd_coarse",
        "candidate": {"mode": "PD", "kp": 10.0, "ki": 0.0, "kd": 0.1,
                      "target_angle_deg": 87.14},
        "metrics": metrics(90),
        "session": "pd-old",
    })

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "p_confirm"
    assert migrated["selected"] == {}
    assert len(migrated["queue"]) == 4
    assert migrated["history"][-1]["stage"] == "pd_preconfirmation"


def test_v3_migration_excludes_unconfirmed_candidate_and_repairs_kd_stage():
    state = new_search_state(config())
    state["schema_version"] = 3
    state["stage"] = "pd_confirm"
    state["selected"] = {"kp": 9.0}
    state["queue"] = []
    state["history"] = []
    candidates = {
        9.0: (("p_refine", 211),),
        10.0: (("p_coarse", 291), ("p_confirm", -195), ("p_confirm", -214)),
        15.0: (("p_coarse", 220), ("p_confirm", 72), ("p_confirm", 142)),
    }
    for kp, evidence in candidates.items():
        for stage, score in evidence:
            state["history"].append({
                "stage": stage,
                "candidate": {"mode": "P", "kp": kp, "ki": 0.0, "kd": 0.0,
                              "target_angle_deg": 87.14},
                "metrics": metrics(score),
                "session": f"{kp}-{score}",
            })
    state["history"].append({
        "stage": "pd_coarse",
        "candidate": {"mode": "PD", "kp": 9.0, "ki": 0.0, "kd": 0.1,
                      "target_angle_deg": 87.14},
        "metrics": metrics(130),
        "session": "invalid-pd",
    })

    migrated = migrate_search_state(state, config())

    assert migrated["selected"] == {"kp": 15.0}
    assert migrated["stage"] == "pd_coarse"
    assert [candidate["kd"] for candidate in migrated["queue"]] == list(KD_COARSE)
    assert migrated["history"][-1]["stage"] == "post_p_preconfirmation"


def test_kd_search_includes_p_only_baseline_and_small_values():
    assert KD_COARSE == (0.0, 0.5, 1.0, 2.0)


def test_motor_mapping_revision_restarts_gain_search():
    state = new_search_state(config())
    state["schema_version"] = 7
    state["history"] = [{"old": "evidence"}]
    state["stage"] = "pd_confirm"

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "p_coarse"
    assert migrated["history"] == []
    assert [item["kp"] for item in migrated["queue"]] == [160.0, 200.0, 240.0, 320.0]


def test_regulated_speed_revision_restarts_gain_search():
    state = new_search_state(config())
    state["schema_version"] = 10
    state["history"] = [{"old": "duty evidence"}]

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "p_coarse"
    assert migrated["history"] == []
    assert [item["kp"] for item in migrated["queue"]] == [160.0, 200.0, 240.0, 320.0]


def test_bias_diagnostic_preserves_kp_and_replaces_kd_refinement():
    state = new_search_state(config())
    state["schema_version"] = 11
    state["stage"] = "pd_refine"
    state["selected"] = {"kp": 160.0}
    state["queue"] = [{"old": "kd candidate"}]

    migrated = migrate_search_state(state, config())

    assert migrated["selected"] == {"kp": 160.0, "kd": 0.0}
    assert migrated["stage"] == "early_target_coarse"
    assert [item["target_angle_deg"] for item in migrated["queue"]] == [
        86.74, 86.94, 87.14
    ]
    assert all(item["kd"] == 0.0 for item in migrated["queue"])


def test_one_sided_target_diagnostic_is_extended_upward():
    state = new_search_state(config())
    state["schema_version"] = 12
    state["selected"] = {"kp": 160.0, "kd": 0.0}
    state["stage"] = "early_target_confirm"

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "early_target_coarse"
    assert [item["target_angle_deg"] for item in migrated["queue"]] == [87.34, 87.54]


def test_target_confirmation_keeps_bias_reducing_upward_candidates():
    state = new_search_state(config())
    state["schema_version"] = 13
    state["selected"] = {"kp": 160.0, "kd": 0.0}
    state["stage"] = "early_target_confirm"

    migrated = migrate_search_state(state, config())

    assert [item["target_angle_deg"] for item in migrated["queue"]] == [
        87.34, 87.34, 87.54, 87.54
    ]


def test_target_diagnostic_that_loses_to_baseline_stops_without_repeating_kd():
    state = new_search_state(config())
    state["schema_version"] = 14
    state["selected"] = {"kp": 160.0, "target_angle_deg": 87.54}
    state["stage"] = "pd_coarse"
    state["queue"] = [{"would": "repeat kd"}]

    migrated = migrate_search_state(state, config())

    assert migrated["selected"] == {
        "kp": 160.0, "kd": 0.0, "target_angle_deg": 87.14
    }
    assert migrated["blocked"] is False
    assert migrated["stage"] == "wheel_speed_coarse"
    assert [item["kw"] for item in migrated["queue"]] == [0.1, 0.2, 0.4]


def test_wheel_speed_revision_preserves_confirmed_p_baseline():
    state = new_search_state(config())
    state["schema_version"] = 15
    state["selected"] = {"kp": 160.0, "kd": 0.0, "target_angle_deg": 87.14}
    state["history"] = []
    for score in (100, 110, 120):
        state["history"].append({
            "stage": "p_confirm",
            "candidate": {"mode": "P", "kp": 160.0, "ki": 0.0, "kd": 0.0,
                          "target_angle_deg": 87.14},
            "metrics": metrics(score),
            "session": str(score),
        })

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "wheel_speed_coarse"
    assert [item["kw"] for item in migrated["queue"]] == [0.1, 0.2, 0.4]
    assert len([x for x in migrated["history"] if x["stage"] == "wheel_speed_baseline"]) == 3
    assert KW_COARSE == (0.0, 0.1, 0.2, 0.4)


def test_wheel_position_revision_preserves_zero_feedback_baseline():
    state = new_search_state(config())
    state["schema_version"] = 16
    state["selected"] = {
        "kp": 160.0, "kd": 0.0, "kw": 0.0, "target_angle_deg": 87.14
    }
    state["history"] = []
    for score in (100, 110, 120):
        state["history"].append({
            "stage": "wheel_speed_baseline",
            "candidate": {"mode": "P", "kp": 160.0, "ki": 0.0, "kd": 0.0,
                          "kw": 0.0, "kx": 0.0, "target_angle_deg": 87.14},
            "metrics": metrics(score),
            "session": str(score),
        })

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "wheel_position_coarse"
    assert [item["kx"] for item in migrated["queue"]] == [0.1, 0.25, 0.5]
    assert len([x for x in migrated["history"] if x["stage"] == "wheel_position_baseline"]) == 3
    assert KX_COARSE == (0.0, 0.1, 0.25, 0.5)


def test_upper_p_revision_reuses_160_baseline_and_queues_higher_gains():
    state = new_search_state(config())
    state["schema_version"] = 17
    state["history"] = []
    for score in (100, 110, 120):
        state["history"].append({
            "stage": "wheel_position_baseline",
            "candidate": {"mode": "P", "kp": 160.0, "ki": 0.0, "kd": 0.0,
                          "kw": 0.0, "kx": 0.0, "target_angle_deg": 87.14},
            "metrics": metrics(score),
            "session": str(score),
        })

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "p_coarse"
    assert [item["kp"] for item in migrated["queue"]] == [200.0, 240.0, 320.0]
    assert len(migrated["history"]) == 3
    assert all(item["candidate"]["kp"] == 160.0 for item in migrated["history"])


def test_p_upper_search_transitions_to_bounded_kd_sign_test():
    state = new_search_state(config())
    state["schema_version"] = 18
    state["history"] = []
    for score in (100, 110, 120):
        state["history"].append({
            "stage": "p_coarse",
            "candidate": {"mode": "P", "kp": 160.0, "ki": 0.0, "kd": 0.0,
                          "kw": 0.0, "kx": 0.0, "target_angle_deg": 87.14},
            "metrics": metrics(score),
            "session": str(score),
        })

    migrated = migrate_search_state(state, config())

    assert migrated["stage"] == "pd_sign_test"
    assert [item["kd"] for item in migrated["queue"]] == [-1.0, 1.0]
    assert len([x for x in migrated["history"] if x["stage"] == "pd_sign_baseline"]) == 3


def test_kd_sign_stage_completes_without_unknown_stage_error():
    state = new_search_state(config())
    state["stage"] = "pd_sign_test"
    state["queue"] = [
        {"mode": "PD", "kp": 160.0, "ki": 0.0, "kd": -1.0, "kw": 0.0,
         "kx": 0.0, "target_angle_deg": 87.14},
        {"mode": "PD", "kp": 160.0, "ki": 0.0, "kd": 1.0, "kw": 0.0,
         "kx": 0.0, "target_angle_deg": 87.14},
    ]
    state["history"] = [
        {"stage": "pd_sign_baseline", "candidate":
         {"mode": "PD", "kp": 160.0, "ki": 0.0, "kd": 0.0, "kw": 0.0,
          "kx": 0.0, "target_angle_deg": 87.14},
         "metrics": metrics(100), "session": "baseline"}
    ]
    record_result(state, metrics(50), "negative")
    record_result(state, metrics(75), "positive")

    assert state["blocked"] is True
    assert state["selected"]["kd"] == 0.0
    assert state["queue"] == []


def test_gyro_filter_revision_preserves_kp_and_restarts_only_kd():
    state = new_search_state(config())
    state["schema_version"] = 9
    state["selected"] = {"kp": 7.0, "kd": 0.0}
    state["blocked"] = True
    state["stage"] = "pd_confirm"
    state["history"] = []
    for score in (100, 110, 120):
        state["history"].append({
            "stage": "p_confirm",
            "candidate": {"mode": "P", "kp": 7.0, "ki": 0.0, "kd": 0.0,
                          "target_angle_deg": 87.14},
            "metrics": metrics(score),
            "session": str(score),
        })

    migrated = migrate_search_state(state, config())

    assert migrated["selected"] == {"kp": 7.0}
    assert migrated["stage"] == "pd_coarse"
    assert [item["kd"] for item in migrated["queue"]] == [0.5, 1.0, 2.0]
    assert len([x for x in migrated["history"] if x["stage"] == "pd_baseline"]) == 3
