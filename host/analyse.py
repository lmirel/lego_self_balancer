"""Score an existing Phase 4 results session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .scoring import score_csv


def analyse_session(session_dir: Path) -> dict[str, float | int | bool]:
    metadata_path = session_dir / "session.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metrics = score_csv(
        session_dir / "telemetry.csv",
        metadata["events"],
        trial_duration_s=metadata["config"]["safety"]["trial_duration_s"],
        output_limit=metadata["config"]["controller"]["output_limit"],
    )
    (session_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
    )
    return metrics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    metrics = analyse_session(args.session.resolve())
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
