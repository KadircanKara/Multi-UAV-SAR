"""Backfill RunConfig sidecars for seeded missions. Idempotent; run once.

    PYTHONPATH="$(pwd)/backend:$(pwd)" .venv/bin/python backend/scripts/backfill_run_config.py [--force]

Writes Results/Metadata/{scenario}.json for every Results/Objectives/* whose name
maps to a preset model. Skips existing sidecars unless --force. Pkls are untouched.
"""
import glob
import json
import os
import sys

import app.rootpath  # noqa: F401  (repo root on sys.path)
import pandas as pd

from app import settings
from app.run_config import seed_config_for


def main(force: bool) -> None:
    obj_dir = os.path.join(settings.RESULTS_ROOT, "Objectives")
    meta_dir = os.path.join(settings.RESULTS_ROOT, "Metadata")
    os.makedirs(meta_dir, exist_ok=True)

    suffix = "-ObjectiveValues.pkl"
    written = skipped = unmatched = 0
    for path in sorted(glob.glob(os.path.join(obj_dir, f"*{suffix}"))):
        name = os.path.basename(path)[: -len(suffix)]
        dst = os.path.join(meta_dir, f"{name}.json")
        if os.path.isfile(dst) and not force:
            skipped += 1
            continue
        try:
            n_solutions = int(pd.read_pickle(path).shape[0])
        except Exception:
            n_solutions = 0
        cfg = seed_config_for(name, n_solutions)
        if cfg is None:
            unmatched += 1
            continue
        with open(dst, "w") as fh:
            json.dump(cfg, fh, indent=2)
        written += 1

    print(f"backfill: written={written} skipped={skipped} unmatched={unmatched}")


if __name__ == "__main__":
    main("--force" in sys.argv[1:])
