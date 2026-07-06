"""Export an on-disk scenario's results to the Playground JSON schema.

Usage:
    python backend/scripts/export_run_json.py --scenario <name> --out <path>

Run from the repo root with PYTHONPATH="$PWD/backend:$PWD" so `app.*` and the
root modules import. Produces a file uploadable to the web app's Playground.
"""
import argparse
import json
import sys

import app.rootpath  # noqa: F401  (repo root on sys.path)

from app.library_service import resolve_model_key
from app.selector_service import get_selector
from app.playground_export import serialize_run


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", required=True, help="Scenario name (as in Results/)")
    ap.add_argument("--out", required=True, help="Output .json path")
    args = ap.parse_args()

    try:
        sel = get_selector(args.scenario)
        model_key = resolve_model_key(args.scenario)
    except Exception as exc:  # missing scenario / unresolved model
        print(f"error: cannot load scenario {args.scenario!r}: {exc}", file=sys.stderr)
        return 2

    payload = serialize_run(sel.solutions, sel.F, sel.model, {}, model_key=model_key)
    with open(args.out, "w") as fh:
        json.dump(payload, fh)
    print(f"wrote {args.out}: {len(payload['solutions'])} solutions, model {model_key}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
