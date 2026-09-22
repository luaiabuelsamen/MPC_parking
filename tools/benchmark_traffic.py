#!/usr/bin/env python3
"""Run closed-loop traffic variations against one optimized Buck2 binary."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--buck2", default="buck2")
    parser.add_argument("--output", type=Path, default=Path("artifacts/traffic_benchmark.json"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    build = subprocess.run([args.buck2, "build", "//:traffic_encounter", "--show-output"],
                           cwd=root, text=True, capture_output=True)
    if build.returncode:
        raise SystemExit(build.stderr)
    binary = root / build.stdout.strip().splitlines()[-1].split(maxsplit=1)[1]
    cases = [
        ("baseline", []),
        ("early_oncoming", ["--oncoming-x", "-5", "--oncoming-speed", "2"]),
        ("late_oncoming", ["--oncoming-x", "22", "--oncoming-speed", "1.2"]),
        ("fast_follower", ["--passing-speed", "2"]),
        ("slow_follower", ["--passing-speed", "1.2"]),
        ("dropped_results", ["--fault-step", "140"]),
    ]
    report = {"schema": 1, "cases": []}
    with tempfile.TemporaryDirectory(prefix="mpcpark-traffic-") as tmp:
        for name, options in cases:
            path = Path(tmp) / f"{name}.json"
            run = subprocess.run([str(binary), "--output", str(path), *options],
                                 cwd=root, text=True, capture_output=True)
            if not path.exists():
                report["cases"].append({"name": name, "passed": False, "error": run.stderr})
                print(name, "FAILED:", run.stderr, flush=True)
                continue
            data = json.loads(path.read_text())
            summary = data["summary"]
            passed = (run.returncode == 0 and summary["interaction_ok"] and
                      summary["deadline_misses"] == 0 and
                      (name != "dropped_results" or summary["fallback_steps"] >= 3))
            report["cases"].append({"name": name, "options": options,
                                    "passed": passed, **summary})
            print(name, "PASS" if passed else "FAIL", run.stdout.strip(), flush=True)
    report["passed"] = sum(c["passed"] for c in report["cases"])
    report["total"] = len(cases)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"{report['passed']}/{len(cases)} passed; {args.output}")
    raise SystemExit(0 if report["passed"] == len(cases) else 1)


if __name__ == "__main__":
    main()
