#!/usr/bin/env python3

import argparse
import json
import pathlib
import statistics
import subprocess
import sys
import time


def parse_metrics(stdout: str) -> dict:
    metrics = {}
    for line in stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        try:
            metrics[key] = float(value)
        except ValueError:
            metrics[key] = value
    return metrics


def sample_rss_kb(pid: int) -> int:
    try:
        with open(f"/proc/{pid}/status", "r", encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except FileNotFoundError:
        return 0
    return 0


def run_command(command: list[str], cwd: pathlib.Path) -> dict:
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=True,
        text=True,
        capture_output=True,
    )
    return parse_metrics(completed.stdout)


def summarize_rss_samples(rss_samples: list[int]) -> dict:
    if not rss_samples:
        return {
            "rss_kb_first": 0,
            "rss_kb_min": 0,
            "rss_kb_max": 0,
            "rss_kb_last": 0,
            "rss_kb_samples": 0,
            "rss_kb_series": [],
            "rss_kb_head_median": 0,
            "rss_kb_tail_median": 0,
            "rss_kb_growth_kb": 0,
            "rss_kb_growth_ratio": 0.0,
        }

    stabilized_samples = list(rss_samples)
    stable_threshold = max(rss_samples) * 0.5
    while len(stabilized_samples) > 1 and stabilized_samples[0] < stable_threshold:
        stabilized_samples.pop(0)

    window = max(1, len(stabilized_samples) // 5)
    head_median = int(statistics.median(stabilized_samples[:window]))
    tail_median = int(statistics.median(stabilized_samples[-window:]))
    growth_kb = tail_median - head_median
    growth_ratio = float(tail_median) / float(head_median) if head_median > 0 else 0.0
    return {
        "rss_kb_first": rss_samples[0],
        "rss_kb_min": min(rss_samples),
        "rss_kb_max": max(rss_samples),
        "rss_kb_last": rss_samples[-1],
        "rss_kb_samples": len(rss_samples),
        "rss_kb_series": rss_samples,
        "rss_kb_stable_samples": len(stabilized_samples),
        "rss_kb_head_median": head_median,
        "rss_kb_tail_median": tail_median,
        "rss_kb_growth_kb": growth_kb,
        "rss_kb_growth_ratio": growth_ratio,
    }


def run_soak(command: list[str], cwd: pathlib.Path, duration_seconds: int) -> dict:
    soak_command = [*command, "--soak-seconds", str(duration_seconds)]
    rss_samples = []
    process = subprocess.Popen(
        soak_command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    while process.poll() is None:
        rss_kb = sample_rss_kb(process.pid)
        if rss_kb > 0:
            rss_samples.append(rss_kb)
        time.sleep(1.0)
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        raise RuntimeError(
            f"soak command failed: {' '.join(soak_command)}\nstdout:\n{stdout}\nstderr:\n{stderr}"
        )
    metrics = parse_metrics(stdout)
    result = summarize_rss_samples(rss_samples)
    result["iterations"] = int(metrics.get("iterations", 0))
    result["wall_seconds"] = float(metrics.get("wall_seconds", 0.0))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace-root",
        default=str(pathlib.Path(__file__).resolve().parents[3]),
        help="robot_mujoco workspace root",
    )
    parser.add_argument(
        "--scenario-file",
        default=str(pathlib.Path(__file__).with_name("baseline_scenarios.json")),
    )
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--soak-seconds", type=int, default=60)
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    workspace_root = pathlib.Path(args.workspace_root).resolve()
    scenario_file = pathlib.Path(args.scenario_file).resolve()
    with open(scenario_file, "r", encoding="utf-8") as handle:
        scenarios = json.load(handle)["scenarios"]

    report = {
        "generated_at_epoch_seconds": time.time(),
        "workspace_root": str(workspace_root),
        "repeat": args.repeat,
        "scenarios": [],
    }

    for scenario in scenarios:
        runs = [run_command(scenario["command"], workspace_root) for _ in range(args.repeat)]
        keys = sorted({key for run in runs for key in run.keys()})
        aggregated = {}
        for key in keys:
            values = [run[key] for run in runs if isinstance(run.get(key), (int, float))]
            if values:
                aggregated[f"{key}_mean"] = statistics.fmean(values)
                aggregated[f"{key}_p95"] = max(values) if len(values) == 1 else sorted(values)[
                    max(0, int(len(values) * 0.95) - 1)
                ]
            else:
                aggregated[key] = runs[-1].get(key)
        report["scenarios"].append(
            {
                "name": scenario["name"],
                "command": scenario["command"],
                "runs": runs,
                "aggregated": aggregated,
            }
        )

    soak_command = ["build/robot_mujoco_ros2/perf_read_write_loop"]
    report["soak"] = {
        "command": soak_command,
        "duration_seconds": args.soak_seconds,
        "result": run_soak(soak_command, workspace_root, args.soak_seconds),
    }

    output = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        pathlib.Path(args.output).write_text(output + "\n", encoding="utf-8")
    else:
        print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
