#!/usr/bin/env python3

import argparse
import glob
import json
import pathlib
import sys


def load_json(path: pathlib.Path) -> dict | None:
    if not path.is_file():
        return None
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def format_delta(value) -> str:
    if not isinstance(value, (int, float)):
        return "-"
    return f"{value:+.2f}%"


def format_value(value) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if value is None:
        return "-"
    return str(value)


def run_label(metrics: dict) -> str:
    github = metrics.get("github", {})
    return str(github.get("github_run_id", "unknown"))


def check_map(metrics: dict) -> dict[str, dict]:
    checks = metrics.get("metrics", {}).get("performance_checks", [])
    return {check["name"]: check for check in checks if check.get("name")}


def scenario_map(metrics: dict) -> dict[str, dict]:
    scenarios = metrics.get("metrics", {}).get("scenario_statuses", [])
    return {scenario["name"]: scenario for scenario in scenarios if scenario.get("name")}


def load_runs(current_path: pathlib.Path, previous_pattern: str) -> list[dict]:
    current = load_json(current_path)
    if current is None:
        raise FileNotFoundError(f"missing current nightly metrics: {current_path}")

    runs = [current]
    previous_paths = sorted(
        (pathlib.Path(path) for path in glob.glob(previous_pattern)),
        key=lambda path: path.name,
        reverse=True,
    )
    for path in previous_paths:
        data = load_json(path)
        if data is not None:
            runs.append(data)
    return runs


def build_run_summaries(runs: list[dict]) -> list[dict]:
    result = []
    for run in runs:
        summary = run.get("metrics", {}).get("summary", {})
        github = run.get("github", {})
        result.append(
            {
                "run_id": github.get("github_run_id"),
                "ref_name": github.get("github_ref_name"),
                "overall_status": summary.get("overall_status"),
                "performance_status": summary.get("performance_status"),
                "performance_warnings": summary.get("performance_warnings", 0),
                "performance_gating_failures": summary.get("performance_gating_failures", 0),
                "tsan_status": summary.get("tsan_status"),
            }
        )
    return result


def build_check_histories(runs: list[dict]) -> list[dict]:
    names = sorted({name for run in runs for name in check_map(run).keys()})
    histories = []
    for name in names:
        observations = []
        for run in runs:
            check = check_map(run).get(name)
            if check is None:
                continue
            observations.append(
                {
                    "run_id": run.get("github", {}).get("github_run_id"),
                    "status": check.get("status"),
                    "semantic_delta_percent": check.get("semantic_delta_percent"),
                    "delta_percent": check.get("delta_percent"),
                    "message": check.get("message"),
                }
            )
        histories.append(
            {
                "name": name,
                "observations": observations,
            }
        )
    return histories


def persistent_warnings(histories: list[dict]) -> list[dict]:
    result = []
    for history in histories:
        observations = history["observations"]
        if len(observations) < 2:
            continue
        if all(obs.get("status") in {"warn", "fail"} for obs in observations[: min(3, len(observations))]):
            result.append(history)
    return result


def window_regressions(histories: list[dict]) -> list[dict]:
    regressions = []
    improvements = []
    for history in histories:
        observations = history["observations"]
        if len(observations) < 2:
            continue
        latest = observations[0].get("semantic_delta_percent")
        oldest = observations[-1].get("semantic_delta_percent")
        if isinstance(latest, (int, float)) and isinstance(oldest, (int, float)):
            change = latest - oldest
            entry = {
                "name": history["name"],
                "window_semantic_delta_change": change,
                "latest_semantic_delta_percent": latest,
                "oldest_semantic_delta_percent": oldest,
                "latest_status": observations[0].get("status"),
            }
            if change < 0.0:
                regressions.append(entry)
            elif change > 0.0:
                improvements.append(entry)
    regressions.sort(key=lambda item: item["window_semantic_delta_change"])
    improvements.sort(key=lambda item: item["window_semantic_delta_change"], reverse=True)
    return regressions, improvements


def scenario_histories(runs: list[dict]) -> list[dict]:
    names = sorted({name for run in runs for name in scenario_map(run).keys()})
    result = []
    for name in names:
        observations = []
        for run in runs:
            scenario = scenario_map(run).get(name)
            if scenario is None:
                continue
            observations.append(
                {
                    "run_id": run.get("github", {}).get("github_run_id"),
                    "status": scenario.get("status"),
                    "warnings": scenario.get("warnings", 0),
                    "errors": scenario.get("errors", 0),
                    "gating_failures": scenario.get("gating_failures", 0),
                }
            )
        result.append({"name": name, "observations": observations})
    return result


def build_report(runs: list[dict]) -> dict:
    run_summaries = build_run_summaries(runs)
    check_histories = build_check_histories(runs)
    scenario_history = scenario_histories(runs)
    persistent = persistent_warnings(check_histories)
    regressions, improvements = window_regressions(check_histories)
    return {
        "status": "history" if len(runs) > 1 else "missing_history",
        "message": f"runs={len(runs)}",
        "runs": run_summaries,
        "scenario_history": scenario_history,
        "check_histories": check_histories,
        "highlights": {
            "persistent_warnings": persistent[:10],
            "largest_window_regressions": regressions[:10],
            "largest_window_improvements": improvements[:10],
        },
    }


def write_markdown(report: dict, output_path: pathlib.Path) -> None:
    lines = [
        "# Nightly History",
        "",
        f"- Status: `{report['status']}`",
        f"- Message: {report['message']}",
    ]

    lines.extend(
        [
            "",
            "## Runs",
            "",
            "| Run | Overall | Perf | Perf Warn | Perf Gate | TSan |",
            "| --- | --- | --- | ---: | ---: | --- |",
        ]
    )
    for run in report.get("runs", []):
        lines.append(
            "| "
            f"{run.get('run_id', '-')} | "
            f"{run.get('overall_status', '-')} | "
            f"{run.get('performance_status', '-')} | "
            f"{format_value(run.get('performance_warnings'))} | "
            f"{format_value(run.get('performance_gating_failures'))} | "
            f"{run.get('tsan_status', '-')} |"
        )

    highlights = report.get("highlights", {})
    lines.extend(["", "## Highlights", ""])
    lines.append("### Persistent Warnings")
    if highlights.get("persistent_warnings"):
        for history in highlights["persistent_warnings"]:
            obs = history["observations"][:3]
            rendered = ", ".join(
                f"{item.get('run_id')}: {item.get('status')}" for item in obs
            )
            lines.append(f"- `{history['name']}`: {rendered}")
    else:
        lines.append("- None")

    lines.append("")
    lines.append("### Largest Window Regressions")
    if highlights.get("largest_window_regressions"):
        for item in highlights["largest_window_regressions"][:10]:
            lines.append(
                f"- `{item['name']}`: {format_delta(item.get('window_semantic_delta_change'))} "
                f"({format_delta(item.get('oldest_semantic_delta_percent'))} -> {format_delta(item.get('latest_semantic_delta_percent'))})"
            )
    else:
        lines.append("- None")

    lines.append("")
    lines.append("### Largest Window Improvements")
    if highlights.get("largest_window_improvements"):
        for item in highlights["largest_window_improvements"][:10]:
            lines.append(
                f"- `{item['name']}`: {format_delta(item.get('window_semantic_delta_change'))} "
                f"({format_delta(item.get('oldest_semantic_delta_percent'))} -> {format_delta(item.get('latest_semantic_delta_percent'))})"
            )
    else:
        lines.append("- None")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current", required=True)
    parser.add_argument("--previous-pattern", required=True)
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--markdown-output", required=True)
    args = parser.parse_args()

    runs = load_runs(pathlib.Path(args.current), args.previous_pattern)
    report = build_report(runs)

    json_output_path = pathlib.Path(args.json_output)
    json_output_path.parent.mkdir(parents=True, exist_ok=True)
    json_output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(report, pathlib.Path(args.markdown_output))

    print(f"nightly_history_status={report['status']} runs={len(runs)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
