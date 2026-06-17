#!/usr/bin/env python3

import argparse
import json
import pathlib
import sys


def load_json(path: pathlib.Path) -> dict | None:
    if not path.is_file():
        return None
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def check_map(metrics: dict) -> dict[str, dict]:
    checks = metrics.get("metrics", {}).get("performance_checks", [])
    return {check["name"]: check for check in checks if check.get("name")}


def scenario_map(metrics: dict) -> dict[str, dict]:
    scenarios = metrics.get("metrics", {}).get("scenario_statuses", [])
    return {scenario["name"]: scenario for scenario in scenarios if scenario.get("name")}


def format_value(value) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if value is None:
        return "-"
    return str(value)


def format_delta(value) -> str:
    if not isinstance(value, (int, float)):
        return "-"
    return f"{value:+.2f}%"


def compare_checks(previous: dict, current: dict) -> list[dict]:
    previous_checks = check_map(previous)
    current_checks = check_map(current)
    names = sorted(set(previous_checks.keys()) | set(current_checks.keys()))
    comparisons = []
    for name in names:
        prev = previous_checks.get(name)
        curr = current_checks.get(name)
        entry = {
            "name": name,
            "previous": prev,
            "current": curr,
            "status_changed": (prev or {}).get("status") != (curr or {}).get("status"),
            "previous_status": (prev or {}).get("status"),
            "current_status": (curr or {}).get("status"),
            "previous_semantic_delta_percent": (prev or {}).get("semantic_delta_percent"),
            "current_semantic_delta_percent": (curr or {}).get("semantic_delta_percent"),
        }
        prev_semantic = entry["previous_semantic_delta_percent"]
        curr_semantic = entry["current_semantic_delta_percent"]
        if isinstance(prev_semantic, (int, float)) and isinstance(curr_semantic, (int, float)):
            entry["semantic_delta_change"] = curr_semantic - prev_semantic
        else:
            entry["semantic_delta_change"] = None
        comparisons.append(entry)
    return comparisons


def scenario_comparisons(previous: dict, current: dict) -> list[dict]:
    previous_scenarios = scenario_map(previous)
    current_scenarios = scenario_map(current)
    names = sorted(set(previous_scenarios.keys()) | set(current_scenarios.keys()))
    result = []
    for name in names:
        prev = previous_scenarios.get(name, {})
        curr = current_scenarios.get(name, {})
        result.append(
            {
                "name": name,
                "previous_status": prev.get("status"),
                "current_status": curr.get("status"),
                "previous_warnings": prev.get("warnings"),
                "current_warnings": curr.get("warnings"),
                "previous_errors": prev.get("errors"),
                "current_errors": curr.get("errors"),
                "previous_gating_failures": prev.get("gating_failures"),
                "current_gating_failures": curr.get("gating_failures"),
            }
        )
    return result


def build_report(previous: dict | None, current: dict) -> dict:
    current_summary = current.get("metrics", {}).get("summary", {})
    if previous is None:
        return {
            "status": "missing_previous",
            "message": "previous nightly_metrics.json is unavailable",
            "previous_github": None,
            "current_github": current.get("github", {}),
            "scenario_comparisons": [],
            "check_comparisons": [],
        }

    previous_summary = previous.get("metrics", {}).get("summary", {})
    checks = compare_checks(previous, current)
    scenarios = scenario_comparisons(previous, current)
    status_changes = [check for check in checks if check["status_changed"]]
    worse_checks = [
        check for check in checks
        if isinstance(check.get("semantic_delta_change"), (int, float)) and check["semantic_delta_change"] < 0.0
    ]
    better_checks = [
        check for check in checks
        if isinstance(check.get("semantic_delta_change"), (int, float)) and check["semantic_delta_change"] > 0.0
    ]
    worse_checks.sort(key=lambda item: item["semantic_delta_change"])
    better_checks.sort(key=lambda item: item["semantic_delta_change"], reverse=True)

    return {
        "status": "compared",
        "message": (
            f"previous={previous_summary.get('overall_status', 'unknown')} "
            f"current={current_summary.get('overall_status', 'unknown')} "
            f"status_changes={len(status_changes)}"
        ),
        "previous_github": previous.get("github", {}),
        "current_github": current.get("github", {}),
        "previous_summary": previous_summary,
        "current_summary": current_summary,
        "scenario_comparisons": scenarios,
        "check_comparisons": checks,
        "highlights": {
            "status_changes": status_changes[:10],
            "worse_checks": worse_checks[:10],
            "better_checks": better_checks[:10],
        },
    }


def write_markdown(report: dict, output_path: pathlib.Path) -> None:
    lines = [
        "# Nightly Trend",
        "",
        f"- Status: `{report['status']}`",
        f"- Message: {report['message']}",
    ]

    current_github = report.get("current_github") or {}
    previous_github = report.get("previous_github") or {}
    if current_github.get("github_run_id"):
        lines.append(f"- Current run: `{current_github.get('github_run_id')}`")
    if previous_github.get("github_run_id"):
        lines.append(f"- Previous run: `{previous_github.get('github_run_id')}`")

    if report["status"] == "missing_previous":
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
        return

    lines.extend(
        [
            "",
            "## Scenario Changes",
            "",
            "| Scenario | Previous | Current | Prev Warn | Curr Warn | Prev Gate | Curr Gate |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for scenario in report.get("scenario_comparisons", []):
        lines.append(
            "| "
            f"{scenario.get('name', '')} | "
            f"{scenario.get('previous_status', '-') or '-'} | "
            f"{scenario.get('current_status', '-') or '-'} | "
            f"{format_value(scenario.get('previous_warnings'))} | "
            f"{format_value(scenario.get('current_warnings'))} | "
            f"{format_value(scenario.get('previous_gating_failures'))} | "
            f"{format_value(scenario.get('current_gating_failures'))} |"
        )

    highlights = report.get("highlights", {})
    lines.extend(["", "## Check Highlights", ""])
    lines.append("### Status Changes")
    if highlights.get("status_changes"):
        for check in highlights["status_changes"]:
            lines.append(
                f"- `{check['name']}`: `{check.get('previous_status')}` -> `{check.get('current_status')}`"
            )
    else:
        lines.append("- None")

    lines.append("")
    lines.append("### Biggest Regressions")
    if highlights.get("worse_checks"):
        for check in highlights["worse_checks"]:
            lines.append(
                f"- `{check['name']}`: semantic delta change {format_delta(check.get('semantic_delta_change'))} "
                f"({format_delta(check.get('previous_semantic_delta_percent'))} -> {format_delta(check.get('current_semantic_delta_percent'))})"
            )
    else:
        lines.append("- None")

    lines.append("")
    lines.append("### Biggest Improvements")
    if highlights.get("better_checks"):
        for check in highlights["better_checks"]:
            lines.append(
                f"- `{check['name']}`: semantic delta change {format_delta(check.get('semantic_delta_change'))} "
                f"({format_delta(check.get('previous_semantic_delta_percent'))} -> {format_delta(check.get('current_semantic_delta_percent'))})"
            )
    else:
        lines.append("- None")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current", required=True)
    parser.add_argument("--previous", required=True)
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--markdown-output", required=True)
    args = parser.parse_args()

    current = load_json(pathlib.Path(args.current))
    if current is None:
        print("current nightly_metrics.json is missing", file=sys.stderr)
        return 1
    previous = load_json(pathlib.Path(args.previous))
    report = build_report(previous, current)

    json_output_path = pathlib.Path(args.json_output)
    json_output_path.parent.mkdir(parents=True, exist_ok=True)
    json_output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(report, pathlib.Path(args.markdown_output))

    print(
        "nightly_trend_status="
        f"{report['status']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
