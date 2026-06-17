#!/usr/bin/env python3

import argparse
import json
import math
import pathlib
import sys


def load_json(path: pathlib.Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def scenario_map(report: dict) -> dict:
    return {scenario["name"]: scenario for scenario in report.get("scenarios", [])}


def metric_value(scenario: dict, key: str):
    return scenario.get("aggregated", {}).get(key)


def percent_delta(candidate: float, baseline: float) -> float | None:
    if baseline == 0:
        return None
    return (candidate - baseline) / baseline * 100.0


def make_check(
    scenario: str,
    metric: str,
    baseline_value,
    candidate_value,
    *,
    min_ratio: float | None = None,
    max_ratio: float | None = None,
    gating: bool = True,
    higher_is_better: bool = True,
) -> dict:
    name = f"{scenario}.{metric}"
    check = {
        "name": name,
        "scenario": scenario,
        "metric": metric,
        "baseline": baseline_value,
        "candidate": candidate_value,
        "gating": gating,
        "classification": "gating" if gating else "trend",
        "direction": "higher_is_better" if higher_is_better else "lower_is_better",
        "status": "pass",
        "message": "metric within threshold",
    }
    if min_ratio is not None or max_ratio is not None:
        check["threshold"] = {
            "min_ratio": min_ratio,
            "max_ratio": max_ratio,
        }

    if baseline_value is None or candidate_value is None:
        check["status"] = "error"
        check["message"] = "metric is missing from baseline or candidate report"
        return check

    if not isinstance(baseline_value, (int, float)) or not isinstance(candidate_value, (int, float)):
        check["status"] = "error"
        check["message"] = "metric is not numeric"
        return check

    if math.isnan(baseline_value) or math.isnan(candidate_value):
        check["status"] = "error"
        check["message"] = "metric contains NaN"
        return check

    delta = percent_delta(float(candidate_value), float(baseline_value))
    check["delta_percent"] = delta
    check["semantic_delta_percent"] = None if delta is None else (delta if higher_is_better else -delta)

    if min_ratio is not None and candidate_value < baseline_value * min_ratio:
        check["status"] = "fail" if gating else "warn"
        check["message"] = (
            f"candidate dropped below {min_ratio:.2f}x baseline"
        )
        return check

    if max_ratio is not None and candidate_value > baseline_value * max_ratio:
        check["status"] = "fail" if gating else "warn"
        check["message"] = (
            f"candidate exceeded {max_ratio:.2f}x baseline"
        )
        return check

    return check


def candidate_only_check(
    scenario: str,
    metric: str,
    candidate_value,
    *,
    min_value: float | None = None,
    max_value: float | None = None,
    gating: bool = True,
    message_on_pass: str = "metric within threshold",
) -> dict:
    check = {
        "name": f"{scenario}.{metric}",
        "scenario": scenario,
        "metric": metric,
        "baseline": None,
        "candidate": candidate_value,
        "gating": gating,
        "classification": "gating" if gating else "trend",
        "status": "pass",
        "message": message_on_pass,
    }
    if min_value is not None or max_value is not None:
        check["threshold"] = {
            "min_value": min_value,
            "max_value": max_value,
        }

    if candidate_value is None:
        check["status"] = "error"
        check["message"] = "metric is missing from candidate report"
        return check
    if not isinstance(candidate_value, (int, float)):
        check["status"] = "error"
        check["message"] = "metric is not numeric"
        return check
    if math.isnan(candidate_value):
        check["status"] = "error"
        check["message"] = "metric contains NaN"
        return check

    if min_value is not None and candidate_value < min_value:
        check["status"] = "fail" if gating else "warn"
        check["message"] = f"candidate dropped below {min_value}"
        return check

    if max_value is not None and candidate_value > max_value:
        check["status"] = "fail" if gating else "warn"
        check["message"] = f"candidate exceeded {max_value}"
        return check

    return check


def report_metadata(report: dict) -> dict:
    scenarios = report.get("scenarios", [])
    return {
        "path": report.get("_path"),
        "generated_at_epoch_seconds": report.get("generated_at_epoch_seconds"),
        "repeat": report.get("repeat"),
        "workspace_root": report.get("workspace_root"),
        "scenario_names": [scenario.get("name") for scenario in scenarios],
        "soak_duration_seconds": report.get("soak", {}).get("duration_seconds"),
    }


def scenario_summary(name: str, checks: list[dict]) -> dict:
    failures = [check for check in checks if check["status"] == "fail"]
    warnings = [check for check in checks if check["status"] == "warn"]
    errors = [check for check in checks if check["status"] == "error"]
    return {
        "name": name,
        "status": "fail" if failures or errors else "pass",
        "gating_failures": len(failures),
        "warnings": len(warnings),
        "errors": len(errors),
        "checks": checks,
    }


def build_scenario_summaries(checks: list[dict]) -> list[dict]:
    grouped: dict[str, list[dict]] = {}
    for check in checks:
        grouped.setdefault(check["scenario"], []).append(check)
    return [scenario_summary(name, grouped[name]) for name in sorted(grouped.keys())]


def highlight_checks(checks: list[dict], *, limit: int = 3) -> dict:
    delta_checks = [
        check for check in checks if isinstance(check.get("semantic_delta_percent"), (int, float))
    ]
    regressions = sorted(
        [check for check in delta_checks if check["semantic_delta_percent"] < 0.0],
        key=lambda check: check["semantic_delta_percent"],
    )
    improvements = sorted(
        [check for check in delta_checks if check["semantic_delta_percent"] > 0.0],
        key=lambda check: check["semantic_delta_percent"],
        reverse=True,
    )
    return {
        "largest_regressions": regressions[:limit],
        "largest_improvements": improvements[:limit],
        "missing_or_invalid": [
            check for check in checks if check["status"] == "error"
        ][:limit],
    }


def format_value(value) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if value is None:
        return "-"
    return str(value)


def format_delta(delta_percent: float | None) -> str:
    if delta_percent is None:
        return "-"
    return f"{delta_percent:+.2f}%"


def markdown_table(checks: list[dict]) -> list[str]:
    lines = [
        "| Check | Baseline | Candidate | Delta | Status | Message |",
        "| --- | ---: | ---: | ---: | --- | --- |",
    ]
    for check in checks:
        lines.append(
            "| "
            f"{check['name']} | "
            f"{format_value(check.get('baseline'))} | "
            f"{format_value(check.get('candidate'))} | "
            f"{format_delta(check.get('delta_percent'))} | "
            f"{check['status']} | "
            f"{check['message']} |"
        )
    return lines


def build_markdown_summary(report: dict) -> str:
    summary = report["summary"]
    baseline = report["baseline"]
    candidate = report["candidate"]
    gating_checks = [check for check in report["checks"] if check["gating"]]
    trend_checks = [check for check in report["checks"] if not check["gating"]]

    lines = [
        "# Performance Comparison Summary",
        "",
        f"- Status: `{summary['status']}`",
        f"- Gating failures: `{summary['gating_failures']}`",
        f"- Warnings: `{summary['warnings']}`",
        f"- Errors: `{summary['errors']}`",
        "",
        "## Inputs",
        "",
        f"- Baseline: `{baseline['path']}`",
        f"- Candidate: `{candidate['path']}`",
        f"- Baseline repeat / soak: `{baseline['repeat']}` / `{baseline['soak_duration_seconds']}` s",
        f"- Candidate repeat / soak: `{candidate['repeat']}` / `{candidate['soak_duration_seconds']}` s",
        "",
        "## Gating Checks",
        "",
    ]
    lines.extend(markdown_table(gating_checks))
    lines.extend(
        [
            "",
            "## Trend Checks",
            "",
        ]
    )
    lines.extend(markdown_table(trend_checks))
    lines.extend(
        [
            "",
            "## Scenario Summary",
            "",
            "| Scenario | Status | Gating Failures | Warnings | Errors |",
            "| --- | --- | ---: | ---: | ---: |",
        ]
    )
    for scenario in report["scenarios"]:
        lines.append(
            "| "
            f"{scenario['name']} | "
            f"{scenario['status']} | "
            f"{scenario['gating_failures']} | "
            f"{scenario['warnings']} | "
            f"{scenario['errors']} |"
        )

    highlights = report["highlights"]
    lines.extend(
        [
            "",
            "## Highlights",
            "",
            "### Largest Regressions",
        ]
    )
    if highlights["largest_regressions"]:
        for check in highlights["largest_regressions"]:
            lines.append(
                f"- `{check['name']}`: {format_delta(check.get('delta_percent'))} "
                f"({format_value(check.get('baseline'))} -> {format_value(check.get('candidate'))})"
            )
    else:
        lines.append("- None")

    lines.append("")
    lines.append("### Largest Improvements")
    if highlights["largest_improvements"]:
        for check in highlights["largest_improvements"]:
            lines.append(
                f"- `{check['name']}`: {format_delta(check.get('delta_percent'))} "
                f"({format_value(check.get('baseline'))} -> {format_value(check.get('candidate'))})"
            )
    else:
        lines.append("- None")

    lines.append("")
    lines.append("### Missing Or Invalid Metrics")
    if highlights["missing_or_invalid"]:
        for check in highlights["missing_or_invalid"]:
            lines.append(f"- `{check['name']}`: {check['message']}")
    else:
        lines.append("- None")

    return "\n".join(lines) + "\n"


def build_report(baseline: dict, candidate: dict) -> dict:
    baseline_scenarios = scenario_map(baseline)
    candidate_scenarios = scenario_map(candidate)

    checks = []

    scheduler_baseline = baseline_scenarios.get("scheduler_1khz", {})
    scheduler_candidate = candidate_scenarios.get("scheduler_1khz", {})
    checks.append(
        make_check(
            "scheduler_1khz",
            "throughput_hz_mean",
            metric_value(scheduler_baseline, "throughput_hz_mean"),
            metric_value(scheduler_candidate, "throughput_hz_mean"),
            min_ratio=0.95,
        )
    )
    checks.append(
        make_check(
            "scheduler_1khz",
            "realtime_factor_mean",
            metric_value(scheduler_baseline, "realtime_factor_mean"),
            metric_value(scheduler_candidate, "realtime_factor_mean"),
            min_ratio=0.95,
        )
    )

    camera_baseline = baseline_scenarios.get("headless_camera", {})
    camera_candidate = candidate_scenarios.get("headless_camera", {})
    checks.append(
        make_check(
            "headless_camera",
            "sample_frequency_hz_mean",
            metric_value(camera_baseline, "sample_frequency_hz_mean"),
            metric_value(camera_candidate, "sample_frequency_hz_mean"),
            min_ratio=0.90,
        )
    )
    checks.append(
        make_check(
            "headless_camera",
            "sample_latency_ms_mean_mean",
            metric_value(camera_baseline, "sample_latency_ms_mean_mean"),
            metric_value(camera_candidate, "sample_latency_ms_mean_mean"),
            max_ratio=1.10,
            gating=False,
            higher_is_better=False,
        )
    )
    checks.append(
        make_check(
            "headless_camera",
            "sample_latency_ms_p95_mean",
            metric_value(camera_baseline, "sample_latency_ms_p95_mean"),
            metric_value(camera_candidate, "sample_latency_ms_p95_mean"),
            max_ratio=1.10,
            gating=False,
            higher_is_better=False,
        )
    )

    loop_baseline = baseline_scenarios.get("hardware_read_write_loop", {})
    loop_candidate = candidate_scenarios.get("hardware_read_write_loop", {})
    for key in (
        "write_ms_mean_mean",
        "write_ms_p95_mean",
        "read_ms_mean_mean",
        "read_ms_p95_mean",
        "loop_ms_mean_mean",
        "loop_ms_p95_mean",
    ):
        checks.append(
            make_check(
                "hardware_read_write_loop",
                key,
                metric_value(loop_baseline, key),
                metric_value(loop_candidate, key),
                gating=False,
                higher_is_better=False,
            )
        )

    soak = candidate.get("soak", {}).get("result", {})
    soak_checks = [
        {
            "name": "soak.iterations",
            "scenario": "soak",
            "metric": "iterations",
            "baseline": baseline.get("soak", {}).get("result", {}).get("iterations"),
            "candidate": soak.get("iterations"),
            "gating": True,
            "classification": "gating",
            "status": "pass" if soak.get("iterations", 0) > 0 else "fail",
            "message": "soak produced at least one completed iteration"
            if soak.get("iterations", 0) > 0
            else "soak did not complete any iteration",
        },
        {
            "name": "soak.rss_kb_samples",
            "scenario": "soak",
            "metric": "rss_kb_samples",
            "baseline": baseline.get("soak", {}).get("result", {}).get("rss_kb_samples"),
            "candidate": soak.get("rss_kb_samples"),
            "gating": True,
            "classification": "gating",
            "status": "pass" if soak.get("rss_kb_samples", 0) > 0 else "fail",
            "message": "soak collected RSS samples"
            if soak.get("rss_kb_samples", 0) > 0
            else "soak collected no RSS samples",
        },
    ]
    checks.extend(soak_checks)
    checks.append(
        candidate_only_check(
            "soak",
            "rss_kb_growth_ratio",
            soak.get("rss_kb_growth_ratio"),
            gating=False,
            max_value=1.10,
            message_on_pass="soak RSS tail/head median ratio within tracking threshold",
        )
    )
    checks.append(
        candidate_only_check(
            "soak",
            "rss_kb_growth_kb",
            soak.get("rss_kb_growth_kb"),
            gating=False,
            max_value=10240,
            message_on_pass="soak RSS head/tail median growth within tracking threshold",
        )
    )

    failures = [check for check in checks if check["status"] == "fail"]
    warnings = [check for check in checks if check["status"] == "warn"]
    errors = [check for check in checks if check["status"] == "error"]
    scenarios = build_scenario_summaries(checks)

    return {
        "baseline_path": baseline.get("_path"),
        "candidate_path": candidate.get("_path"),
        "baseline": report_metadata(baseline),
        "candidate": report_metadata(candidate),
        "summary": {
            "status": "fail" if failures or errors else "pass",
            "gating_failures": len(failures),
            "warnings": len(warnings),
            "errors": len(errors),
        },
        "scenarios": scenarios,
        "gating_checks": [check for check in checks if check["gating"]],
        "trend_checks": [check for check in checks if not check["gating"]],
        "highlights": highlight_checks(checks),
        "checks": checks,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--output", default="")
    parser.add_argument("--markdown-output", default="")
    args = parser.parse_args()

    baseline_path = pathlib.Path(args.baseline).resolve()
    candidate_path = pathlib.Path(args.candidate).resolve()

    baseline = load_json(baseline_path)
    candidate = load_json(candidate_path)
    baseline["_path"] = str(baseline_path)
    candidate["_path"] = str(candidate_path)

    report = build_report(baseline, candidate)
    output = json.dumps(report, indent=2, sort_keys=True)

    if args.output:
        output_path = pathlib.Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(output + "\n", encoding="utf-8")
    else:
        print(output)

    if args.markdown_output:
        markdown_path = pathlib.Path(args.markdown_output)
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        markdown_path.write_text(build_markdown_summary(report), encoding="utf-8")

    summary = report["summary"]
    print(
        "comparison_status="
        f"{summary['status']} gating_failures={summary['gating_failures']} "
        f"warnings={summary['warnings']} errors={summary['errors']}"
    )

    return 1 if summary["status"] != "pass" else 0


if __name__ == "__main__":
    sys.exit(main())
