#!/usr/bin/env python3

import argparse
import json
import os
import pathlib
import sys


def load_json(path: pathlib.Path) -> dict | None:
    if not path.is_file():
        return None
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def github_metadata() -> dict:
    keys = (
        "GITHUB_WORKFLOW",
        "GITHUB_RUN_ID",
        "GITHUB_RUN_ATTEMPT",
        "GITHUB_SHA",
        "GITHUB_REF",
        "GITHUB_REF_NAME",
        "GITHUB_EVENT_NAME",
        "GITHUB_REPOSITORY",
        "GITHUB_SERVER_URL",
    )
    metadata = {key.lower(): os.environ.get(key, "") for key in keys}
    server_url = metadata.get("github_server_url", "")
    repository = metadata.get("github_repository", "")
    run_id = metadata.get("github_run_id", "")
    if server_url and repository and run_id:
        metadata["run_url"] = f"{server_url}/{repository}/actions/runs/{run_id}"
    else:
        metadata["run_url"] = ""
    return metadata


def summarize_perf(report: dict | None) -> dict:
    if report is None:
        return {
            "status": "missing",
            "message": "performance comparison report is missing",
            "checks": [],
        }
    summary = report.get("summary", {})
    return {
        "status": summary.get("status", "unknown"),
        "message": (
            f"gating_failures={summary.get('gating_failures', 0)} "
            f"warnings={summary.get('warnings', 0)} errors={summary.get('errors', 0)}"
        ),
        "summary": summary,
        "checks": report.get("checks", []),
        "highlights": report.get("highlights", {}),
        "scenarios": report.get("scenarios", []),
    }


def summarize_tsan(report: dict | None) -> dict:
    if report is None:
        return {
            "status": "missing",
            "message": "TSan summary report is missing",
        }
    return {
        "status": report.get("status", "unknown"),
        "message": report.get("message", ""),
        "tests": report.get("tests", {}),
        "commands": report.get("commands", {}),
        "preflight": report.get("preflight"),
    }


def overall_status(perf_summary: dict, tsan_summary: dict) -> str:
    perf_status = perf_summary.get("status", "unknown")
    tsan_status = tsan_summary.get("status", "unknown")
    perf_warnings = perf_summary.get("summary", {}).get("warnings", 0)
    if perf_status in {"fail", "error", "missing"}:
        return "fail"
    if tsan_status in {"fail", "build_failed", "compile_failed", "invalid_results", "missing", "no_results", "preflight_failed"}:
        return "fail"
    if perf_warnings > 0:
        return "warn"
    if tsan_status == "runtime_unsupported":
        return "warn"
    if perf_status == "pass" and tsan_status == "pass":
        return "pass"
    return "warn"


def extract_nightly_metrics(perf_summary: dict, tsan_summary: dict, overall: str) -> dict:
    perf_checks = []
    for check in perf_summary.get("checks", []):
        perf_checks.append(
            {
                "name": check.get("name"),
                "scenario": check.get("scenario"),
                "metric": check.get("metric"),
                "status": check.get("status"),
                "classification": check.get("classification"),
                "direction": check.get("direction"),
                "baseline": check.get("baseline"),
                "candidate": check.get("candidate"),
                "delta_percent": check.get("delta_percent"),
                "semantic_delta_percent": check.get("semantic_delta_percent"),
                "message": check.get("message"),
            }
        )

    scenario_statuses = [
        {
            "name": scenario.get("name"),
            "status": scenario.get("status"),
            "gating_failures": scenario.get("gating_failures", 0),
            "warnings": scenario.get("warnings", 0),
            "errors": scenario.get("errors", 0),
        }
        for scenario in perf_summary.get("scenarios", [])
    ]

    tsan_tests = tsan_summary.get("tests", {})
    return {
        "summary": {
            "overall_status": overall,
            "performance_status": perf_summary.get("status"),
            "performance_gating_failures": perf_summary.get("summary", {}).get("gating_failures", 0),
            "performance_warnings": perf_summary.get("summary", {}).get("warnings", 0),
            "performance_errors": perf_summary.get("summary", {}).get("errors", 0),
            "tsan_status": tsan_summary.get("status"),
            "tsan_tests": tsan_tests.get("tests", 0),
            "tsan_failures": tsan_tests.get("failures", 0),
            "tsan_errors": tsan_tests.get("errors", 0),
        },
        "scenario_statuses": scenario_statuses,
        "performance_checks": perf_checks,
        "tsan": {
            "status": tsan_summary.get("status"),
            "message": tsan_summary.get("message"),
            "preflight_status": (tsan_summary.get("preflight") or {}).get("status"),
            "compiler": (tsan_summary.get("preflight") or {}).get("compiler"),
            "tests": tsan_tests.get("tests", 0),
            "failures": tsan_tests.get("failures", 0),
            "errors": tsan_tests.get("errors", 0),
        },
    }


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


def build_report(args: argparse.Namespace) -> dict:
    perf_json_path = pathlib.Path(args.perf_comparison_json).resolve()
    tsan_json_path = pathlib.Path(args.tsan_summary_json).resolve()
    perf_md_path = pathlib.Path(args.perf_comparison_markdown).resolve()
    tsan_md_path = pathlib.Path(args.tsan_summary_markdown).resolve()

    perf_report = load_json(perf_json_path)
    tsan_report = load_json(tsan_json_path)
    perf_summary = summarize_perf(perf_report)
    tsan_summary = summarize_tsan(tsan_report)
    overall = overall_status(perf_summary, tsan_summary)

    return {
        "status": overall,
        "github": github_metadata(),
        "artifacts": {
            "perf_comparison_json": str(perf_json_path),
            "perf_comparison_markdown": str(perf_md_path),
            "tsan_summary_json": str(tsan_json_path),
            "tsan_summary_markdown": str(tsan_md_path),
        },
        "performance": perf_summary,
        "metrics": extract_nightly_metrics(perf_summary, tsan_summary, overall),
        "tsan": tsan_summary,
    }


def write_markdown(report: dict, output_path: pathlib.Path) -> None:
    github = report["github"]
    perf = report["performance"]
    tsan = report["tsan"]
    lines = [
        "# Nightly Summary",
        "",
        f"- Overall status: `{report['status']}`",
        f"- Performance status: `{perf['status']}`",
        f"- TSan status: `{tsan['status']}`",
    ]
    if github.get("github_sha"):
        lines.append(f"- Git SHA: `{github['github_sha']}`")
    if github.get("github_ref_name"):
        lines.append(f"- Ref: `{github['github_ref_name']}`")
    if github.get("run_url"):
        lines.append(f"- Run URL: {github['run_url']}")
    lines.extend(
        [
            "",
            "## Performance",
            "",
            f"- Status: `{perf['status']}`",
            f"- Summary: {perf['message']}",
        ]
    )
    scenarios = perf.get("scenarios", [])
    if scenarios:
        lines.extend(
            [
                "",
                "| Scenario | Status | Gating Failures | Warnings | Errors |",
                "| --- | --- | ---: | ---: | ---: |",
            ]
        )
        for scenario in scenarios:
            lines.append(
                "| "
                f"{scenario.get('name', '')} | "
                f"{scenario.get('status', '')} | "
                f"{scenario.get('gating_failures', 0)} | "
                f"{scenario.get('warnings', 0)} | "
                f"{scenario.get('errors', 0)} |"
            )

    highlights = perf.get("highlights", {})
    regressions = highlights.get("largest_regressions", [])
    if regressions:
        lines.extend(["", "### Largest Performance Regressions", ""])
        for check in regressions:
            lines.append(
                f"- `{check.get('name', '')}`: {format_delta(check.get('delta_percent'))} "
                f"({format_value(check.get('baseline'))} -> {format_value(check.get('candidate'))}), "
                f"{check.get('message', '')}"
            )

    lines.extend(
        [
            "",
            "## TSan",
            "",
            f"- Status: `{tsan['status']}`",
            f"- Summary: {tsan['message']}",
        ]
    )
    preflight = tsan.get("preflight")
    if preflight is not None:
        lines.append(
            f"- Preflight: `{preflight.get('status', 'unknown')}` with `{preflight.get('compiler', '')}`"
        )
    tests = tsan.get("tests", {})
    if tests:
        lines.append(
            f"- Tests / failures / errors: `{tests.get('tests', 0)}` / `{tests.get('failures', 0)}` / `{tests.get('errors', 0)}`"
        )
        failing = tests.get("failing_testcases", [])
        if failing:
            lines.extend(["", "### Failing TSan Testcases", ""])
            for testcase in failing[:10]:
                label = testcase.get("classname") or testcase.get("file", "")
                lines.append(
                    f"- `{label}.{testcase.get('name', '')}`: {testcase.get('message', testcase.get('status', ''))}"
                )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--perf-comparison-json", required=True)
    parser.add_argument("--perf-comparison-markdown", required=True)
    parser.add_argument("--tsan-summary-json", required=True)
    parser.add_argument("--tsan-summary-markdown", required=True)
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--markdown-output", required=True)
    parser.add_argument("--metrics-output", required=True)
    args = parser.parse_args()

    report = build_report(args)
    json_output_path = pathlib.Path(args.json_output)
    json_output_path.parent.mkdir(parents=True, exist_ok=True)
    json_output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(report, pathlib.Path(args.markdown_output))
    metrics_output_path = pathlib.Path(args.metrics_output)
    metrics_output_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_output_path.write_text(
        json.dumps(
            {
                "github": report["github"],
                "artifacts": report["artifacts"],
                "metrics": report["metrics"],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    print(
        "nightly_summary_status="
        f"{report['status']} perf={report['performance']['status']} tsan={report['tsan']['status']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
