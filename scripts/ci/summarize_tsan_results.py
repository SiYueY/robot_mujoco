#!/usr/bin/env python3

import argparse
import json
import pathlib
import sys
import xml.etree.ElementTree as ET


def load_json(path: pathlib.Path) -> dict | None:
    if not path.is_file():
        return None
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_int(value: str | None) -> int:
    if value is None or value == "":
        return 0
    return int(value)


def parse_float(value: str | None) -> float:
    if value is None or value == "":
        return 0.0
    return float(value)


def find_test_result_files(build_base: pathlib.Path) -> list[pathlib.Path]:
    return sorted(build_base.glob("*/test_results/*/*.xml"))


def parse_test_results(files: list[pathlib.Path], workspace_root: pathlib.Path) -> dict:
    summary = {
        "files": [],
        "tests": 0,
        "failures": 0,
        "errors": 0,
        "disabled": 0,
        "time_seconds": 0.0,
        "failing_testcases": [],
        "invalid_files": [],
    }

    for file_path in files:
        relative_path = str(file_path.relative_to(workspace_root))
        try:
            root = ET.parse(file_path).getroot()
        except ET.ParseError as exc:
            summary["invalid_files"].append(
                {
                    "path": relative_path,
                    "error": str(exc),
                }
            )
            continue

        file_entry = {
            "path": relative_path,
            "tests": parse_int(root.get("tests")),
            "failures": parse_int(root.get("failures")),
            "errors": parse_int(root.get("errors")),
            "disabled": parse_int(root.get("disabled")),
            "time_seconds": parse_float(root.get("time")),
        }
        summary["files"].append(file_entry)
        summary["tests"] += file_entry["tests"]
        summary["failures"] += file_entry["failures"]
        summary["errors"] += file_entry["errors"]
        summary["disabled"] += file_entry["disabled"]
        summary["time_seconds"] += file_entry["time_seconds"]

        for testcase in root.iter("testcase"):
            failure = testcase.find("failure")
            error = testcase.find("error")
            if failure is None and error is None:
                continue
            summary["failing_testcases"].append(
                {
                    "file": relative_path,
                    "classname": testcase.get("classname", ""),
                    "name": testcase.get("name", ""),
                    "time_seconds": parse_float(testcase.get("time")),
                    "status": "error" if error is not None else "failure",
                    "message": (
                        (error.get("message") if error is not None else failure.get("message"))
                        or ""
                    ),
                }
            )

    return summary


def determine_status(
    *,
    preflight: dict | None,
    build_exit_code: int,
    test_exit_code: int,
    test_result_exit_code: int,
    tests_summary: dict,
) -> tuple[str, str]:
    if preflight is not None:
        preflight_status = preflight.get("status", "unknown")
        if preflight_status == "runtime_unsupported":
            return "runtime_unsupported", "host ThreadSanitizer runtime is unsupported"
        if preflight_status == "compile_failed":
            return "compile_failed", "ThreadSanitizer preflight compilation failed"
        return "preflight_failed", f"ThreadSanitizer preflight failed with status {preflight_status}"

    if build_exit_code != 0:
        return "build_failed", "TSan build failed before test execution"
    if tests_summary["invalid_files"]:
        return "invalid_results", "test result XML parsing failed"
    if not tests_summary["files"]:
        return "no_results", "no TSan test result XML files were produced"
    if test_exit_code != 0 or test_result_exit_code != 0:
        return "fail", "TSan tests reported failures or errors"
    if tests_summary["failures"] != 0 or tests_summary["errors"] != 0:
        return "fail", "TSan test result XML contains failures or errors"
    return "pass", "TSan subset completed without reported failures"


def build_report(args: argparse.Namespace) -> dict:
    workspace_root = pathlib.Path(args.workspace_root).resolve()
    build_base = pathlib.Path(args.build_base).resolve()
    log_base = pathlib.Path(args.log_base).resolve()
    preflight_report = pathlib.Path(args.preflight_report).resolve()

    preflight = load_json(preflight_report)
    test_result_files = find_test_result_files(build_base)
    tests_summary = parse_test_results(test_result_files, workspace_root)
    status, message = determine_status(
        preflight=preflight,
        build_exit_code=args.build_exit_code,
        test_exit_code=args.test_exit_code,
        test_result_exit_code=args.test_result_exit_code,
        tests_summary=tests_summary,
    )

    return {
        "status": status,
        "message": message,
        "artifacts": {
            "workspace_root": str(workspace_root),
            "build_base": str(build_base),
            "log_base": str(log_base),
            "preflight_report": str(preflight_report),
        },
        "commands": {
            "packages": args.packages,
            "ctest_regex": args.ctest_regex,
            "build_exit_code": args.build_exit_code,
            "test_exit_code": args.test_exit_code,
            "test_result_exit_code": args.test_result_exit_code,
        },
        "preflight": preflight,
        "tests": tests_summary,
    }


def write_markdown(report: dict, output_path: pathlib.Path) -> None:
    tests = report["tests"]
    commands = report["commands"]
    lines = [
        "# TSan Summary",
        "",
        f"- Status: `{report['status']}`",
        f"- Message: {report['message']}",
        f"- Packages: `{', '.join(commands['packages'])}`",
        f"- CTest regex: `{commands['ctest_regex']}`",
        f"- Build / test / result exit codes: `{commands['build_exit_code']}` / `{commands['test_exit_code']}` / `{commands['test_result_exit_code']}`",
        "",
    ]

    preflight = report.get("preflight")
    if preflight is not None:
        lines.extend(
            [
                "## Preflight",
                "",
                f"- Status: `{preflight.get('status', 'unknown')}`",
                f"- Compiler: `{preflight.get('compiler', '')}`",
            ]
        )
        stderr = (preflight.get("stderr") or "").strip()
        if stderr:
            lines.extend(["", "```text", stderr, "```"])
        lines.append("")

    lines.extend(
        [
            "## Test Results",
            "",
            f"- XML files: `{len(tests['files'])}`",
            f"- Tests / failures / errors / disabled: `{tests['tests']}` / `{tests['failures']}` / `{tests['errors']}` / `{tests['disabled']}`",
            f"- Total reported time: `{tests['time_seconds']:.3f}` s",
            "",
        ]
    )

    if tests["invalid_files"]:
        lines.extend(["### Invalid XML", ""])
        for entry in tests["invalid_files"]:
            lines.append(f"- `{entry['path']}`: {entry['error']}")
        lines.append("")

    if tests["failing_testcases"]:
        lines.extend(["### Failing Testcases", ""])
        for testcase in tests["failing_testcases"][:20]:
            label = testcase["classname"] or testcase["file"]
            message = testcase["message"] or testcase["status"]
            lines.append(f"- `{label}.{testcase['name']}`: {message}")
        if len(tests["failing_testcases"]) > 20:
            lines.append(
                f"- ... and {len(tests['failing_testcases']) - 20} more failing testcases"
            )
        lines.append("")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace-root", default=str(pathlib.Path(__file__).resolve().parents[2]))
    parser.add_argument("--build-base", required=True)
    parser.add_argument("--log-base", required=True)
    parser.add_argument("--preflight-report", required=True)
    parser.add_argument("--ctest-regex", required=True)
    parser.add_argument("--packages", nargs="+", required=True)
    parser.add_argument("--build-exit-code", type=int, default=0)
    parser.add_argument("--test-exit-code", type=int, default=0)
    parser.add_argument("--test-result-exit-code", type=int, default=0)
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--markdown-output", required=True)
    args = parser.parse_args()

    report = build_report(args)
    json_output_path = pathlib.Path(args.json_output)
    json_output_path.parent.mkdir(parents=True, exist_ok=True)
    json_output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(report, pathlib.Path(args.markdown_output))

    print(
        "tsan_summary_status="
        f"{report['status']} tests={report['tests']['tests']} "
        f"failures={report['tests']['failures']} errors={report['tests']['errors']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
