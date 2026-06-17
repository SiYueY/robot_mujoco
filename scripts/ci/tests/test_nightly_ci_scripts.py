#!/usr/bin/env python3

import importlib.util
import json
import pathlib
import tempfile
import unittest


TEST_ROOT = pathlib.Path(__file__).resolve().parent
CI_ROOT = TEST_ROOT.parent


def load_module(name: str, filename: str):
    path = CI_ROOT / filename
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


fetch_previous = load_module("fetch_previous_nightly_artifact", "fetch_previous_nightly_artifact.py")
compare_performance = load_module("compare_performance_baseline", "compare_performance_baseline.py")
compare_nightly = load_module("compare_nightly_metrics", "compare_nightly_metrics.py")
summarize_nightly = load_module("summarize_nightly_results", "summarize_nightly_results.py")
summarize_history = load_module("summarize_nightly_history", "summarize_nightly_history.py")
summarize_tsan = load_module("summarize_tsan_results", "summarize_tsan_results.py")


class FetchPreviousNightlyArtifactTest(unittest.TestCase):
    def test_github_dot_com_uses_public_api_host(self):
        self.assertEqual(
            fetch_previous.github_api_base_url("https://github.com"),
            "https://api.github.com",
        )

    def test_ghes_uses_api_v3_path(self):
        self.assertEqual(
            fetch_previous.github_api_base_url("https://git.example.internal"),
            "https://git.example.internal/api/v3",
        )


class SummarizeNightlyResultsTest(unittest.TestCase):
    def test_overall_status_warns_on_perf_warning(self):
        perf = {"status": "pass", "summary": {"warnings": 1}}
        tsan = {"status": "pass"}
        self.assertEqual(summarize_nightly.overall_status(perf, tsan), "warn")

    def test_overall_status_warns_on_runtime_unsupported(self):
        perf = {"status": "pass", "summary": {"warnings": 0}}
        tsan = {"status": "runtime_unsupported"}
        self.assertEqual(summarize_nightly.overall_status(perf, tsan), "warn")

    def test_overall_status_passes_only_when_clean(self):
        perf = {"status": "pass", "summary": {"warnings": 0}}
        tsan = {"status": "pass"}
        self.assertEqual(summarize_nightly.overall_status(perf, tsan), "pass")


class ComparePerformanceBaselineTest(unittest.TestCase):
    def test_latency_regression_is_reported_as_semantic_regression(self):
        baseline = {
            "scenarios": [
                {
                    "name": "scheduler_1khz",
                    "aggregated": {
                        "throughput_hz_mean": 100.0,
                        "realtime_factor_mean": 1.0,
                    },
                },
                {
                    "name": "headless_camera",
                    "aggregated": {
                        "sample_frequency_hz_mean": 50.0,
                        "sample_latency_ms_mean_mean": 1.0,
                        "sample_latency_ms_p95_mean": 2.0,
                    },
                },
                {
                    "name": "hardware_read_write_loop",
                    "aggregated": {
                        "write_ms_mean_mean": 1.0,
                        "write_ms_p95_mean": 1.0,
                        "read_ms_mean_mean": 1.0,
                        "read_ms_p95_mean": 1.0,
                        "loop_ms_mean_mean": 1.0,
                        "loop_ms_p95_mean": 1.0,
                    },
                },
            ],
            "soak": {
                "result": {
                    "iterations": 1,
                    "rss_kb_samples": 1,
                    "rss_kb_growth_ratio": 1.0,
                    "rss_kb_growth_kb": 0,
                }
            },
        }
        candidate = {
            "scenarios": [
                {
                    "name": "scheduler_1khz",
                    "aggregated": {
                        "throughput_hz_mean": 100.0,
                        "realtime_factor_mean": 1.0,
                    },
                },
                {
                    "name": "headless_camera",
                    "aggregated": {
                        "sample_frequency_hz_mean": 50.0,
                        "sample_latency_ms_mean_mean": 1.3,
                        "sample_latency_ms_p95_mean": 2.4,
                    },
                },
                {
                    "name": "hardware_read_write_loop",
                    "aggregated": {
                        "write_ms_mean_mean": 1.0,
                        "write_ms_p95_mean": 1.0,
                        "read_ms_mean_mean": 1.0,
                        "read_ms_p95_mean": 1.0,
                        "loop_ms_mean_mean": 1.0,
                        "loop_ms_p95_mean": 1.0,
                    },
                },
            ],
            "soak": {
                "result": {
                    "iterations": 1,
                    "rss_kb_samples": 1,
                    "rss_kb_growth_ratio": 1.0,
                    "rss_kb_growth_kb": 0,
                }
            },
        }
        report = compare_performance.build_report(baseline, candidate)
        latency_check = next(
            check for check in report["checks"]
            if check["name"] == "headless_camera.sample_latency_ms_mean_mean"
        )
        self.assertEqual(latency_check["status"], "warn")
        self.assertLess(latency_check["semantic_delta_percent"], 0.0)
        self.assertEqual(
            report["highlights"]["largest_regressions"][0]["name"],
            "headless_camera.sample_latency_ms_mean_mean",
        )

    def test_missing_metric_surfaces_error_highlight(self):
        baseline = {"scenarios": [], "soak": {"result": {}}}
        candidate = {"scenarios": [], "soak": {"result": {}}}
        report = compare_performance.build_report(baseline, candidate)
        self.assertEqual(report["summary"]["status"], "fail")
        self.assertGreater(len(report["highlights"]["missing_or_invalid"]), 0)

    def test_soak_growth_surfaces_warning_without_failing_report(self):
        baseline = {
            "scenarios": [
                {
                    "name": "scheduler_1khz",
                    "aggregated": {
                        "throughput_hz_mean": 100.0,
                        "realtime_factor_mean": 1.0,
                    },
                },
                {
                    "name": "headless_camera",
                    "aggregated": {
                        "sample_frequency_hz_mean": 50.0,
                        "sample_latency_ms_mean_mean": 1.0,
                        "sample_latency_ms_p95_mean": 2.0,
                    },
                },
                {
                    "name": "hardware_read_write_loop",
                    "aggregated": {
                        "write_ms_mean_mean": 1.0,
                        "write_ms_p95_mean": 1.0,
                        "read_ms_mean_mean": 1.0,
                        "read_ms_p95_mean": 1.0,
                        "loop_ms_mean_mean": 1.0,
                        "loop_ms_p95_mean": 1.0,
                    },
                },
            ],
            "soak": {"result": {"iterations": 1000, "rss_kb_samples": 10}},
        }
        candidate = {
            "scenarios": baseline["scenarios"],
            "soak": {
                "result": {
                    "iterations": 1000,
                    "rss_kb_samples": 10,
                    "rss_kb_growth_ratio": 1.25,
                    "rss_kb_growth_kb": 16384,
                }
            },
        }
        report = compare_performance.build_report(baseline, candidate)
        self.assertEqual(report["summary"]["status"], "pass")
        self.assertEqual(report["summary"]["warnings"], 2)
        growth_ratio_check = next(
            check for check in report["checks"] if check["name"] == "soak.rss_kb_growth_ratio"
        )
        self.assertEqual(growth_ratio_check["status"], "warn")
        self.assertFalse(growth_ratio_check["gating"])


class CompareNightlyMetricsTest(unittest.TestCase):
    def test_missing_previous_report(self):
        current = {
            "github": {"github_run_id": "12345"},
            "metrics": {"summary": {"overall_status": "warn"}},
        }
        report = compare_nightly.build_report(None, current)
        self.assertEqual(report["status"], "missing_previous")
        self.assertEqual(report["current_github"]["github_run_id"], "12345")


class SummarizeNightlyHistoryTest(unittest.TestCase):
    def test_window_regressions_filters_zero_change(self):
        histories = [
            {
                "name": "regression",
                "observations": [
                    {"semantic_delta_percent": -20.0, "status": "warn"},
                    {"semantic_delta_percent": -5.0, "status": "pass"},
                ],
            },
            {
                "name": "improvement",
                "observations": [
                    {"semantic_delta_percent": 10.0, "status": "pass"},
                    {"semantic_delta_percent": 2.0, "status": "pass"},
                ],
            },
            {
                "name": "no_change",
                "observations": [
                    {"semantic_delta_percent": 1.0, "status": "pass"},
                    {"semantic_delta_percent": 1.0, "status": "pass"},
                ],
            },
        ]
        regressions, improvements = summarize_history.window_regressions(histories)
        self.assertEqual([item["name"] for item in regressions], ["regression"])
        self.assertEqual([item["name"] for item in improvements], ["improvement"])

    def test_persistent_warnings_requires_multiple_warn_or_fail_runs(self):
        histories = [
            {
                "name": "persistent",
                "observations": [
                    {"run_id": "3", "status": "warn"},
                    {"run_id": "2", "status": "warn"},
                    {"run_id": "1", "status": "fail"},
                ],
            },
            {
                "name": "not_persistent",
                "observations": [
                    {"run_id": "3", "status": "warn"},
                    {"run_id": "2", "status": "pass"},
                    {"run_id": "1", "status": "warn"},
                ],
            },
        ]
        result = summarize_history.persistent_warnings(histories)
        self.assertEqual([item["name"] for item in result], ["persistent"])


class SummarizeTsanResultsTest(unittest.TestCase):
    def test_determine_status_runtime_unsupported(self):
        status, message = summarize_tsan.determine_status(
            preflight={"status": "runtime_unsupported"},
            build_exit_code=0,
            test_exit_code=0,
            test_result_exit_code=0,
            tests_summary={"invalid_files": [], "files": [], "failures": 0, "errors": 0},
        )
        self.assertEqual(status, "runtime_unsupported")
        self.assertIn("unsupported", message)

    def test_parse_test_results_collects_failures(self):
        with tempfile.TemporaryDirectory() as tempdir:
            workspace_root = pathlib.Path(tempdir)
            xml_dir = workspace_root / "build_tsan" / "pkg" / "test_results" / "pkg"
            xml_dir.mkdir(parents=True)
            xml_path = xml_dir / "sample.gtest.xml"
            xml_path.write_text(
                """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="2" failures="1" disabled="0" errors="0" time="0.123">
  <testsuite name="Suite" tests="2" failures="1" disabled="0" errors="0" time="0.123">
    <testcase name="Passes" classname="pkg.Suite" time="0.01" />
    <testcase name="Fails" classname="pkg.Suite" time="0.02">
      <failure message="boom"/>
    </testcase>
  </testsuite>
</testsuites>
""",
                encoding="utf-8",
            )
            summary = summarize_tsan.parse_test_results([xml_path], workspace_root)
            self.assertEqual(summary["tests"], 2)
            self.assertEqual(summary["failures"], 1)
            self.assertEqual(len(summary["failing_testcases"]), 1)
            self.assertEqual(summary["failing_testcases"][0]["name"], "Fails")

    def test_build_report_marks_fail_when_xml_has_failures(self):
        with tempfile.TemporaryDirectory() as tempdir:
            workspace_root = pathlib.Path(tempdir)
            build_base = workspace_root / "build_tsan"
            log_base = workspace_root / "log_tsan"
            xml_dir = build_base / "pkg" / "test_results" / "pkg"
            xml_dir.mkdir(parents=True)
            log_base.mkdir(parents=True)
            (xml_dir / "sample.gtest.xml").write_text(
                """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="1" disabled="0" errors="0" time="0.01">
  <testsuite name="Suite" tests="1" failures="1" disabled="0" errors="0" time="0.01">
    <testcase name="Fails" classname="pkg.Suite" time="0.01">
      <failure message="boom"/>
    </testcase>
  </testsuite>
</testsuites>
""",
                encoding="utf-8",
            )
            args = type(
                "Args",
                (),
                {
                    "workspace_root": str(workspace_root),
                    "build_base": str(build_base),
                    "log_base": str(log_base),
                    "preflight_report": str(log_base / "missing.json"),
                    "ctest_regex": ".*",
                    "packages": ["pkg"],
                    "build_exit_code": 0,
                    "test_exit_code": 1,
                    "test_result_exit_code": 0,
                },
            )()
            report = summarize_tsan.build_report(args)
            self.assertEqual(report["status"], "fail")
            self.assertEqual(report["tests"]["failures"], 1)


class NightlyPipelineEndToEndTest(unittest.TestCase):
    def test_end_to_end_nightly_chain_reports_warn_trend_and_history(self):
        baseline = {
            "generated_at_epoch_seconds": 1,
            "repeat": 1,
            "workspace_root": "/tmp/ws",
            "scenarios": [
                {
                    "name": "scheduler_1khz",
                    "aggregated": {
                        "throughput_hz_mean": 1000.0,
                        "realtime_factor_mean": 1.0,
                    },
                },
                {
                    "name": "headless_camera",
                    "aggregated": {
                        "sample_frequency_hz_mean": 50.0,
                        "sample_latency_ms_mean_mean": 1.0,
                        "sample_latency_ms_p95_mean": 2.0,
                    },
                },
                {
                    "name": "hardware_read_write_loop",
                    "aggregated": {
                        "write_ms_mean_mean": 0.1,
                        "write_ms_p95_mean": 0.2,
                        "read_ms_mean_mean": 0.1,
                        "read_ms_p95_mean": 0.2,
                        "loop_ms_mean_mean": 0.2,
                        "loop_ms_p95_mean": 0.3,
                    },
                },
            ],
            "soak": {
                "duration_seconds": 5,
                "result": {
                    "iterations": 1,
                    "rss_kb_samples": 5,
                    "rss_kb_growth_ratio": 1.0,
                    "rss_kb_growth_kb": 0,
                },
            },
        }
        candidate = {
            "generated_at_epoch_seconds": 2,
            "repeat": 3,
            "workspace_root": "/tmp/ws",
            "scenarios": [
                {
                    "name": "scheduler_1khz",
                    "aggregated": {
                        "throughput_hz_mean": 1000.0,
                        "realtime_factor_mean": 1.0,
                    },
                },
                {
                    "name": "headless_camera",
                    "aggregated": {
                        "sample_frequency_hz_mean": 50.0,
                        "sample_latency_ms_mean_mean": 1.3,
                        "sample_latency_ms_p95_mean": 2.5,
                    },
                },
                {
                    "name": "hardware_read_write_loop",
                    "aggregated": {
                        "write_ms_mean_mean": 0.16,
                        "write_ms_p95_mean": 0.22,
                        "read_ms_mean_mean": 0.09,
                        "read_ms_p95_mean": 0.18,
                        "loop_ms_mean_mean": 0.24,
                        "loop_ms_p95_mean": 0.31,
                    },
                },
            ],
            "soak": {
                "duration_seconds": 5,
                "result": {
                    "iterations": 1,
                    "rss_kb_samples": 6,
                    "rss_kb_growth_ratio": 1.0,
                    "rss_kb_growth_kb": 0,
                },
            },
        }
        perf_report = compare_performance.build_report(baseline, candidate)
        self.assertEqual(perf_report["summary"]["status"], "pass")
        self.assertEqual(perf_report["summary"]["warnings"], 2)

        tsan_report = {
            "status": "runtime_unsupported",
            "message": "host ThreadSanitizer runtime is unsupported",
            "tests": {"tests": 0, "failures": 0, "errors": 0},
            "commands": {},
            "preflight": {"status": "runtime_unsupported", "compiler": "g++"},
        }

        with tempfile.TemporaryDirectory() as tempdir:
            root = pathlib.Path(tempdir)
            perf_json = root / "performance_comparison.json"
            perf_md = root / "performance_comparison.md"
            tsan_json = root / "tsan_summary.json"
            tsan_md = root / "tsan_summary.md"
            perf_json.write_text(json.dumps(perf_report) + "\n", encoding="utf-8")
            perf_md.write_text("perf\n", encoding="utf-8")
            tsan_json.write_text(json.dumps(tsan_report) + "\n", encoding="utf-8")
            tsan_md.write_text("tsan\n", encoding="utf-8")

            args = type(
                "Args",
                (),
                {
                    "perf_comparison_json": str(perf_json),
                    "perf_comparison_markdown": str(perf_md),
                    "tsan_summary_json": str(tsan_json),
                    "tsan_summary_markdown": str(tsan_md),
                },
            )()
            nightly_report = summarize_nightly.build_report(args)
            self.assertEqual(nightly_report["status"], "warn")
            self.assertEqual(nightly_report["metrics"]["summary"]["performance_warnings"], 2)

            current_metrics = {
                "github": {"github_run_id": "12345", "github_ref_name": "main"},
                "artifacts": nightly_report["artifacts"],
                "metrics": nightly_report["metrics"],
            }
            previous_metrics = json.loads(json.dumps(current_metrics))
            previous_metrics["github"]["github_run_id"] = "12344"
            previous_metrics["metrics"]["summary"]["overall_status"] = "pass"
            previous_metrics["metrics"]["summary"]["performance_warnings"] = 0
            for check in previous_metrics["metrics"]["performance_checks"]:
                if check["name"] == "headless_camera.sample_latency_ms_mean_mean":
                    check["status"] = "pass"
                    check["semantic_delta_percent"] = -5.0
                    check["delta_percent"] = 5.0

            trend_report = compare_nightly.build_report(previous_metrics, current_metrics)
            self.assertEqual(trend_report["status"], "compared")
            self.assertEqual(
                trend_report["highlights"]["status_changes"][0]["name"],
                "headless_camera.sample_latency_ms_mean_mean",
            )

            history_report = summarize_history.build_report(
                [current_metrics, previous_metrics, previous_metrics]
            )
            self.assertEqual(history_report["status"], "history")
            self.assertGreaterEqual(len(history_report["highlights"]["persistent_warnings"]), 1)


if __name__ == "__main__":
    unittest.main()
