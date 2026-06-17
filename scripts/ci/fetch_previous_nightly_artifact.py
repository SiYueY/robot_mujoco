#!/usr/bin/env python3

import argparse
import io
import json
import pathlib
import shutil
import sys
import urllib.error
import urllib.parse
import urllib.request
import zipfile


def github_api_base_url(server_url: str) -> str:
    normalized = server_url.rstrip("/")
    if normalized == "https://github.com":
        return "https://api.github.com"
    return f"{normalized}/api/v3"


def github_api_request(url: str, token: str) -> dict:
    request = urllib.request.Request(
        url,
        headers={
            "Authorization": f"Bearer {token}",
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
        },
    )
    with urllib.request.urlopen(request) as response:
        return json.load(response)


def github_download_bytes(url: str, token: str) -> bytes:
    request = urllib.request.Request(
        url,
        headers={
            "Authorization": f"Bearer {token}",
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
        },
    )
    with urllib.request.urlopen(request) as response:
        return response.read()


def list_workflow_runs(server_url: str, repository: str, workflow: str, token: str, per_page: int) -> list[dict]:
    query = urllib.parse.urlencode({"status": "completed", "per_page": per_page})
    url = f"{github_api_base_url(server_url)}/repos/{repository}/actions/workflows/{workflow}/runs?{query}"
    return github_api_request(url, token).get("workflow_runs", [])


def list_run_artifacts(server_url: str, repository: str, run_id: int, token: str) -> list[dict]:
    url = f"{github_api_base_url(server_url)}/repos/{repository}/actions/runs/{run_id}/artifacts?per_page=100"
    return github_api_request(url, token).get("artifacts", [])


def download_artifact_zip(server_url: str, repository: str, artifact_id: int, token: str) -> bytes:
    url = f"{github_api_base_url(server_url)}/repos/{repository}/actions/artifacts/{artifact_id}/zip"
    return github_download_bytes(url, token)


def extract_zip(content: bytes, output_dir: pathlib.Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(content)) as archive:
        archive.extractall(output_dir)


def find_previous_run_ids(runs: list[dict], current_run_id: int, max_runs: int) -> list[int]:
    selected: list[int] = []
    for run in runs:
        if run.get("id") == current_run_id:
            continue
        if run.get("conclusion") == "success":
            selected.append(int(run["id"]))
        if len(selected) >= max_runs:
            break
    return selected


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-url", required=True)
    parser.add_argument("--repository", required=True)
    parser.add_argument("--workflow", default="nightly.yml")
    parser.add_argument("--current-run-id", type=int, required=True)
    parser.add_argument("--artifact-name", default="nightly-summary")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--token", required=True)
    parser.add_argument("--per-page", type=int, default=20)
    parser.add_argument("--max-runs", type=int, default=1)
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        runs = list_workflow_runs(
            server_url=args.server_url,
            repository=args.repository,
            workflow=args.workflow,
            token=args.token,
            per_page=args.per_page,
        )
        previous_run_ids = find_previous_run_ids(runs, args.current_run_id, args.max_runs)
        if not previous_run_ids:
            print("previous_nightly_artifact_status=missing_run")
            return 0

        downloaded = []
        latest_dir = output_dir / "latest"
        if latest_dir.exists():
            shutil.rmtree(latest_dir)
        for index, run_id in enumerate(previous_run_ids):
            artifacts = list_run_artifacts(
                server_url=args.server_url,
                repository=args.repository,
                run_id=run_id,
                token=args.token,
            )
            artifact = next((item for item in artifacts if item.get("name") == args.artifact_name), None)
            if artifact is None:
                continue

            content = download_artifact_zip(
                server_url=args.server_url,
                repository=args.repository,
                artifact_id=int(artifact["id"]),
                token=args.token,
            )
            run_dir = output_dir / f"run_{run_id}"
            if run_dir.exists():
                shutil.rmtree(run_dir)
            extract_zip(content, run_dir)
            if index == 0:
                shutil.copytree(run_dir, latest_dir)
            downloaded.append(
                {
                    "run_id": run_id,
                    "artifact_id": int(artifact["id"]),
                    "path": str(run_dir),
                }
            )

        if not downloaded:
            print("previous_nightly_artifact_status=missing_artifact")
            return 0

        index_path = output_dir / "download_index.json"
        index_path.write_text(json.dumps({"downloads": downloaded}, indent=2) + "\n", encoding="utf-8")
        print(
            "previous_nightly_artifact_status=downloaded "
            f"count={len(downloaded)} latest_run_id={downloaded[0]['run_id']}"
        )
        return 0
    except urllib.error.HTTPError as exc:
        print(
            f"previous_nightly_artifact_status=http_error code={exc.code}",
            file=sys.stderr,
        )
        return 1
    except urllib.error.URLError as exc:
        print(
            f"previous_nightly_artifact_status=url_error reason={exc.reason}",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
