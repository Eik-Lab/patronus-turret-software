#!/usr/bin/env python3
import argparse
import os

from dotenv import load_dotenv
from roboflow import Roboflow


def download(workspace, project, version, output, api_key, fmt="coco"):
    rf = Roboflow(api_key=api_key)
    project_handle = rf.workspace(workspace).project(project)
    dataset = project_handle.version(version).download(fmt, location=output)
    print(f"downloaded to {dataset.location}")
    return dataset.location


def main():
    load_dotenv()

    parser = argparse.ArgumentParser(description="Download a Roboflow COCO dataset")
    parser.add_argument("--workspace", required=True, help="Roboflow workspace id")
    parser.add_argument("--project", required=True, help="Roboflow project id")
    parser.add_argument("--version", type=int, required=True, help="dataset version")
    parser.add_argument("--output", required=True, help="download location")
    parser.add_argument("--format", default="coco", help="export format (default: coco)")
    args = parser.parse_args()

    api_key = os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        parser.error("set ROBOFLOW_API_KEY in your .env (see .env.example)")

    download(
        args.workspace,
        args.project,
        args.version,
        args.output,
        api_key,
        args.format,
    )


if __name__ == "__main__":
    main()
