"""Read-only Python environment inventory; does not import project modules."""

import argparse
from datetime import datetime, timezone
from importlib import metadata
import json
from pathlib import Path
import platform
import sys


def environment_report(expected_env):
    packages = {}
    for name in ("numpy", "matplotlib", "Pillow"):
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            packages[name] = None
    environment_name = Path(sys.prefix).name
    return {
        "checked_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "environment_name": environment_name,
        "expected_environment": expected_env,
        "environment_matches": environment_name == expected_env,
        "platform": platform.platform(),
        "packages": packages,
        "missing_packages": [name for name, version in packages.items()
                             if version is None],
        "scope": "Environment inventory only; no simulation or algorithm validation.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-env", default="sca_py38")
    parser.add_argument("--output", type=Path,
                        help="Optional new JSON file; existing files are preserved.")
    args = parser.parse_args()
    report = environment_report(args.expected_env)
    serialized = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    print(serialized)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(serialized + "\n")
    return 0 if report["environment_matches"] and not report["missing_packages"] else 1


if __name__ == "__main__":
    sys.exit(main())
