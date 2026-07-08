#!/usr/bin/env bash

set -euo pipefail

BLACK_VERSION="26.3.1"
ISORT_VERSION="8.0.1"
CLANG_FORMAT_VERSION="16.0.6"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
FORMAT_VENV="${FORMAT_VENV:-$SCRIPT_DIR/.venv-format}"
FORMAT_PYTHON="$FORMAT_VENV/bin/python"
CLANG_FORMAT="$FORMAT_VENV/bin/clang-format"
cd "$SCRIPT_DIR"

check=false
paths=()

usage() {
    cat <<'EOF'
Usage: ./format_code.sh [--check] [path ...]

Formats Python files with isort then black, and C/C++ files with clang-format.
If no paths are provided, the entire repository is formatted. Formatter
packages are installed into .venv-format on first use.

Options:
  --check     Check formatting without modifying files.
  -h, --help  Show this help.
EOF
}

formatter_versions_ok() {
    [[ -x "$FORMAT_PYTHON" ]] || return 1
    [[ -x "$CLANG_FORMAT" ]] || return 1
    "$FORMAT_PYTHON" - "$BLACK_VERSION" "$ISORT_VERSION" "$CLANG_FORMAT_VERSION" <<'PYCHECK' >/dev/null 2>&1
from importlib.metadata import version
from sys import argv

expected = {
    "black": argv[1],
    "isort": argv[2],
    "clang-format": argv[3],
}
raise SystemExit(any(version(name) != expected[name] for name in expected))
PYCHECK
}

create_formatter_venv() {
    if python3 -m venv "$FORMAT_VENV" >/dev/null 2>&1; then
        return
    fi

    if python3 -m virtualenv "$FORMAT_VENV" >/dev/null 2>&1; then
        return
    fi

    echo "Unable to create $FORMAT_VENV with python3 -m venv or python3 -m virtualenv" >&2
    exit 1
}

ensure_formatter_tools() {
    if formatter_versions_ok; then
        return
    fi

    echo "Installing formatter tools into $FORMAT_VENV"
    if [[ ! -x "$FORMAT_PYTHON" ]] || ! "$FORMAT_PYTHON" -m pip --version >/dev/null 2>&1; then
        create_formatter_venv
    fi
    if ! "$FORMAT_PYTHON" -m pip --version >/dev/null 2>&1; then
        "$FORMAT_PYTHON" -m ensurepip --upgrade >/dev/null
    fi
    "$FORMAT_PYTHON" -m pip install \
        "black==$BLACK_VERSION" \
        "isort==$ISORT_VERSION" \
        "clang-format==$CLANG_FORMAT_VERSION"
}

collect_files() {
    "$FORMAT_PYTHON" - "$@" <<'PYCOLLECT'
from pathlib import Path
from sys import argv

mode = argv[1]
raw_paths = argv[2:]
extensions_by_mode = {
    "cpp": {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".inl"},
    "python": {".py", ".pyi"},
}
extensions = extensions_by_mode[mode]
skip_parts = {
    ".git",
    ".pixi",
    ".pytest_cache",
    ".venv",
    ".venv-format",
    "__pycache__",
    "build",
    "install",
    "log",
}
root = Path.cwd().resolve()
seen = set()


def configured_submodule_paths() -> set[Path]:
    gitmodules = root / ".gitmodules"
    if not gitmodules.exists():
        return set()

    paths = set()
    for line in gitmodules.read_text().splitlines():
        key, sep, value = line.strip().partition("=")
        if sep and key.strip() == "path":
            paths.add((root / value.strip()).resolve())
    return paths


submodule_paths = configured_submodule_paths()


def is_skipped(path: Path) -> bool:
    resolved = path.resolve()
    try:
        rel = resolved.relative_to(root)
    except ValueError:
        rel = resolved
    return (
        "build_ws" in rel.parts
        or any(part in skip_parts for part in rel.parts)
        or any(resolved == submodule_path or resolved.is_relative_to(submodule_path) for submodule_path in submodule_paths)
    )


def emit(path: Path) -> None:
    if path.suffix not in extensions or is_skipped(path):
        return
    resolved = path.resolve()
    if resolved in seen:
        return
    seen.add(resolved)
    print(path)


for raw_path in raw_paths:
    path = Path(raw_path)
    if not path.exists() or is_skipped(path):
        continue
    if path.is_file():
        emit(path)
    elif path.is_dir():
        for child in path.rglob("*"):
            if child.is_file():
                emit(child)
PYCOLLECT
}

collect_cpp_files() {
    collect_files cpp "$@"
}

collect_python_targets() {
    collect_files python "$@"
}

run_python_format() {
    mapfile -t python_targets < <(collect_python_targets "${paths[@]}")
    if [[ ${#python_targets[@]} -eq 0 ]]; then
        return
    fi

    if [[ "$check" == true ]]; then
        local status=0
        "$FORMAT_PYTHON" -m isort --check-only --diff --settings-path "$SCRIPT_DIR" "${python_targets[@]}" || status=1
        "$FORMAT_PYTHON" -m black --check --diff --config "$SCRIPT_DIR/pyproject.toml" "${python_targets[@]}" || status=1
        return "$status"
    else
        "$FORMAT_PYTHON" -m isort --settings-path "$SCRIPT_DIR" "${python_targets[@]}"
        "$FORMAT_PYTHON" -m black --config "$SCRIPT_DIR/pyproject.toml" "${python_targets[@]}"
    fi
}


run_clang_format() {
    mapfile -t cpp_files < <(collect_cpp_files "${paths[@]}")
    if [[ ${#cpp_files[@]} -eq 0 ]]; then
        return
    fi

    if [[ "$check" == true ]]; then
        "$CLANG_FORMAT" --dry-run --Werror --style=file "${cpp_files[@]}"
    else
        "$CLANG_FORMAT" -i --style=file "${cpp_files[@]}"
    fi
}

run_formatters_in_venv() {
    (
        # Keep activation scoped away from the caller, even when sourced.
        source "$FORMAT_VENV/bin/activate"
        trap deactivate EXIT
        if [[ "$check" == true ]]; then
            local status=0
            run_python_format || status=1
            run_clang_format || status=1
            exit "$status"
        fi
        run_python_format
        run_clang_format
        deactivate
        trap - EXIT
    )
}

for arg in "$@"; do
    case "$arg" in
        --check)
            check=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            paths+=("$arg")
            ;;
    esac
done

if [[ ${#paths[@]} -eq 0 ]]; then
    paths=(".")
fi

ensure_formatter_tools
run_formatters_in_venv
