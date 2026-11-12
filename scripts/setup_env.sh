#!/usr/bin/env bash
# Creates the "hexapod" conda environment. Run it as your normal user, not with sudo.
#
#   bash scripts/setup_env.sh            install exactly what the lock file lists
#   bash scripts/setup_env.sh --relock   solve environment.yml again and rewrite the lock file
#
# Both files live in hexapod/, next to the code they install. The lock file
# (conda-<system>-<arch>.lock) pins every package to an exact build, so every machine of the
# same kind gets an identical environment. Commit it.
# To add a package: edit hexapod/environment.yml, run with --relock, commit both files.
set -euo pipefail

SOFTWARE="$(cd "$(dirname "$0")/../hexapod" && pwd)"
CONDA_DIR="$HOME/miniforge3"
ENV_NAME=hexapod
LOCK="$SOFTWARE/conda-$(uname -s)-$(uname -m).lock"

if [ "$(id -u)" -eq 0 ]; then
    echo "Run it without sudo: the environment lives in your home folder." >&2
    exit 1
fi
case "$(uname -m)" in
    aarch64 | x86_64) ;;
    *) echo "conda needs a 64-bit OS (this one is $(uname -m))." >&2; exit 1 ;;
esac

# 1. Miniforge (conda, set up for the conda-forge channel), installed once in your home.
if [ ! -x "$CONDA_DIR/bin/conda" ]; then
    echo "Installing Miniforge in $CONDA_DIR"
    installer="$(mktemp --suffix=.sh)"   # the installer refuses names not ending in .sh
    curl -fsSL -o "$installer" \
        "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname -s)-$(uname -m).sh"
    bash "$installer" -b -u -p "$CONDA_DIR"   # -u: also completes an interrupted install
    rm "$installer"
    "$CONDA_DIR/bin/conda" init bash   # makes `conda activate` work in new shells
fi
conda="$CONDA_DIR/bin/conda"

# 2. The environment, rebuilt from scratch so nothing outside the spec survives.
if [ -d "$CONDA_DIR/envs/$ENV_NAME" ]; then
    "$conda" remove --yes --name "$ENV_NAME" --all
fi
if [ -f "$LOCK" ] && [ "${1:-}" != "--relock" ]; then
    echo "Creating $ENV_NAME from $(basename "$LOCK")"
    "$conda" create --yes --name "$ENV_NAME" --file "$LOCK"
else
    echo "Solving environment.yml, then writing $(basename "$LOCK")"
    "$conda" env create --file "$SOFTWARE/environment.yml"
    "$conda" list --name "$ENV_NAME" --explicit --md5 > "$LOCK"
fi

# 3. This project's package, editable: `import hexapod` always sees your latest code.
"$CONDA_DIR/envs/$ENV_NAME/bin/python" -m pip install --quiet --no-deps --no-build-isolation -e "$SOFTWARE"

echo "Done. Open a new shell, then: conda activate $ENV_NAME"
