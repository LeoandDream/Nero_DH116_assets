#!/usr/bin/env bash
# Rebuild the isolated, runtime-only LIBERO reference environment. Linux x86_64, curl/git required.
set -euo pipefail
REF_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
REF_ENV_FILES="$REF_ROOT/examples/libero_reference/env"
REF_DEPS="$REF_ROOT/.reference-deps"
unset PYTHONPATH PYTHONHOME PIP_INDEX_URL PIP_EXTRA_INDEX_URL PIP_FIND_LINKS PIP_TARGET PIP_PREFIX PIP_USER
export PIP_CONFIG_FILE=/dev/null
export PYTHONNOUSERSITE=1 PIP_DISABLE_PIP_VERSION_CHECK=1
export PIP_CACHE_DIR="$REF_DEPS/pip-cache"
mkdir -p "$REF_DEPS/downloads" "$REF_DEPS/config"
REF_PYTHON_ARCHIVE="$REF_DEPS/downloads/python.tar.gz"
REF_PYTHON_URL='https://github.com/astral-sh/python-build-standalone/releases/download/20260924/cpython-3.11.16%2B20260924-x86_64-unknown-linux-gnu-install_only_stripped.tar.gz'
if [[ ! -f "$REF_PYTHON_ARCHIVE" ]]; then
  curl -fL --retry 3 "$REF_PYTHON_URL" -o "$REF_PYTHON_ARCHIVE"
fi
printf '%s  %s\n' '68c6739376b65258dee5058ccf6777232fe38d31a578965ae8bda327ec7da3a8' "$REF_PYTHON_ARCHIVE" | sha256sum --check --status
if [[ ! -x "$REF_DEPS/python/bin/python3.11" ]]; then
  tar -xzf "$REF_PYTHON_ARCHIVE" -C "$REF_DEPS"
fi
printf '%s  %s\n' 'ec579de7512e84d960bc642a840de69c785c52d6adf89e3a15f51717b29be40f' "$REF_DEPS/python/bin/python3.11" | sha256sum --check --status
if [[ ! -x "$REF_ROOT/.reference-env/bin/python" ]]; then
  "$REF_DEPS/python/bin/python3.11" -m venv "$REF_ROOT/.reference-env"
fi
REF_PY="$REF_ROOT/.reference-env/bin/python"
mkdir -p "$REF_DEPS/downloads/build" "$REF_DEPS/downloads/packages"
"$REF_PY" -m pip download --index-url https://pypi.org/simple --no-deps --require-hashes --dest "$REF_DEPS/downloads/build" -r "$REF_ENV_FILES/build.lock"
"$REF_PY" -m pip install --no-index --find-links "$REF_DEPS/downloads/build" --no-deps --require-hashes -r "$REF_ENV_FILES/build.lock"
"$REF_PY" -m pip download --index-url https://pypi.org/simple --no-deps --no-build-isolation --require-hashes --dest "$REF_DEPS/downloads/packages" -r "$REF_ENV_FILES/requirements.lock"
"$REF_PY" -m pip install --no-index --find-links "$REF_DEPS/downloads/packages" --no-deps --no-build-isolation --require-hashes -r "$REF_ENV_FILES/requirements.lock"
REF_LIBERO="$REF_DEPS/LIBERO"
if [[ ! -d "$REF_LIBERO/.git" ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/Lifelong-Robot-Learning/LIBERO.git "$REF_LIBERO"
fi
git -C "$REF_LIBERO" checkout --detach f78abd68ee283de9f9be3c8f7e2a9ad60246e95c
git -C "$REF_LIBERO" diff --exit-code HEAD
(cd "$REF_LIBERO" && sha256sum --check --status "$REF_ENV_FILES/libero-source.sha256")
# Editable installation preserves package assets; imports point only into this pinned clone.
"$REF_PY" -m pip install --no-deps --no-build-isolation -e "$REF_LIBERO"
mkdir -p "$REF_LIBERO/datasets"
export LIBERO_CONFIG_PATH="$REF_DEPS/config"
"$REF_PY" - "$REF_LIBERO" "$LIBERO_CONFIG_PATH/config.yaml" <<'PY'
from pathlib import Path
import sys
import yaml
source = Path(sys.argv[1]).resolve()
package = source / 'libero/libero'
paths = {'benchmark_root': package, 'bddl_files': package / 'bddl_files',
         'assets': package / 'assets', 'init_states': package / 'init_files',
         'datasets': source / 'datasets'}
Path(sys.argv[2]).write_text(yaml.safe_dump({key: str(value) for key, value in paths.items()}))
PY
source "$REF_ENV_FILES/activate.sh"
"$REF_PY" "$REF_ENV_FILES/test_provenance.py"
"$REF_PY" "$REF_ENV_FILES/provenance.py" --output "$REF_DEPS/environment.json"
"$REF_PY" -m pip check
