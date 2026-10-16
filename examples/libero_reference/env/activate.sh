# Source this file from bash. All paths derive from this fresh worktree.
REF_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
export LIBERO_CONFIG_PATH="$REF_ROOT/.reference-deps/config"
export MUJOCO_GL=egl
export MPLCONFIGDIR="$REF_ROOT/.reference-deps/matplotlib"
export NUMBA_CACHE_DIR="$REF_ROOT/.reference-deps/numba-cache"
source "$REF_ROOT/.reference-env/bin/activate"
unset REF_ROOT
