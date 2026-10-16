#!/usr/bin/env bash
set -euo pipefail
source examples/libero_reference/env/activate.sh
export XDG_CACHE_HOME="$PWD/.reference-deps/cache"
mkdir -p "$XDG_CACHE_HOME"
python - <<'PY'
import json, os, subprocess
from pathlib import Path
root=Path.cwd()
masked=['/data_robot/projects','/tmp']
checks={p: {'exists':Path(p).exists(),'directory_empty':not any(Path(p).iterdir())} for p in masked}
assert all(v['directory_empty'] for v in checks.values()), checks
assert not Path('.git/objects/info/alternates').exists()
Path('runs/isolation.json').write_text(json.dumps({'checkout':str(root),'commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'masked_roots':checks,'git_object_alternates':False,'pythonpath':os.environ.get('PYTHONPATH'),'cwd_outside_masked_roots':True},indent=2)+'\n')
PY
strace -f -e trace=file -o runs/file-access.log python -u examples/libero_reference/run.py --output runs/acceptance_001
python -m unittest discover -s examples/libero_reference -p 'test_*.py' -v > runs/reference-tests.log 2>&1
python -m unittest discover -s tests -v > runs/static-tests.log 2>&1
python examples/libero_reference/env/test_provenance.py > runs/environment-tests.log 2>&1
python -m pip check > runs/pip-check.log 2>&1
sha256sum --check --status SHA256SUMS
python examples/libero_reference/render_video.py --run runs/acceptance_001
