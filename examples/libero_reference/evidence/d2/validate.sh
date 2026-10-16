#!/usr/bin/env bash
set -euo pipefail
source examples/libero_reference/env/activate.sh
export XDG_CACHE_HOME="$PWD/.reference-deps/cache"
python - <<'PY'
import json,subprocess
from pathlib import Path
masked={p:not any(Path(p).iterdir()) for p in ('/data_robot/projects','/tmp')}
assert all(masked.values())
Path('runs/d2/isolation.json').write_text(json.dumps({'checkout':str(Path.cwd()),'code_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'masked_roots_empty':masked},indent=2)+'\n')
PY
python -m unittest discover -s examples/libero_reference -p 'test_*.py' -v > runs/d2/reference-tests.log 2>&1
python -m unittest discover -s tests -v > runs/d2/static-tests.log 2>&1
python examples/libero_reference/env/test_provenance.py > runs/d2/environment-tests.log 2>&1
python examples/libero_reference/env/provenance.py --output runs/d2/environment.json > runs/d2/environment-check.log 2>&1
python -m pip check > runs/d2/pip-check.log 2>&1
sha256sum --check --status SHA256SUMS
set +e
python -u examples/libero_reference/run.py --output runs/d2/reference_001
reference_exit=$?
set -e
python - "$reference_exit" <<'PY'
import json,sys
from pathlib import Path
exit_code=int(sys.argv[1])
r=json.loads(Path('runs/d2/reference_001/run-result.json').read_text())
Path('runs/d2/execution.json').write_text(json.dumps({'command':'python -u examples/libero_reference/run.py --output runs/d2/reference_001','exit_code':exit_code,'run_result':r},indent=2)+'\n')
assert exit_code==0,exit_code
assert r['status']=='EXACT_REFERENCE_REPRODUCED'
assert (r['commands'],r['physics_steps'])==(230,5750)
assert r['first_difference'] is None
assert r['historical_trajectory_equal'] and r['raw_jsonl_bytes_equal']
assert r['trace_sha256']==r['reference_trace_sha256']=='880581b82882313c4abc4c9c838402be6ed04eac18dd5c30ee3780258a27fafa'
PY
