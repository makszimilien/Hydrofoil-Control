"""Run current R5.5 host checks; never contacts a board."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
root = Path(__file__).resolve().parents[1]
node = os.environ.get('NODE_BINARY') or shutil.which('node')
if not node:
    raise SystemExit('Node.js is required (or set NODE_BINARY to its executable).')
commands = [[sys.executable, str(root/'tests'/name)] for name in
            ('run_r55_shared_tests.py', 'run_r55_remote_setup_tests.py', 'run_r55_center_tests.py')]
commands += [[sys.executable, str(root/'tests/run_r55_regression_tests.py'), '--only', name]
             for name in ('quickpid', 'filesystem', 'validation', 'receiver', 'modes', 'task')]
commands += [[sys.executable, str(root/'tests/run_standalone_tests.py')],
             [node, str(root/'tests/run_r55_ui_tests.mjs')]]
for command in commands:
    print('\nRUN', ' '.join(Path(x).name if '/' in x else x for x in command), flush=True)
    subprocess.run(command, cwd=root, check=True)
print('\nPASS all R5.5 host checks; hardware acceptance remains separate.')
