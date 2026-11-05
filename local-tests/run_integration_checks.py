#!/usr/bin/env python3
"""Build and run hardware-free merge regressions with the real SDK/FTCLib/Pedro.

Requires JAVA_HOME and an Android SDK configured for Gradle. No robot connection.
Failures remain failures, including any inherited parameter-validation failure.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('checks', nargs='*', help='Check class names; defaults to the merge regression set')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = root / 'build/host-checks'
    classes = out / 'classes'
    classes.mkdir(parents=True, exist_ok=True)
    jdk = Path(os.environ['JAVA_HOME']) / 'bin'
    gradle = [str(root / 'gradlew'), '--no-daemon', '--console=plain']
    if args.offline:
        gradle.append('--offline')
    gradle += ['-I', str(root / 'local-tests/host-classpath.init.gradle'),
               ':TeamCode:assembleDebug', ':TeamCode:exportHostCheckClasspath']
    with (out / 'build.log').open('w') as log:
        subprocess.run(gradle, cwd=root, stdout=log, stderr=subprocess.STDOUT, check=True)
    cp = []
    for line in (out / 'classpath.txt').read_text().splitlines():
        path = Path(line)
        if path.name.endswith('-api.jar'):
            cache = path.parents[2]
            matches = sorted(cache.glob('*/transformed/' + path.name.replace('-api.jar', '-runtime.jar')))
            if not matches:
                raise RuntimeError('Missing runtime JAR: ' + str(path))
            path = matches[0]
        cp.append(str(path))
    classpath = os.pathsep.join([str(classes)] + cp)
    sources = sorted((root / 'local-tests').glob('*.java'))
    compiled = subprocess.run([str(jdk / 'javac'), '-encoding', 'UTF-8', '-cp', classpath,
                               '-d', str(classes)] + [str(p) for p in sources],
                              capture_output=True, text=True)
    (out / 'compile.log').write_text(compiled.stdout + compiled.stderr)
    if compiled.returncode:
        print(compiled.stderr)
        return compiled.returncode
    names = args.checks or ['AutoAimPidfCheck', 'ShooterDistanceCheck', 'PracticeBlueCheck',
                           'RtCalibrationCheck', 'HiveGeometryCheck', 'TestAutoCheck',
                           'PedroTuningCheck', 'SharedShootMergeCheck', 'SharedAutoAimCheck']
    results = []
    for name in names:
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9]*', name):
            raise ValueError('Invalid check name: ' + name)
        source = root / 'local-tests' / (name + '.java')
        package = re.search(r'^package (.*);', source.read_text(), re.MULTILINE).group(1)
        try:
            run = subprocess.run([str(jdk / 'java'), '-cp', classpath, package + '.' + name],
                                 cwd=root, capture_output=True, text=True, timeout=60)
            code, output = run.returncode, run.stdout + run.stderr
        except subprocess.TimeoutExpired:
            code, output = 124, 'Host check exceeded 60 seconds.\n'
        (out / (name + '.log')).write_text(output)
        print(('PASS ' if code == 0 else 'FAIL ') + name)
        print(output.strip())
        results.append({'check': name, 'exit_code': code})
    (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    return int(any(r['exit_code'] for r in results))


if __name__ == '__main__':
    sys.exit(main())
