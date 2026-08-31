#!/usr/bin/env python3
"""Run fresh isolated Gazebo/move_base trials for all approach poses."""
import argparse
import datetime
import hashlib
import json
import math
import os
import pathlib
import subprocess
import sys
import time

import yaml

MINIMUM_TRIALS = 14
PACKAGE = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE / 'src'))
from bunker_navigation.metrics import summarize_trials  # noqa: E402


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write('\n')
    temporary.replace(path)


def source_tree_hash():
    digest = hashlib.sha256()
    paths = [PACKAGE, WORKSPACE / 'docker/Dockerfile',
             WORKSPACE / 'src/bunker_aubo_project/bunker_aubo_description/urdf/bunker_aubo.urdf.xacro',
             WORKSPACE / 'src/bunker_aubo_project/bunker_aubo_gazebo/launch/combined_robot.launch']
    files = []
    for path in paths:
        files.extend(path.rglob('*') if path.is_dir() else [path])
    for path in sorted(item for item in files if item.is_file() and
                       item.suffix not in ('.pyc', '.log')):
        digest.update(str(path.relative_to(WORKSPACE)).encode('utf-8'))
        digest.update(path.read_bytes())
    return digest.hexdigest()


def run_trial(scenario, output, timeout):
    result_path = output / (scenario['id'] + '.json')
    log_path = output / (scenario['id'] + '.log')
    if result_path.exists():
        result_path.unlink()
    container = 'bunker_nav_{}_{}'.format(os.getpid(), scenario['id'])
    relative = result_path.relative_to(WORKSPACE)
    command = [
        'docker', 'run', '--rm', '--name', container, '--network=host',
        '-v', '{}:/ws'.format(WORKSPACE), '-w', '/ws',
        'bunker-aubo:melodic', 'bash', '-lc',
        ('source /opt/ros/melodic/setup.bash; '
         'source /ws/vendor_ws/devel/setup.bash; source /ws/devel/setup.bash; '
         'roslaunch bunker_navigation navigation_trial.launch '
         'scenario_id:={sid} goal_x:={x} goal_y:={y} goal_yaw:={yaw} '
         'expect_heading_gate:={expect_gate} '
         'timeout:={timeout} result_file:=/ws/{result}').format(
             sid=scenario['id'], x=scenario['x'], y=scenario['y'],
             yaw=scenario['yaw'],
             expect_gate=str(bool(scenario.get(
                 'expect_heading_gate', False))).lower(),
             timeout=max(10, timeout - 15),
             result=relative)]
    started = time.time()
    with log_path.open('w') as stream:
        process = subprocess.Popen(command, stdout=stream,
                                   stderr=subprocess.STDOUT)
        try:
            exit_code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            subprocess.run(['docker', 'stop', '-t', '5', container],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, check=False)
            process.wait(timeout=15)
            exit_code = 124
    result = None
    if exit_code == 0 and result_path.exists():
        with result_path.open() as stream:
            candidate = json.load(stream)
        if candidate.get('scenario_id') == scenario['id']:
            result = candidate
    if result is None:
        result = {'scenario_id': scenario['id'], 'success': False,
                  'failure_type': 'harness_failed',
                  'failure_detail': 'container exit {}'.format(exit_code),
                  'xy_error': 1e9, 'yaw_error': 1e9,
                  'duration': time.time() - started}
    result['container_exit_code'] = exit_code
    write_json(result_path, result)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=100.0)
    parser.add_argument('--output-dir')
    args = parser.parse_args()
    with (PACKAGE / 'config/navigation_scenarios.yaml').open() as stream:
        matrix = yaml.safe_load(stream)
    scenarios = matrix['scenarios']
    if len(scenarios) < MINIMUM_TRIALS:
        raise RuntimeError('at least {} scenarios required'.format(
            MINIMUM_TRIALS))
    subprocess.check_call([
        'docker', 'run', '--rm', '-v', '{}:/ws'.format(WORKSPACE),
        '-w', '/ws', 'bunker-aubo:melodic', 'bash', '-lc',
        './scripts/build_workspace.sh'])
    run_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    output = pathlib.Path(args.output_dir) if args.output_dir else (
        WORKSPACE / 'docs/results/bunker_navigation' / run_id)
    output.mkdir(parents=True, exist_ok=True)
    trials = []
    for index, scenario in enumerate(scenarios, 1):
        print('[{}/{}] {}'.format(index, len(scenarios), scenario['id']),
              flush=True)
        result = run_trial(scenario, output, args.timeout)
        trials.append(result)
        print('  success={} xy={:.4f}m yaw={:.3f}deg t={:.1f}s'.format(
            result['success'], result['xy_error'],
            math.degrees(result['yaw_error']), result['duration']), flush=True)
    summary = summarize_trials(trials)
    artifact = {'run_id': run_id, 'trials': trials, 'summary': summary,
                'scenario_config': matrix,
                'git_head': subprocess.check_output(
                    ['git', 'rev-parse', 'HEAD'], cwd=str(WORKSPACE),
                    text=True).strip(),
                'git_status': subprocess.check_output(
                    ['git', 'status', '--short'], cwd=str(WORKSPACE),
                    text=True).splitlines(),
                'source_tree_sha256': source_tree_hash(),
                'command': str(PACKAGE / 'scripts/run_navigation_matrix.py'),
                'docker_image_id': subprocess.check_output(
                    ['docker', 'image', 'inspect', 'bunker-aubo:melodic',
                     '--format', '{{.Id}}'], text=True).strip()}
    write_json(output / 'summary.json', artifact)
    print('results: {}'.format(output), flush=True)
    return 0 if (summary['trial_count'] >= MINIMUM_TRIALS and
                 summary['success_count'] == summary['trial_count']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
