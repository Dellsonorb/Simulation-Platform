#!/usr/bin/env python3
"""Run isolated known-brick approach generation/navigation Gazebo trials."""
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


MINIMUM_TRIALS = 10
PACKAGE = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE / 'src'))
from bunker_navigation.approach_metrics import summarize_approach_trials  # noqa: E402


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write('\n')
    temporary.replace(path)


def source_tree_hash():
    digest = hashlib.sha256()
    paths = [
        PACKAGE,
        WORKSPACE / 'docker/Dockerfile',
        WORKSPACE / 'src/bunker_aubo_project/bunker_aubo_description/urdf/bunker_aubo.urdf.xacro',
        WORKSPACE / 'src/bunker_aubo_project/bunker_aubo_gazebo/launch/combined_robot.launch',
    ]
    files = []
    for path in paths:
        files.extend(path.rglob('*') if path.is_dir() else [path])
    for path in sorted(item for item in files if item.is_file() and
                       item.suffix not in ('.pyc', '.log')):
        digest.update(str(path.relative_to(WORKSPACE)).encode('utf-8'))
        digest.update(path.read_bytes())
    return digest.hexdigest()


def boolean(value):
    return str(bool(value)).lower()


def run_trial(scenario, output, timeout):
    result_path = output / (scenario['id'] + '.json')
    log_path = output / (scenario['id'] + '.log')
    if result_path.exists():
        result_path.unlink()
    container = 'bunker_approach_{}_{}'.format(os.getpid(), scenario['id'])
    relative = result_path.relative_to(WORKSPACE)
    launch = (
        'roslaunch bunker_navigation approach_pose_trial.launch '
        'scenario_id:={sid} brick_x:={x} brick_y:={y} brick_yaw:={yaw} '
        'expect_generation:={expected} small_obstacle:={small} '
        'obstacle_x:={ox} obstacle_y:={oy} '
        'timeout:={runtime} result_file:=/ws/{result}').format(
            sid=scenario['id'], x=scenario['brick_x'],
            y=scenario['brick_y'], yaw=scenario['brick_yaw'],
            expected=boolean(scenario['expect_generation']),
            small=boolean(scenario.get('small_obstacle', False)),
            ox=scenario.get('obstacle_x', 0.0),
            oy=scenario.get('obstacle_y', 0.0),
            runtime=max(20, timeout - 20), result=relative)
    command = [
        'docker', 'run', '--rm', '--name', container, '--network=host',
        '-v', '{}:/ws'.format(WORKSPACE), '-w', '/ws',
        'bunker-aubo:melodic', 'bash', '-lc',
        ('source /opt/ros/melodic/setup.bash; '
         'source /ws/vendor_ws/devel/setup.bash; '
         'source /ws/devel/setup.bash; ' + launch)]
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
    if result_path.exists():
        with result_path.open() as stream:
            candidate = json.load(stream)
        if candidate.get('scenario_id') == scenario['id']:
            result = candidate
    if result is None:
        result = {
            'scenario_id': scenario['id'],
            'expect_generation': bool(scenario['expect_generation']),
            'generation_success': False,
            'navigation_success': False,
            'success': False,
            'failure_type': 'harness_failed',
            'failure_detail': 'container exit {}'.format(exit_code),
            'distance_error': None,
            'facing_error': None,
            'duration': time.time() - started,
        }
    result['container_exit_code'] = exit_code
    write_json(result_path, result)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=125.0)
    parser.add_argument('--output-dir')
    args = parser.parse_args()
    with (PACKAGE / 'config/approach_scenarios.yaml').open() as stream:
        scenario_config = yaml.safe_load(stream)
    scenarios = scenario_config['scenarios']
    if len(scenarios) < MINIMUM_TRIALS:
        raise RuntimeError('at least {} scenarios required'.format(
            MINIMUM_TRIALS))
    subprocess.check_call([
        'docker', 'run', '--rm', '-v', '{}:/ws'.format(WORKSPACE),
        '-w', '/ws', 'bunker-aubo:melodic', 'bash', '-lc',
        './scripts/build_workspace.sh'])
    run_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    output = pathlib.Path(args.output_dir) if args.output_dir else (
        WORKSPACE / 'docs/results/approach_pose' / run_id)
    output.mkdir(parents=True, exist_ok=True)
    trials = []
    for index, scenario in enumerate(scenarios, 1):
        print('[{}/{}] {}'.format(index, len(scenarios), scenario['id']),
              flush=True)
        result = run_trial(scenario, output, args.timeout)
        trials.append(result)
        print('  success={} generated={} navigation={} distance={}m '
              'facing={}deg'.format(
                  result['success'], result['generation_success'],
                  result['navigation_success'], result.get('distance_error'),
                  (math.degrees(result['facing_error'])
                   if result.get('facing_error') is not None else None)),
              flush=True)
    summary = summarize_approach_trials(trials)
    artifact = {
        'run_id': run_id,
        'trials': trials,
        'summary': summary,
        'scenario_config': scenario_config,
        'git_head': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=str(WORKSPACE),
            text=True).strip(),
        'git_status': subprocess.check_output(
            ['git', 'status', '--short'], cwd=str(WORKSPACE),
            text=True).splitlines(),
        'source_tree_sha256': source_tree_hash(),
        'command': str(PACKAGE / 'scripts/run_approach_matrix.py'),
        'docker_image_id': subprocess.check_output(
            ['docker', 'image', 'inspect', 'bunker-aubo:melodic',
             '--format', '{{.Id}}'], text=True).strip(),
    }
    write_json(output / 'summary.json', artifact)
    print('results: {}'.format(output), flush=True)
    passed = (
        len(trials) >= MINIMUM_TRIALS and
        all(item.get('success') and item.get('container_exit_code') == 0
            for item in trials))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
