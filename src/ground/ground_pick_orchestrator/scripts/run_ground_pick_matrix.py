#!/usr/bin/env python3
"""Run isolated V0.3 Step 3 Gazebo trials and write auditable results."""
import datetime
import hashlib
import json
import os
import pathlib
import subprocess
import sys

import yaml

WORKSPACE = pathlib.Path(__file__).resolve().parents[4]
PACKAGE = WORKSPACE / 'src/bunker_aubo_project/ground_pick_orchestrator'
sys.path.insert(0, str(PACKAGE / 'src'))
from ground_pick_orchestrator.mission_metrics import (acceptance_passed,
                                                       summarize_trials)


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.host.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write('\n')
    temporary.replace(path)


def tree_hash():
    digest = hashlib.sha256()
    for path in sorted((WORKSPACE / 'src/bunker_aubo_project').rglob('*')):
        if path.is_file() and not path.name.endswith(('.pyc', '.log')):
            digest.update(str(path.relative_to(WORKSPACE)).encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def run_trial(item, output):
    result = output / (item['id'] + '.json')
    log = output / (item['id'] + '.log')
    if result.exists():
        result.unlink()
    args = {
        'scenario_id': item['id'], 'brick_x': item['brick_x'],
        'brick_y': item['brick_y'], 'brick_yaw': item['brick_yaw'],
        'known_x': item.get('known_x', item['brick_x']),
        'known_y': item.get('known_y', item['brick_y']),
        'known_yaw': item.get('known_yaw', item['brick_yaw']),
        'expect_success': str(item['expect_success']).lower(),
        'expected_failure': item.get('expected_failure', ''),
        'result_file': '/ws/' + str(result.relative_to(WORKSPACE)),
        'pre_grasp_height': 5.0 if item.get('force_planning_failure') else 0.15,
        'small_obstacle': str(item.get('small_obstacle', False)).lower(),
        'obstacle_x': item.get('obstacle_x', 0.0),
        'obstacle_y': item.get('obstacle_y', 0.0),
    }
    launch_args = ' '.join('{}:={}'.format(k, v) for k, v in args.items())
    name = 'ground_pick_matrix_{}_{}'.format(os.getpid(), item['id'])
    command = ['docker', 'run', '--rm', '--name', name, '--gpus', 'all',
               '--network=host', '--ipc=host', '-e', 'DISPLAY=' +
               os.environ.get('DISPLAY', ':0'), '-e', 'QT_X11_NO_MITSHM=1',
               '-e', 'NVIDIA_DRIVER_CAPABILITIES=all', '-v',
               '/tmp/.X11-unix:/tmp/.X11-unix:rw', '-v', str(WORKSPACE) +
               ':/ws', '-w', '/ws', 'bunker-aubo:melodic', 'bash', '-lc',
               'source /opt/ros/melodic/setup.bash; source /ws/vendor_ws/devel/setup.bash; '
               'source /ws/devel/setup.bash; roslaunch ground_pick_orchestrator '
               'ground_pick_trial.launch ' + launch_args]
    with log.open('w') as stream:
        process = subprocess.Popen(command, stdout=stream,
                                   stderr=subprocess.STDOUT)
        try:
            code = process.wait(timeout=190)
        except subprocess.TimeoutExpired:
            subprocess.call(['docker', 'stop', '-t', '5', name],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
            process.wait(timeout=15)
            code = 124
    data = json.loads(result.read_text()) if result.exists() else {
        'scenario_id': item['id'], 'expect_success': item['expect_success'],
        'expected_failure': item.get('expected_failure', ''),
        'failure_type': 'harness_failed', 'safe_stop': False}
    data['container_exit_code'] = code
    write_json(result, data)
    return data


def main():
    matrix = yaml.safe_load((PACKAGE / 'config/mission_scenarios.yaml').read_text())
    run_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    output = WORKSPACE / 'docs/results/ground_pick_v03_step3' / run_id
    output.mkdir(parents=True)
    results = []
    for index, item in enumerate(matrix['scenarios'], 1):
        print('[{}/{}] {}'.format(index, len(matrix['scenarios']), item['id']),
              flush=True)
        results.append(run_trial(item, output))
        print(json.dumps(results[-1], sort_keys=True), flush=True)
    summary = summarize_trials(results)
    valid = sum(item['expect_success'] for item in matrix['scenarios'])
    controls = len(matrix['scenarios']) - valid
    passed = acceptance_passed(summary, 10, controls)
    artifact = {'run_id': run_id, 'summary': summary, 'trials': results,
                'acceptance_passed': passed, 'provenance': {
                    'git_head': subprocess.check_output(
                        ['git', 'rev-parse', 'HEAD'], cwd=str(WORKSPACE),
                        text=True).strip(),
                    'git_status': subprocess.check_output(
                        ['git', 'status', '--short'], cwd=str(WORKSPACE),
                        text=True).splitlines(),
                    'source_tree_sha256': tree_hash(),
                    'docker_image_id': subprocess.check_output(
                        ['docker', 'image', 'inspect', 'bunker-aubo:melodic',
                         '--format', '{{.Id}}'], text=True).strip(),
                    'scenario_config': matrix}}
    write_json(output / 'summary.json', artifact)
    print('results:', output, flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
