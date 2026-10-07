from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import json
import subprocess

sim = Path('/home/th1rt3en/dev/forge/duojin02-sim')
hardware = sim.with_name('duojin02')
cache = sim / '.cache/navigation-narrow'
image = 'duojin02:sim-humble-harmonic'
common = 'source /opt/ros/humble/setup.bash; source /opt/duojin01/slam_backend/setup.bash; '
cases = 'open corridor_0.50 thin_obstacles s_bend dynamic_crossing sensor_dropout initial_pose_error online_thin online_open office_chicane online_chicane reverse_goal'


def create(name, domain, previous=False):
    command = ['docker', 'run', '-d', '--rm', '--init', '--gpus', 'all', '--shm-size=1g', '--name', name,
               '-e', 'NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute', '-e', f'ROS_DOMAIN_ID={domain}',
               '-e', f'GZ_PARTITION={name}', '-v', f'{cache / "previous-hardware" if previous else hardware}:/hardware:ro',
               '-v', f'{sim}:/ws:ro', '-v', f'{cache}:/evidence']
    if previous:
        command += ['-v', f'{cache / "previous-simulation-src"}:/ws/simulation/src:ro']
    command += ['--entrypoint', 'bash', image, '-c', 'exec tail -f /dev/null']
    subprocess.run(command, check=True, stdout=subprocess.DEVNULL)


def execute(name, script, log):
    with (cache / log).open('w') as output:
        subprocess.run(['docker', 'exec', name, 'bash', '-lc', 'set -e; ' + common + script],
                       stdout=output, stderr=subprocess.STDOUT, check=True)


def suite(root, seeds, selected):
    return f'python3 /ws/simulation/nav_benchmark_suite.py --output /evidence/{root} --variants upgraded --seeds {seeds} --cases {selected}; '


def worker(seed):
    name = f'duojin02-narrow-{seed}'
    create(name, 102 + seed - 20)
    execute(name, 'cd /tmp; tar -xzf /evidence/optimized-build.tar.gz; source /tmp/navbuild/install/setup.bash; ' + suite('frozen-v5', str(seed), cases), f'frozen-seed-{seed}.log')
    return name


def auxiliary():
    name = 'duojin02-narrow-aux'
    create(name, 108)
    execute(name, 'cd /tmp; tar -xzf /evidence/optimized-build.tar.gz; source /tmp/navbuild/install/setup.bash; ' +
            suite('regression-v5', '1 2 3 16 17 19', 'corridor_0.50') +
            suite('online-door-v5', '20 21 22 23 24 25', 'online_corridor_0.50') +
            suite('waypoints-v5', '20 21', 'corridor_waypoints_0.50'), 'auxiliary-v5.log')
    return name


def boundary():
    name = 'duojin02-narrow-boundary'
    create(name, 109)
    execute(name, 'cd /tmp; tar -xzf /evidence/optimized-build.tar.gz; source /tmp/navbuild/install/setup.bash; ' +
            suite('checks-v5', '20', 'corridor_0.56 corridor_0.52 corridor_0.48 corridor_0.46 corridor_0.44 corridor_0.40 legacy_map_3cm late_initial_pose unsafe_fast_command'), 'boundary-v5.log')
    return name


def paired(previous):
    name = 'duojin02-narrow-previous' if previous else 'duojin02-narrow-paired'
    create(name, 110 if previous else 111, previous)
    if previous:
        prepare = 'mkdir /tmp/navbuild; cd /tmp/navbuild; colcon build --base-paths /hardware/src/duojin01_navigation /hardware/src/duojin01_bringup /hardware/src/duojin01_description /ws/simulation/src --cmake-args -DCMAKE_BUILD_TYPE=Release; source install/setup.bash; python3 /ws/simulation/record_navigation_build.py; '
    else:
        prepare = 'cd /tmp; tar -xzf /evidence/optimized-build.tar.gz; source /tmp/navbuild/install/setup.bash; '
    execute(name, prepare + suite('paired-previous-v4' if previous else 'paired-optimized-v5', '26 27', 'open corridor_0.50 online_thin dynamic_crossing office_chicane'), 'paired-previous-v4.log' if previous else 'paired-optimized-v5.log')
    return name


with ThreadPoolExecutor(max_workers=10) as pool:
    futures = [pool.submit(worker, seed) for seed in range(20, 26)]
    futures += [pool.submit(auxiliary), pool.submit(boundary), pool.submit(paired, True), pool.submit(paired, False)]
    for future in as_completed(futures):
        print('COMPLETED', future.result(), flush=True)
