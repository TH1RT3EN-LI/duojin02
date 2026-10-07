from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import queue
import subprocess
import time

sim = Path('/home/th1rt3en/dev/forge/duojin02-sim')
hardware = sim.with_name('duojin02')
cache = sim / '.cache/navigation-narrow'
cases = 'open corridor_0.50 thin_obstacles s_bend dynamic_crossing sensor_dropout initial_pose_error online_thin online_open office_chicane online_chicane reverse_goal online_corridor_0.50'.split()
tasks = queue.Queue()
for seed in range(35, 41):
    for case in cases: tasks.put((seed, case))
common = 'set -e; source /opt/ros/humble/setup.bash; source /opt/duojin01/slam_backend/setup.bash; '


def execute(name, command, log):
    with (cache / log).open('a') as output:
        subprocess.run(['docker', 'exec', name, 'bash', '-lc', common + command], check=True, stdout=output, stderr=subprocess.STDOUT)


def create(name, domain):
    subprocess.run(['docker', 'run', '-d', '--rm', '--init', '--gpus', 'all', '--shm-size=1g', '--name', name,
        '-e', 'NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute', '-e', f'ROS_DOMAIN_ID={domain}',
        '-e', f'GZ_PARTITION={name}', '-v', f'{hardware}:/hardware:ro', '-v', f'{sim}:/ws:ro', '-v', f'{cache}:/evidence',
        '--entrypoint', 'bash', 'duojin02:sim-humble-harmonic', '-c', 'exec tail -f /dev/null'], check=True, stdout=subprocess.DEVNULL)
    execute(name, 'cd /tmp; tar -xzf /evidence/final-build.tar.gz', name + '.log')


def suite(name, root, seeds, selected):
    execute(name, f'source /tmp/navbuild/install/setup.bash; python3 /ws/simulation/nav_benchmark_suite.py --output /evidence/{root} --variants upgraded --seeds {seeds} --cases {selected}', name + '.log')


def primary(worker):
    time.sleep(worker * 3)
    name = f'duojin02-narrow-final-{worker}'
    create(name, 121 + worker)
    while True:
        try: seed, case = tasks.get_nowait()
        except queue.Empty: break
        suite(name, 'frozen-v6', str(seed), case)
        tasks.task_done()
    print('COMPLETED', name, flush=True)


def auxiliary():
    time.sleep(18)
    name = 'duojin02-narrow-final-aux'
    create(name, 125)
    suite(name, 'regression-v6', '1 2 3 16 17 19', 'corridor_0.50')
    suite(name, 'online-counterexample-v6', '17 24', 'online_corridor_0.50')
    suite(name, 'waypoints-v6', '20 21', 'corridor_waypoints_0.50')
    suite(name, 'checks-v6', '41', 'corridor_0.56 corridor_0.52 corridor_0.48 corridor_0.46 corridor_0.44 corridor_0.40 legacy_map_3cm late_initial_pose unsafe_fast_command')
    print('COMPLETED', name, flush=True)


def paired_previous():
    name = 'duojin02-narrow-previous'
    execute(name, 'cd /tmp/navbuild; colcon build --base-paths /hardware/src/duojin01_navigation /hardware/src/duojin01_bringup /hardware/src/duojin01_description /ws/simulation/src --cmake-args -DCMAKE_BUILD_TYPE=Release; source install/setup.bash; python3 /ws/simulation/record_navigation_build.py; cd /tmp; tar -czf /evidence/previous-build-final.tar.gz navbuild', 'previous-final-build.log')
    suite(name, 'paired-previous-final-v4', '42 43', 'open corridor_0.50 online_thin dynamic_crossing office_chicane')


def paired_new():
    suite('duojin02-narrow-final-0', 'paired-optimized-v6', '42 43', 'open corridor_0.50 online_thin dynamic_crossing office_chicane')


def limited():
    name = 'duojin02-narrow-limited'
    execute(name, 'cd /tmp; rm -rf navbuild; tar -xzf /evidence/final-build.tar.gz', 'limited-final-prepare.log')
    suite(name, 'cpu-limited-v6', '41', 'open online_chicane dynamic_crossing sensor_dropout corridor_0.50')


with ThreadPoolExecutor(max_workers=5) as pool:
    primary_jobs = [pool.submit(primary, worker) for worker in range(4)]
    aux = pool.submit(auxiliary)
    for future in as_completed(primary_jobs): future.result()
    jobs = [pool.submit(paired_previous), pool.submit(paired_new), pool.submit(limited)]
    for future in as_completed(jobs): future.result()
    aux.result()
