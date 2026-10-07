from concurrent.futures import ThreadPoolExecutor, as_completed
import importlib.util
from pathlib import Path
import subprocess
import time

base = Path(__file__).parent
# Independent verification uses the same frozen binaries and acceptance rules.
# Limit this batch to four simultaneous graphs and preserve the ten-graph batch.
sim = base.resolve().parents[1]
hardware = sim.with_name('duojin02')
cases = 'open corridor_0.50 thin_obstacles s_bend dynamic_crossing sensor_dropout initial_pose_error online_thin online_open office_chicane online_chicane reverse_goal'


def worker(seed):
    time.sleep((seed - 29) % 4 * 4)
    name = f'duojin02-narrow-validation-{seed}'
    subprocess.run(['docker', 'run', '-d', '--rm', '--init', '--gpus', 'all', '--shm-size=1g', '--name', name,
        '-e', 'NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute', '-e', f'ROS_DOMAIN_ID={114 + seed - 29}',
        '-e', f'GZ_PARTITION={name}', '-v', f'{hardware}:/hardware:ro', '-v', f'{sim}:/ws:ro', '-v', f'{base.resolve()}:/evidence',
        '--entrypoint', 'bash', 'duojin02:sim-humble-harmonic', '-c', 'exec tail -f /dev/null'], check=True, stdout=subprocess.DEVNULL)
    command = 'set -e; source /opt/ros/humble/setup.bash; source /opt/duojin01/slam_backend/setup.bash; cd /tmp; tar -xzf /evidence/optimized-build.tar.gz; source /tmp/navbuild/install/setup.bash; '
    command += f'python3 /ws/simulation/nav_benchmark_suite.py --output /evidence/validation-v5 --variants upgraded --seeds {seed} --cases {cases}'
    with (base / f'validation-seed-{seed}.log').open('w') as output:
        subprocess.run(['docker', 'exec', name, 'bash', '-lc', command], check=True, stdout=output, stderr=subprocess.STDOUT)
    print('COMPLETED', name, flush=True)


with ThreadPoolExecutor(max_workers=4) as pool:
    for future in as_completed([pool.submit(worker, seed) for seed in range(29, 35)]):
        future.result()
