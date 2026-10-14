"""Protect direct probe invocations from inheriting a deployment ROS domain."""

import ast
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
INTEGRATION = ROOT / 'tests/integration'


@pytest.fixture
def isolation(monkeypatch):
    for key, value in {
        'ROS_DOMAIN_ID': '215', 'ROS_LOCALHOST_ONLY': '0',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'ROS_STATIC_PEERS': '192.0.2.1',
        'CYCLONEDDS_URI': '/deployment/cyclonedds.xml',
        'FASTRTPS_DEFAULT_PROFILES_FILE': '/deployment/fastdds.xml',
        'FASTDDS_DEFAULT_PROFILES_FILE': '/deployment/fastdds.xml',
    }.items():
        monkeypatch.setenv(key, value)
    spec = importlib.util.spec_from_file_location('probe_isolation', INTEGRATION / 'ros_isolation.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    yield module
    if module._reservation is not None:
        module._reservation.close()


def assert_isolated():
    assert int(os.environ['ROS_DOMAIN_ID']) in range(216, 230)
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    for key in ('ROS_STATIC_PEERS', 'CYCLONEDDS_URI',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        assert key not in os.environ


def test_subprocesses_inherit_the_reserved_local_domain(isolation):
    domain = isolation.configure_isolation()
    assert isolation.configure_isolation() == domain
    assert_isolated()
    inherited = json.loads(subprocess.check_output([
        sys.executable, '-c',
        'import json, os; print(json.dumps({key: os.environ.get(key) for key in '
        '("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "CYCLONEDDS_URI")}))'], text=True))
    assert inherited == {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
                         'CYCLONEDDS_URI': None}


@pytest.mark.parametrize('name', ['localization_manager_probe', 'navigation_command_timeout_probe'])
def test_direct_probe_configures_isolation_before_ros_init(isolation, tmp_path, name):
    # Execute the actual entry function with ROS init replaced by a stopping
    # sentinel. This exercises initialization order without loading DDS on host.
    tree = ast.parse((INTEGRATION / f'{name}.py').read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'run')

    class Initialized(Exception):
        pass

    def init():
        assert_isolated()
        raise Initialized

    namespace = {'configure_isolation': isolation.configure_isolation,
                 'rclpy': SimpleNamespace(init=init), 'yaml': __import__('yaml')}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(INTEGRATION / f'{name}.py'), 'exec'), namespace)
    args = SimpleNamespace(output_dir=tmp_path / name,
                           params=ROOT / 'src/duojin02_bringup/config/nav2.yaml')
    with pytest.raises(Initialized):
        namespace['run'](args)


def test_concurrent_probe_domains_are_reserved(isolation):
    first_domain = isolation.configure_isolation()
    code = ('import sys; sys.path.insert(0, sys.argv[1]); '
            'from ros_isolation import configure_isolation; print(configure_isolation())')
    environment = {**os.environ, 'ROS_DOMAIN_ID': '215'}
    second_domain = int(subprocess.check_output(
        [sys.executable, '-c', code, str(INTEGRATION)], env=environment, text=True))
    assert second_domain != first_domain
