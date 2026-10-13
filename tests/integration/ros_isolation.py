"""Give a probe and all of its subprocesses one local, reserved ROS domain."""

import fcntl
import os
from pathlib import Path
import secrets
import subprocess
import sys
import tempfile


_reservation = None
_domain = None


def configure_isolation():
    """Override deployment discovery settings before any ROS context is created."""
    global _reservation, _domain
    if _reservation is not None:
        return _domain
    inherited_domain = os.environ.get('ROS_DOMAIN_ID')
    # These domains avoid Linux's default ephemeral-port range. A lock keeps
    # concurrent local probes apart; inherited deployment domains are excluded.
    domains = [domain for domain in range(215, 230) if str(domain) != inherited_domain]
    secrets.SystemRandom().shuffle(domains)
    directory = Path(tempfile.gettempdir()) / f'duojin02-probe-domains-{os.getuid()}'
    directory.mkdir(mode=0o700, exist_ok=True)
    for domain in domains:
        reservation = (directory / str(domain)).open('a')
        try:
            fcntl.flock(reservation, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            reservation.close()
            continue
        _reservation, _domain = reservation, domain
        os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                          ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        os.environ.pop('ROS_STATIC_PEERS', None)
        # External DDS profiles can widen discovery despite the ROS setting.
        for key in ('CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                    'FASTDDS_DEFAULT_PROFILES_FILE'):
            os.environ.pop(key, None)
        return domain
    raise RuntimeError('All isolated integration-test ROS domains are in use')


if __name__ == '__main__':
    configure_isolation()
    if len(sys.argv) < 2:
        raise SystemExit('Usage: ros_isolation.py COMMAND [ARG ...]')
    raise SystemExit(subprocess.call(sys.argv[1:]))
