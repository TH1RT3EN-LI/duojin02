"""A changed or redirected native release must fail before overlay activation."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock
from conftest import CHASSIS_PROFILE
from duojin01_bringup import chassis_profile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("verify_orin_top1", ROOT / "scripts/verify_orin_top1.py")
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


class ReleaseAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "root"
        self.root.mkdir()
        self.install = self.root / verify.INSTALL_SUBDIR
        self.required = {}
        for name in verify.REQUIRED_LIBRARIES | verify.REQUIRED_METADATA:
            path = self.install / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(("pinned file " + name).encode())
            self.required[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.profile = self.root / verify.PROFILE_PATH
        self.profile.parent.mkdir(parents=True)
        shutil.copy(CHASSIS_PROFILE, self.profile)
        renderer = self.root / "src/duojin01_bringup/duojin01_bringup/chassis_profile.py"
        renderer.parent.mkdir(parents=True)
        shutil.copy(ROOT / "src/duojin01_bringup/duojin01_bringup/chassis_profile.py", renderer)
        selected = chassis_profile.load_profile(self.profile)
        derived = {name: hashlib.sha256(chassis_profile.payload(selected, name)).hexdigest()
                   for name in sorted(chassis_profile.COMPONENTS | {'lidar', 'slam_offline'})}
        self.manifest = {
            "schema_version": 1,
            "configuration_schema": 1,
            "architecture": "aarch64",
            "cuda_architecture": "sm87",
            "profile": {"path": verify.PROFILE_PATH, "sha256": hashlib.sha256(self.profile.read_bytes()).hexdigest(), "derived_component_sha256": derived},
            "install_subdir": verify.INSTALL_SUBDIR,
            "required_install_files": copy.deepcopy(self.required),
            "runtime_environment": copy.deepcopy(selected["runtime_environment"]),
        }
        self.write_manifest()

    def write_manifest(self):
        path = self.root / verify.MANIFEST_PATH
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.manifest))

    def audit(self):
        return verify.verify_release(self.root, files_only=True)

    def test_pinned_release_admits_without_loading_native_code(self):
        with mock.patch.object(verify.platform, "machine", return_value="aarch64"):
            result = verify.verify_release(self.root)
        self.assertTrue(result["ok"])
        self.assertTrue(result["architecture_checked"])
        self.assertEqual(result["verified_install_files"], len(self.required))

    def test_offline_flag_only_skips_host_architecture(self):
        with mock.patch.object(verify.platform, "machine", return_value="x86_64"):
            with self.assertRaisesRegex(verify.ReleaseError, "requires aarch64"):
                verify.verify_release(self.root)
            self.assertFalse(self.audit()["architecture_checked"])
        self.profile.write_text("changed profile")
        with self.assertRaisesRegex(verify.ReleaseError, "SHA-256 mismatch"):
            self.audit()

    def test_corrupt_and_missing_libraries_refuse(self):
        name = sorted(verify.REQUIRED_LIBRARIES)[0]
        path = self.install / name
        path.write_bytes(b"different native code")
        with self.assertRaisesRegex(verify.ReleaseError, "SHA-256 mismatch"):
            self.audit()
        path.unlink()
        with self.assertRaises(FileNotFoundError):
            self.audit()

    def test_metadata_cannot_be_omitted_from_manifest(self):
        del self.manifest["required_install_files"][sorted(verify.REQUIRED_METADATA)[0]]
        self.write_manifest()
        with self.assertRaisesRegex(verify.ReleaseError, "omits required files"):
            self.audit()

    def test_lexical_traversal_and_absolute_paths_refuse(self):
        for path in ("../outside", "/tmp/outside", "duojin01_navigation/../outside", "./outside", "bad\\path"):
            with self.subTest(path=path):
                self.manifest["required_install_files"] = {**self.required, path: "0" * 64}
                self.write_manifest()
                with self.assertRaises(verify.ReleaseError):
                    self.audit()

    def test_file_symlink_cannot_escape_even_with_matching_hash(self):
        external = Path(self.temp.name) / "external"
        external.write_bytes(b"external code")
        name = sorted(verify.REQUIRED_LIBRARIES)[0]
        path = self.install / name
        path.unlink()
        path.symlink_to(external)
        self.manifest["required_install_files"][name] = hashlib.sha256(external.read_bytes()).hexdigest()
        self.write_manifest()
        with self.assertRaisesRegex(verify.ReleaseError, "escapes"):
            self.audit()

    def test_install_symlink_cannot_escape_root(self):
        external = Path(self.temp.name) / "external-install"
        self.install.rename(external)
        self.install.symlink_to(external, target_is_directory=True)
        with self.assertRaisesRegex(verify.ReleaseError, "escapes"):
            self.audit()

    def test_configured_runtime_cannot_drift(self):
        self.manifest["runtime_environment"]["DUOJIN_GPU_COST_KERNEL"] = "scalar"
        self.write_manifest()
        with self.assertRaisesRegex(verify.ReleaseError, "runtime environment"):
            self.audit()

    def test_old_collection_manifest_and_derived_drift_are_rejected(self):
        del self.manifest["configuration_schema"]
        self.write_manifest()
        with self.assertRaisesRegex(verify.ReleaseError, "single chassis"):
            self.audit()
        self.manifest["configuration_schema"] = 1
        self.manifest["profile"]["derived_component_sha256"]["navigation"] = "0" * 64
        self.write_manifest()
        with self.assertRaisesRegex(verify.ReleaseError, "derived chassis"):
            self.audit()

    def test_invalid_master_refuses_even_with_matching_file_hash(self):
        text = self.profile.read_text().replace("forward_mps: 0.5", "forward_mps: .nan", 1)
        self.profile.write_text(text)
        self.manifest["profile"]["sha256"] = hashlib.sha256(self.profile.read_bytes()).hexdigest()
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "nonfinite|finite"):
            self.audit()


class ShellEntryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "scripts").mkdir()
        self.overlay = self.root / "install/orin_top1"
        self.overlay.mkdir(parents=True)
        shutil.copy(ROOT / "source_top1.sh", self.root / "source_top1.sh")
        shutil.copy(ROOT / "scripts/navigation_top1.sh", self.root / "scripts/navigation_top1.sh")
        self.fake_bin = self.root / "bin"
        self.fake_bin.mkdir()
        self.write_executable("python3", """#!/bin/bash
if [[ $1 == */chassis_config.py ]]; then
  printf '%s\\n' 'OMP_NUM_THREADS=4' 'DUOJIN_GPU_COST_KERNEL=interval' 'DUOJIN_GPU_ROLLOUT_THREADS=32' 'DUOJIN_GPU_COST_THREADS=32'
  exit 0
fi
exit "${FAKE_VERIFY_STATUS:-0}"
""")
        self.write_executable("ros2", '#!/bin/bash\nprintf "%s\\n" "$@" > "$CALL_FILE"\n')
        (self.root / "source.sh").write_text('''[[ -z "${COLCON_CURRENT_PREFIX+x}" ]] || return 11
export BASE_WAS_SOURCED=1
export AMENT_PREFIX_PATH=/ordinary/navigation
export LD_LIBRARY_PATH=/ordinary/lib
''')
        (self.overlay / "local_setup.bash").write_text('''[[ -z "${COLCON_CURRENT_PREFIX+x}" ]] || return 12
export TOP1_WAS_SOURCED=1
COLCON_CURRENT_PREFIX=/overlay-set-prefix
''')
        self.env = os.environ.copy()
        self.env.update(PATH=f"{self.fake_bin}:{self.env['PATH']}", CALL_FILE=str(self.root / "ros2-call"))

    def write_executable(self, name, body):
        path = self.fake_bin / name
        path.write_text(body)
        path.chmod(0o755)

    def bash(self, body):
        return subprocess.run(["bash", "-c", body], cwd=self.root, env=self.env, text=True, capture_output=True)

    def test_failed_admission_does_not_source_overlays(self):
        self.env["FAKE_VERIFY_STATUS"] = "1"
        result = self.bash('source ./source_top1.sh; rc=$?; [[ $rc == 1 && -z ${BASE_WAS_SOURCED+x} && -z ${TOP1_WAS_SOURCED+x} ]]')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_activation_preserves_graph_and_restores_colcon_prefix(self):
        result = self.bash('''export COLCON_CURRENT_PREFIX="/wrong prefix"
export ROS_DOMAIN_ID=37 CYCLONEDDS_URI="file:///kept.xml"
export DUOJIN_TRACE_DIR=old HIL_LAYER_SPANS=old DUOJIN_LAYER_SPANS_DIR=old DUOJIN_LAYER_OBSERVER_IMPL=old
LD_PRELOAD=old
source ./source_top1.sh || exit
[[ $COLCON_CURRENT_PREFIX == "/wrong prefix" && $ROS_DOMAIN_ID == 37 && $CYCLONEDDS_URI == file:///kept.xml ]] || exit 21
[[ $TOP1_WAS_SOURCED == 1 && $OMP_NUM_THREADS == 4 && $DUOJIN_GPU_COST_KERNEL == interval && $DUOJIN_GPU_ROLLOUT_THREADS == 32 && $DUOJIN_GPU_COST_THREADS == 32 ]] || exit 22
[[ $AMENT_PREFIX_PATH == "$PWD/install/orin_top1/duojin01_navigation:/ordinary/navigation" ]] || exit 23
[[ $LD_LIBRARY_PATH == "$PWD/install/orin_top1/duojin01_navigation/lib:/ordinary/lib" ]] || exit 24
[[ -z ${DUOJIN_TRACE_DIR+x} && -z ${HIL_LAYER_SPANS+x} && -z ${DUOJIN_LAYER_SPANS_DIR+x} && -z ${DUOJIN_LAYER_OBSERVER_IMPL+x} && -z ${LD_PRELOAD+x} ]] || exit 25
[[ $(declare -p COLCON_CURRENT_PREFIX) == 'declare -x '* ]]
''')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_unset_colcon_prefix_stays_unset(self):
        result = self.bash('unset COLCON_CURRENT_PREFIX; source ./source_top1.sh || exit; [[ -z ${COLCON_CURRENT_PREFIX+x} ]]')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_failed_base_setup_restores_unexported_colcon_prefix(self):
        (self.root / "source.sh").write_text("return 17\n")
        result = self.bash('COLCON_CURRENT_PREFIX="/wrong prefix"; export -n COLCON_CURRENT_PREFIX; source ./source_top1.sh; rc=$?; [[ $rc == 17 && $COLCON_CURRENT_PREFIX == "/wrong prefix" && $(declare -p COLCON_CURRENT_PREFIX) != "declare -x "* ]]')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_readonly_trace_flag_refuses_activation(self):
        result = self.bash('readonly DUOJIN_TRACE_DIR=old; source ./source_top1.sh; rc=$?; [[ $rc != 0 && -z ${BASE_WAS_SOURCED+x} ]]')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_wrapper_rejects_profile_override_before_launch(self):
        result = self.bash('./scripts/navigation_top1.sh chassis_profile:=/another/chassis.yaml')
        self.assertEqual(result.returncode, 2)
        self.assertFalse((self.root / "ros2-call").exists())

    def test_wrapper_passes_show_args_and_pinned_profile(self):
        result = self.bash('./scripts/navigation_top1.sh --show-args')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = (self.root / "ros2-call").read_text().splitlines()
        self.assertEqual(args, ["launch", "duojin01_bringup", "navigation.launch.py", f"chassis_profile:={self.root}/src/duojin01_bringup/config/chassis.yaml", "--show-args"])


if __name__ == "__main__":
    unittest.main()
