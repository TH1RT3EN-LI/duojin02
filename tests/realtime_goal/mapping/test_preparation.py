import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('prepare_slam_backend',ROOT/'scripts/prepare_slam_backend.py')
backend=importlib.util.module_from_spec(spec);spec.loader.exec_module(backend)
class ManifestTests(unittest.TestCase):
    def test_only_explicit_unique_ordered_patch_names(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);patches=root/'patches/slam_toolbox';patches.mkdir(parents=True)
            (patches/'a.patch').write_text('a');(patches/'b.patch').write_text('b');(patches/'realtime-goal-unlisted.patch').write_text('unlisted')
            source=patches/'source.json';source.write_text(json.dumps({'patches':['b.patch','a.patch']}))
            _,files,_=backend.source_provenance(root)
            self.assertEqual([f.name for f in files],['b.patch','a.patch'])
            for names in ([],['a.patch','a.patch'],['../a.patch'],[[]]):
                source.write_text(json.dumps({'patches':names}))
                with self.assertRaises(ValueError):backend.source_provenance(root)
if __name__=='__main__':unittest.main()
