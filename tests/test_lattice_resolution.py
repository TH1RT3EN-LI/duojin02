"""Old map reuse must preserve metric geometry and connected cell endpoints."""
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/duojin02_bringup'))
from duojin02_bringup.lattice import for_resolution
SOURCE=Path(__file__).resolve().parents[1]/'src/duojin02_bringup/config/lattice_primitives/duojin02_omni_2cm_rich_lattice.json'


@pytest.mark.parametrize('resolution',[.01,.02,.03,.05])
def test_scaled_primitives_preserve_endpoints_and_complete_metric_lengths(resolution):
    original=json.loads(SOURCE.read_text());scaled=json.loads(Path(for_resolution(SOURCE,resolution)).read_text())
    assert scaled['lattice_metadata']['grid_resolution']==resolution
    assert scaled['lattice_metadata']['heading_angles']==original['lattice_metadata']['heading_angles']
    assert len(scaled['primitives'])==480
    for a,b in zip(original['primitives'],scaled['primitives']):
        for field in ['trajectory_radius','trajectory_length','arc_length','straight_length']:
            assert b[field]/resolution==pytest.approx(a[field]/.02)
        for p,q in zip(a['poses'],b['poses']):
            assert q[0]/resolution==pytest.approx(p[0]/.02)
            assert q[1]/resolution==pytest.approx(p[1]/.02)
            assert q[2]==p[2]
        assert (a['start_angle_index'],a['end_angle_index'])==(b['start_angle_index'],b['end_angle_index'])


@pytest.mark.parametrize('resolution',[0,float('nan'),float('inf'),-.02])
def test_invalid_resolution_is_rejected(resolution):
    with pytest.raises(ValueError):for_resolution(SOURCE,resolution)
