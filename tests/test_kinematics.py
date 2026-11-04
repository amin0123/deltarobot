import numpy as np
import pytest
from delta_sorting.kinematics import DeltaKinematics, KinematicsError


def test_cad_reference_closes_all_six_rods(geometry):
    k = DeltaKinematics(geometry)
    ref = np.array(geometry["platform_reference"])
    q = np.array(geometry["q_reference"])
    np.testing.assert_allclose(k.forward(q), ref+k.tool, atol=1e-10)
    lengths = np.linalg.norm(ref+np.array(geometry["platform_offsets"])-np.array(geometry["elbows_reference"]), axis=2)
    np.testing.assert_allclose(lengths, geometry["lower_lengths"], atol=1e-10)


def test_random_workspace_roundtrip_and_constraint_residual(geometry, cfg):
    k = DeltaKinematics(geometry, cfg)
    rng = np.random.default_rng(8)
    accepted = 0
    for _ in range(500):
        p = rng.uniform([-.13, -.13, -.38], [.13, .13, -.285])
        try:
            q = k.inverse(p)
        except KinematicsError:
            continue
        accepted += 1
        np.testing.assert_allclose(k.forward(q), p, atol=1e-10)
        np.testing.assert_allclose(np.linalg.norm(p-k.tool+k.offsets-k.elbows(q), axis=1), k.l, atol=1e-10)
    assert accepted > 300


@pytest.mark.parametrize("p", [[1, 0, -.35], [0, 0, 0], [0, 0, -1], [float("nan"), 0, -.33], [0, float("inf"), -.33]])
def test_unreachable_or_nonfinite_rejected(p, geometry, cfg):
    with pytest.raises(KinematicsError):
        DeltaKinematics(geometry, cfg).inverse(p)


def test_all_bins_and_safe_routes_reachable(geometry, cfg):
    k = DeltaKinematics(geometry, cfg)
    for xy in cfg["bins"].values():
        target = [*xy, cfg["bin_geometry"]["release_z"]]
        duration, q = k.path(cfg["robot"]["home"], target, 0.12)
        dq = np.gradient(q, duration/(len(q)-1), axis=0)
        ddq = np.gradient(dq, duration/(len(q)-1), axis=0)
        assert abs(dq).max() <= cfg["robot"]["joint_speed"]*1.001
        assert abs(ddq).max() <= cfg["robot"]["joint_acceleration"]*1.001
        np.testing.assert_allclose(k.forward(q[-1]), target, atol=1e-10)


def test_belt_collision_path_rejected(geometry, cfg):
    k = DeltaKinematics(geometry, cfg)
    with pytest.raises(KinematicsError, match="belt"):
        k.path([0, 0, -.3], [0, 0, -.375], .1)


def test_bin_wall_collision_rejected(geometry, cfg):
    k = DeltaKinematics(geometry, cfg)
    with pytest.raises(KinematicsError, match="bin"):
        k.path([-.06, .11, -.3], [-.06, .11, -.35], .1)
