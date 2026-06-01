"""Unit tests for the pure-logic helpers in lidar_camera_fusion."""

from types import SimpleNamespace

import numpy as np
import pytest

# lidar_camera_fusion pulls in rclpy / cv2 / cv_bridge / vision_msgs at module
# load. Skip the whole suite cleanly when any of those aren't importable
# (e.g. dev laptop without ros-humble, or numpy/cv_bridge ABI mismatch).
fusion = pytest.importorskip("r2.lidar_camera_fusion")

FusionConfig       = fusion.FusionConfig
MeihuaForestGrid   = fusion.MeihuaForestGrid
TikTacTeoRackGrid  = fusion.TikTacTeoRackGrid
points_in_bbox     = fusion.points_in_bbox
transform_points   = fusion.transform_points


# ── Grids ─────────────────────────────────────────────────────────────────────

def test_forest_nearest_block_hits_block_1_at_origin_corner():
    cfg = FusionConfig()
    grid = MeihuaForestGrid(cfg)
    block_num, dist = grid.nearest_block(
        np.array([cfg.BLOCK_1_X, cfg.BLOCK_1_Y, cfg.BLOCK_TOP_Z])
    )
    assert block_num == 1
    assert dist == pytest.approx(0.0, abs=1e-9)


def test_forest_nearest_block_picks_diagonal_neighbour():
    cfg = FusionConfig()
    grid = MeihuaForestGrid(cfg)
    target = np.array([
        cfg.BLOCK_1_X + cfg.BLOCK_SPACING_X,
        cfg.BLOCK_1_Y + cfg.BLOCK_SPACING_Y,
        cfg.BLOCK_TOP_Z,
    ])
    block_num, dist = grid.nearest_block(target)
    assert block_num == 5  # (col=1, row=1)
    assert dist == pytest.approx(0.0, abs=1e-9)


def test_arena_nearest_slot_hits_slot_1():
    cfg = FusionConfig()
    rack = TikTacTeoRackGrid(cfg)
    slot_num, dist = rack.nearest_slot(
        np.array([cfg.SLOT_1_X, cfg.SLOT_1_Y, cfg.FIRST_ROW_Z])
    )
    assert slot_num == 1
    assert dist == pytest.approx(0.0, abs=1e-9)


def test_arena_top_row_uses_top_z():
    cfg = FusionConfig()
    rack = TikTacTeoRackGrid(cfg)
    target = np.array([cfg.SLOT_1_X, cfg.SLOT_1_Y + 2 * cfg.SLOT_SPACING_Y, cfg.TOP_ROW_Z])
    slot_num, dist = rack.nearest_slot(target)
    assert slot_num == 7  # (col=0, row=2)
    assert dist == pytest.approx(0.0, abs=1e-9)


# ── team_or_opp truth table ───────────────────────────────────────────────────

@pytest.mark.parametrize(
    "playing_as_red,cls,expected",
    [
        (True,  1, "TEAM_KFS"),
        (True,  0, "OPP_KFS"),
        (False, 1, "OPP_KFS"),
        (False, 0, "TEAM_KFS"),
    ],
)
def test_team_or_opp(playing_as_red, cls, expected):
    rack = TikTacTeoRackGrid(FusionConfig())
    assert rack.team_or_opp(playing_as_red, cls) == expected


# ── transform_points ──────────────────────────────────────────────────────────

def test_transform_points_identity_is_noop():
    pts = np.array([[1.0, 2.0, 3.0], [-1.0, 0.5, 2.5]])
    np.testing.assert_allclose(transform_points(pts, np.eye(4)), pts)


def test_transform_points_pure_translation():
    pts = np.array([[1.0, 2.0, 3.0]])
    T = np.eye(4)
    T[:3, 3] = [10.0, -5.0, 0.5]
    np.testing.assert_allclose(transform_points(pts, T), [[11.0, -3.0, 3.5]])


# ── points_in_bbox ────────────────────────────────────────────────────────────

def _make_det(cx, cy, w, h):
    """Build the minimal duck-typed detection points_in_bbox needs."""
    return SimpleNamespace(bbox=SimpleNamespace(
        center=SimpleNamespace(position=SimpleNamespace(x=cx, y=cy)),
        size_x=w, size_y=h,
    ))


def test_points_in_bbox_keeps_only_inside_pixels():
    cfg = FusionConfig()
    cfg.KFS_MIN_POINTS = 1
    cfg.KFS_DEPTH_MARGIN = 100.0  # disable depth filter for this test

    pts = np.array([
        [1.0, 0.0, 5.0],   # inside
        [1.0, 0.0, 5.1],   # inside
        [9.0, 9.0, 5.0],   # outside (pixel far away)
    ])
    pixels = np.array([
        [100.0, 100.0],
        [110.0, 105.0],
        [800.0, 600.0],
    ])
    valid = np.array([True, True, True])
    det = _make_det(cx=105, cy=102, w=40, h=40)

    out = points_in_bbox(pts, pixels, valid, det, cfg, img_w=1280, img_h=720)
    assert out.shape[0] == 2


def test_points_in_bbox_returns_empty_when_below_min_points():
    cfg = FusionConfig()
    cfg.KFS_MIN_POINTS = 5

    pts = np.array([[1.0, 0.0, 5.0], [1.0, 0.0, 5.1]])
    pixels = np.array([[100.0, 100.0], [101.0, 101.0]])
    valid = np.array([True, True])
    det = _make_det(cx=100, cy=100, w=40, h=40)

    out = points_in_bbox(pts, pixels, valid, det, cfg, img_w=1280, img_h=720)
    assert out.shape == (0, 3)


def test_points_in_bbox_depth_filter_drops_background():
    cfg = FusionConfig()
    cfg.KFS_MIN_POINTS = 1
    cfg.KFS_DEPTH_MARGIN = 0.10  # tight margin

    pts = np.array([
        [0.0, 0.0, 1.0],   # near
        [0.0, 0.0, 1.05],  # near
        [0.0, 0.0, 5.0],   # background — should be dropped
    ])
    pixels = np.array([[100.0, 100.0]] * 3)
    valid = np.array([True, True, True])
    det = _make_det(cx=100, cy=100, w=40, h=40)

    out = points_in_bbox(pts, pixels, valid, det, cfg, img_w=1280, img_h=720)
    assert out.shape[0] == 2
    assert np.linalg.norm(out, axis=1).max() < 5.0
