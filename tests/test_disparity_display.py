#!/usr/bin/env python3
"""Phase 4: Disparity acceptance test — unit tests for _apply_disparity_colormap.

Run: python -m pytest tests/test_disparity_display.py -v
Or:  python tests/test_disparity_display.py
"""
import sys
import numpy as np

sys.path.insert(0, ".")

from dashboard.camera_panel import _apply_disparity_colormap, _compute_disparity


def _has_content(img: np.ndarray) -> bool:
    """Return True if image has at least one non-zero pixel."""
    return img is not None and img.size > 0 and np.any(img > 0)


def _fraction_nonzero(img: np.ndarray) -> float:
    """Return fraction of non-zero pixels in BGR image."""
    if img is None or img.size == 0:
        return 0.0
    return float(np.any(img > 0, axis=-1).mean())


def test_positive_disparity():
    """Positive float32 disparity should produce a visible image."""
    h, w = 120, 160
    disp = np.full((h, w), 10.0, dtype=np.float32)
    disp[:, w // 2:] = 5.0
    result = _apply_disparity_colormap(disp)
    assert result is not None, "returned None"
    assert result.shape == (h, w, 3), f"wrong shape: {result.shape}"
    assert result.dtype == np.uint8
    frac = _fraction_nonzero(result)
    assert frac > 0.9, f"too many black pixels: {frac:.2%}"
    print(f"  PASS: positive disparity → {frac:.1%} non-black")


def test_negative_disparity():
    """Negative float32 disparity (ZED convention) should also produce visible image."""
    h, w = 120, 160
    disp = np.full((h, w), -10.0, dtype=np.float32)
    disp[:, w // 2:] = -5.0
    result = _apply_disparity_colormap(disp)
    assert result is not None, "returned None"
    assert result.shape == (h, w, 3)
    frac = _fraction_nonzero(result)
    assert frac > 0.9, f"too many black pixels: {frac:.2%}"
    print(f"  PASS: negative disparity → {frac:.1%} non-black")


def test_mixed_sign_disparity():
    """Mixed positive/negative disparity should produce visible image."""
    h, w = 120, 160
    disp = np.zeros((h, w), dtype=np.float32)
    disp[:h // 2, :] = 8.0    # top half positive
    disp[h // 2:, :] = -6.0   # bottom half negative
    result = _apply_disparity_colormap(disp)
    assert result is not None, "returned None"
    frac = _fraction_nonzero(result)
    assert frac > 0.9, f"too many black pixels: {frac:.2%}"
    print(f"  PASS: mixed disparity → {frac:.1%} non-black")


def test_zero_disparity_all_black():
    """All-zero disparity should produce all-black image (no valid data)."""
    disp = np.zeros((120, 160), dtype=np.float32)
    result = _apply_disparity_colormap(disp)
    assert result is not None, "returned None"
    frac = _fraction_nonzero(result)
    assert frac == 0.0, f"expected all black, got {frac:.2%} non-black"
    print(f"  PASS: all-zero disparity → all black")


def test_nan_inf_filtered():
    """NaN/Inf pixels should be rendered as black."""
    h, w = 120, 160
    disp = np.full((h, w), 10.0, dtype=np.float32)
    disp[0:10, :] = np.nan
    disp[10:20, :] = np.inf
    disp[20:30, :] = -np.inf
    result = _apply_disparity_colormap(disp)
    assert result is not None
    # Rows 0-30 should be black, rest should be colored
    top_frac = _fraction_nonzero(result[30:, :])
    assert top_frac > 0.9, f"valid region too dark: {top_frac:.2%}"
    bot_frac = _fraction_nonzero(result[:30, :])
    assert bot_frac == 0.0, f"invalid region not black: {bot_frac:.2%}"
    print(f"  PASS: NaN/Inf filtered correctly")


def test_single_value_disparity():
    """Constant disparity should produce uniform color (not crash)."""
    disp = np.full((120, 160), 7.5, dtype=np.float32)
    result = _apply_disparity_colormap(disp)
    assert result is not None
    assert result.shape == (120, 160, 3)
    print(f"  PASS: constant disparity → no crash")


def test_empty_and_none():
    """None and empty arrays should return None."""
    assert _apply_disparity_colormap(None) is None
    assert _apply_disparity_colormap(np.array([])) is None
    print(f"  PASS: None/empty → None")


def _approx(a, b, rel=1e-4):
    return abs(a - b) / max(abs(b), 1e-9) < rel


def test_compute_disparity_from_depth():
    """_compute_disparity should match d = f*B/Z."""
    fx, B = 521.92, 0.1196
    depth = np.array([[1.0, 2.0, 5.0], [10.0, 0.0, -1.0]], dtype=np.float32)
    disp = _compute_disparity(depth, baseline=B, fx=fx)
    assert disp is not None
    assert _approx(disp[0, 0], fx * B / 1.0)
    assert _approx(disp[0, 1], fx * B / 2.0)
    assert _approx(disp[0, 2], fx * B / 5.0)
    assert _approx(disp[1, 0], fx * B / 10.0)
    assert disp[1, 1] == 0.0  # depth=0 → disparity=0
    assert disp[1, 2] == 0.0  # negative depth → disparity=0
    print(f"  PASS: depth→disparity formula correct")


def test_disparity_visualization_through_colormap():
    """Full pipeline: compute disparity from depth → colormap → verify non-black."""
    h, w = 120, 160
    depth = np.full((h, w), 3.0, dtype=np.float32)
    depth[:, :w // 2] = 1.0
    disp = _compute_disparity(depth)
    result = _apply_disparity_colormap(disp)
    assert result is not None
    frac = _fraction_nonzero(result)
    assert frac > 0.9, f"too dark: {frac:.2%}"
    print(f"  PASS: depth→disparity→colormap pipeline → {frac:.1%} non-black")


if __name__ == "__main__":
    tests = [
        test_positive_disparity,
        test_negative_disparity,
        test_mixed_sign_disparity,
        test_zero_disparity_all_black,
        test_nan_inf_filtered,
        test_single_value_disparity,
        test_empty_and_none,
        test_compute_disparity_from_depth,
        test_disparity_visualization_through_colormap,
    ]
    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except Exception as e:
            print(f"  FAIL: {t.__name__}: {e}")
            failed += 1
    print(f"\n{'=' * 40}")
    print(f"Results: {passed} passed, {failed} failed")
    if failed:
        sys.exit(1)
