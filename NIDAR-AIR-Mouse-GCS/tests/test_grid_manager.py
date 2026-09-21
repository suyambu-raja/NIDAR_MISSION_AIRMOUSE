"""Unit tests for GridManager (Metric to Search Grid Conversions)."""
import pytest
from mapping.grid_manager import GridManager


def test_grid_initialization():
    gm = GridManager(grid_width_meters=20.0, grid_height_meters=20.0, cell_size_meters=2.5)
    cols, rows, cell_sz = gm.get_grid_dimensions()
    assert cols == 8
    assert rows == 8
    assert cell_sz == 2.5


def test_metric_to_grid_conversions():
    gm = GridManager(grid_width_meters=20.0, grid_height_meters=20.0, cell_size_meters=2.5)
    
    # Origin cell (0.0, 0.0) -> A1
    assert gm.metric_to_grid(0.1, 0.1) == "A1"
    assert gm.metric_to_grid(1.25, 1.25) == "A1"

    # Cell (3.75, 6.25) -> B3 (X: 3.75 -> col 1 'B', Y: 6.25 -> row 2 '3')
    assert gm.metric_to_grid(3.75, 6.25) == "B3"

    # Far corner cell (19.0, 19.0) -> H8
    assert gm.metric_to_grid(19.0, 19.0) == "H8"


def test_grid_to_metric_center():
    gm = GridManager(grid_width_meters=20.0, grid_height_meters=20.0, cell_size_meters=2.5)
    
    # Center of A1 should be (1.25, 1.25)
    center_a1 = gm.grid_to_metric_center("A1")
    assert center_a1 is not None
    assert pytest.approx(center_a1[0], 0.01) == 1.25
    assert pytest.approx(center_a1[1], 0.01) == 1.25

    # Center of B3 should be (3.75, 6.25)
    center_b3 = gm.grid_to_metric_center("B3")
    assert center_b3 is not None
    assert pytest.approx(center_b3[0], 0.01) == 3.75
    assert pytest.approx(center_b3[1], 0.01) == 6.25

    # Invalid cells
    assert gm.grid_to_metric_center("Z99") is None
    assert gm.grid_to_metric_center("INVALID") is None
