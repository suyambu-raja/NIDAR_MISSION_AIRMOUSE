"""
Grid Manager for NIDAR AirMouse GCS.
Handles conversion between continuous local metric Cartesian coordinates (X, Y in meters)
and discrete search grid boxes (e.g., 'A1', 'C4', 'H8').
"""
import math
from typing import Tuple, Optional


class GridManager:
    """
    Manages the search grid coordinate system for GPS-denied indoor search missions.
    
    Standard convention:
    - Columns: Letters 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', ... (left to right / along X)
    - Rows: Numbers '1', '2', '3', '4', '5', '6', '7', '8', ... (bottom to top / along Y)
    
    For a 20m x 20m arena with 2.5m cell size:
    - 8 columns (A-H)
    - 8 rows (1-8)
    - Total 64 search grid cells (A1 to H8)
    """

    ROW_LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"

    def __init__(
        self,
        grid_width_meters: float = 20.0,
        grid_height_meters: float = 20.0,
        cell_size_meters: float = 2.5,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ):
        self.grid_width_meters = max(1.0, float(grid_width_meters))
        self.grid_height_meters = max(1.0, float(grid_height_meters))
        self.cell_size_meters = max(0.1, float(cell_size_meters))
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        self.num_cols = max(1, int(math.ceil(self.grid_width_meters / self.cell_size_meters)))
        self.num_rows = max(1, int(math.ceil(self.grid_height_meters / self.cell_size_meters)))

    def metric_to_grid(self, x_m: float, y_m: float) -> str:
        """
        Converts a continuous metric coordinate (x, y in meters) to a grid box label (e.g. 'C4').
        
        Args:
            x_m: Metric X coordinate (East/Right).
            y_m: Metric Y coordinate (North/Up).
            
        Returns:
            Grid string like 'A1', 'C4', or 'OUT_OF_BOUNDS'.
        """
        rel_x = x_m - self.origin_x
        rel_y = y_m - self.origin_y

        if rel_x < 0.0 or rel_x > self.grid_width_meters or rel_y < 0.0 or rel_y > self.grid_height_meters:
            # Handle slight floating point tolerances on exact border
            if 0.0 <= rel_x <= self.grid_width_meters + 1e-4 and 0.0 <= rel_y <= self.grid_height_meters + 1e-4:
                rel_x = min(rel_x, self.grid_width_meters - 1e-4)
                rel_y = min(rel_y, self.grid_height_meters - 1e-4)
            else:
                # Clamp or label boundary
                col_idx = max(0, min(self.num_cols - 1, int(rel_x // self.cell_size_meters)))
                row_idx = max(0, min(self.num_rows - 1, int(rel_y // self.cell_size_meters)))
                col_letter = self.ROW_LETTERS[col_idx] if col_idx < len(self.ROW_LETTERS) else f"C{col_idx}"
                return f"{col_letter}{row_idx + 1}"

        col_idx = int(rel_x // self.cell_size_meters)
        row_idx = int(rel_y // self.cell_size_meters)

        # Clamp safely within index limits
        col_idx = max(0, min(self.num_cols - 1, col_idx))
        row_idx = max(0, min(self.num_rows - 1, row_idx))

        col_letter = self.ROW_LETTERS[col_idx] if col_idx < len(self.ROW_LETTERS) else f"C{col_idx}"
        row_number = row_idx + 1

        return f"{col_letter}{row_number}"

    def grid_to_metric_center(self, grid_label: str) -> Optional[Tuple[float, float]]:
        """
        Converts a grid label (e.g. 'C4') to the metric center coordinate (x, y in meters).
        
        Args:
            grid_label: String like 'C4'.
            
        Returns:
            Tuple (center_x, center_y) in meters, or None if invalid.
        """
        grid_label = grid_label.strip().upper()
        if len(grid_label) < 2:
            return None

        col_char = grid_label[0]
        row_str = grid_label[1:]

        if col_char not in self.ROW_LETTERS:
            return None

        try:
            row_num = int(row_str)
        except ValueError:
            return None

        col_idx = self.ROW_LETTERS.index(col_char)
        row_idx = row_num - 1

        if col_idx < 0 or col_idx >= self.num_cols or row_idx < 0 or row_idx >= self.num_rows:
            return None

        center_x = self.origin_x + (col_idx + 0.5) * self.cell_size_meters
        center_y = self.origin_y + (row_idx + 0.5) * self.cell_size_meters

        return (center_x, center_y)

    def get_grid_dimensions(self) -> Tuple[int, int, float]:
        """Returns (num_cols, num_rows, cell_size_meters)."""
        return (self.num_cols, self.num_rows, self.cell_size_meters)
