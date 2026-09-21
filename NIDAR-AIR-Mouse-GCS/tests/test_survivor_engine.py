"""Unit tests for SurvivorEngine (Autonomous Localization & De-duplication)."""
import pytest
from mapping.grid_manager import GridManager
from backend.survivor_engine import SurvivorEngine


def test_survivor_detection_and_grid_mapping():
    gm = GridManager(20.0, 20.0, 2.5)
    engine = SurvivorEngine(gm)

    # 1. First Detection: (3.75, 6.25) -> Grid B3
    surv1, is_new1 = engine.process_detection(3.75, 6.25, 0.95, source="THERMAL")
    assert is_new1 is True
    assert surv1.id == "S01"
    assert surv1.grid_cell == "B3"
    assert surv1.confidence == 0.95
    assert surv1.source == "THERMAL"
    assert engine.survivor_count == 1

    # 2. Duplicate Detection nearby (3.80, 6.30) with RGB source -> Should merge
    surv_dup, is_new_dup = engine.process_detection(3.80, 6.30, 0.98, source="RGB")
    assert is_new_dup is False
    assert surv_dup.id == "S01"
    assert surv_dup.confidence == 0.98
    assert surv_dup.source == "FUSED_AI"  # Multimodal fused
    assert engine.survivor_count == 1     # No duplicate created!

    # 3. Second Distinct Survivor: (13.75, 11.25) -> Grid F5
    surv2, is_new2 = engine.process_detection(13.75, 11.25, 0.91, source="RGB")
    assert is_new2 is True
    assert surv2.id == "S02"
    assert surv2.grid_cell == "F5"
    assert engine.survivor_count == 2
