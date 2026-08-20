"""Corridor and room semantic classifier package for NIDAR AirMouse."""
from corridor_classifier.region_classifier import RegionClassifier

try:
    from corridor_classifier.corridor_classifier_node import CorridorClassifierNode
    __all__ = ["RegionClassifier", "CorridorClassifierNode"]
except ImportError:
    __all__ = ["RegionClassifier"]

