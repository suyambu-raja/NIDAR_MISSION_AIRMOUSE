"""Vision package for NIDAR AirMouse GCS."""
from .video_provider import BaseVideoProvider, OpenCVVideoProvider
from .survivor_provider import BaseSurvivorProvider

__all__ = ["BaseVideoProvider", "OpenCVVideoProvider", "BaseSurvivorProvider"]
