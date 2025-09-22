"""
Elevation services for terrain-aware target calculations.
Provides SRTM-based elevation data for accurate altitude calculations.
"""

from .srtm_service import OfflineSRTMService

__all__ = ['OfflineSRTMService']