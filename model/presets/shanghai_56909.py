from __future__ import annotations
from typing import TYPE_CHECKING
from pathlib import Path

from .base import Preset

if TYPE_CHECKING:
    from model.model import DroneModel


class Shanghai56909Preset(Preset):
    # one cell is assumed to be about 2m x 2m
    width: int = 395 // 2
    height: int = 976 // 2
    background: Path = Path(__file__).parent.parent.parent / "visualization/assets/Shanghai_56909.png"
    show_gridlines = False
    
    
    def set_model_params(self, model: DroneModel) -> None:
        model.width = self.width
        model.height = self.height
        model.background = self.background
        model.show_gridlines = self.show_gridlines