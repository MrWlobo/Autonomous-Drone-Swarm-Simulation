from __future__ import annotations
from typing import TYPE_CHECKING
from pathlib import Path

from .base import Preset

if TYPE_CHECKING:
    from model.model import DroneModel


class Hangzhou35806Preset(Preset):
    # one cell is assumed to be about 2m x 2m
    width: int = 985 // 2
    height: int = 1310 // 2
    background: Path = Path(__file__).parent.parent.parent / "visualization/assets/Hangzhou_35806.png"
    show_gridlines = False
    
    
    def set_model_params(self, model: DroneModel) -> None:
        model.width = self.width
        model.height = self.height
        model.background = self.background
        model.show_gridlines = False