from __future__ import annotations
from typing import TYPE_CHECKING
import abc

if TYPE_CHECKING:
    from model.model import DroneModel


class Preset(abc.ABC):
    @abc.abstractmethod
    def set_model_params(self, model: DroneModel) -> None:
        """This method is called at the end of the model's __init__ method
        to override it's parameters according to the preset.

        Args:
            model (DroneModel): Target model for parameter overwriting.
        """
        pass



