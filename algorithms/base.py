from abc import ABC, abstractmethod

class BaseSwarmStrategy(ABC):
    def __init__(self, model):
        self.model = model

    @abstractmethod
    def decide(self, drone):
        pass
