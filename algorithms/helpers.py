from algorithms.dummy import Dummy


def get_strategy_instance(name, model):
    if name == "dummy":
        return Dummy(model)