from model.presets.hangzhou_35806 import Hangzhou35806Preset


def get_preset_instance(name: str):
    if name == "hangzhou_35806":
        return Hangzhou35806Preset()
    else:
        return None