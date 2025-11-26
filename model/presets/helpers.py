from model.presets.hangzhou_35806 import Hangzhou35806Preset
from model.presets.chongqing_38774 import Chongqing38774Preset
from model.presets.shanghai_56909 import Shanghai56909Preset
from model.presets.yantai_31702 import Yantai31702Preset


def get_preset_instance(name: str):
    if name == "hangzhou_35806":
        return Hangzhou35806Preset()
    elif name == "shanghai_56909":
        return Shanghai56909Preset()
    elif name == "yantai_31702":
        return Yantai31702Preset()
    elif name == "chongqing_38774":
        return Chongqing38774Preset()
    else:
        return None