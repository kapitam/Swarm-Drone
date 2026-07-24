from .sectornet import SectorNet8
from .mupydnet_lite import MuPyDNetLite

def build(name: str):
    if name == "sectornet_s":
        return SectorNet8(width=1.0), "bins"
    if name == "sectornet_m":
        return SectorNet8(width=1.5), "bins"
    if name == "mupyd":
        return MuPyDNetLite(), "depth"
    raise ValueError(f"unknown model '{name}' "
                     "(choose: sectornet_s, sectornet_m, mupyd)")
