from dataclasses import dataclass

from src.simulation.fleet.hardware.fake_GPS_driver import fake_USV_gps
from src.simulation.fleet.hardware.fake_IMU_driver import fake_USV_imu
from src.simulation.fleet.hardware.fake_USV_motor_driver import fake_USV_ardu
from src.simulation.fleet.vehicles.USV import USV_body
from src.autonomy.brain import USV_brain

class fleet:
    def __init__(self, dict_of_assets: dict):
        self.dict_of_assets = dict_of_assets
        self.bodies = dict((asset.id, asset.usv_body) for asset in self.dict_of_assets.values())
        self.brains = dict((asset.id, asset.usv_brain) for asset in self.dict_of_assets.values())


@dataclass
class USV_system:
    id: str
    type: str
    usv_brain: USV_brain
    usv_body: USV_body
    usv_gnss: fake_USV_gps
    usv_imu: fake_USV_imu
    usv_motor:fake_USV_ardu