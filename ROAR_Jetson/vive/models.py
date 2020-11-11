from pydantic import BaseModel, Field


class ViveTrackerMessage(BaseModel):
    valid: bool = Field(default=False)
    x: float = Field(default=0.0)
    y: float = Field(default=0.0)
    z: float = Field(default=0.0)
    roll: float = Field(default=0.0)
    pitch: float = Field(default=0.0)
    yaw: float = Field(default=0.0)
    device_name: str = Field(default="Tracker")
    vel_x: float = Field(default=0.0)
    vel_y: float = Field(default=0.0)
    vel_z: float = Field(default=0.0)

    def __repr__(self):
        return f"device name: {self.device_name} -> " \
               f"x: {round(self.x, 5)} | y: {round(self.y, 5)} | z: {round(self.z, 5)} | " \
               f"pitch: {round(self.pitch, 5)} | yaw: {round(self.yaw, 5)} | roll: {round(self.roll, 5)}"

    def __str__(self):
        return self.__repr__()
