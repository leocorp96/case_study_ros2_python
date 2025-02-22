from typing import NamedTuple, List

# Define NamedTuple for ScrewDrive details
class ScrewDrive(NamedTuple):
    type: str
    size: str

# Define NamedTuple for ScrewType details
class ScrewType(NamedTuple):
    drive: ScrewDrive
    length: float
    turns: int

# Define NamedTuple for ScrewPose (Position & Orientation)
class ScrewPose(NamedTuple):
    position: List[float]
    orientation: List[float]

# Define NamedTuple for the entire Screw Data
class ScrewData(NamedTuple):
    screw_type: ScrewType
    poses: List[ScrewPose]