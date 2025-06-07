from pathlib import Path
from typing import Literal

WORKBENCH_NAME = "Assembly2MuJoCo"

##########################################
# Paths
##########################################

ROOT_DIR = Path(__file__).parents[2].resolve()
RESOURCES_DIR = ROOT_DIR / "resources"
ICONS_DIR = RESOURCES_DIR / "icons"
WORKBENCH_ICON_FILE = ICONS_DIR / "assembly2mujoco-icon.svg"

##########################################
# MuJoCo
##########################################

MUJOCO_JOINT_TYPE = Literal["hinge", "slide", "ball", "free"]

JOINT_TYPE_MAPPING: dict[str, MUJOCO_JOINT_TYPE] = {
    "Revolute": "hinge",
    "Slider": "slide",
    "Cylindrical": "hinge",
    "Ball": "ball",
    "Planar": "free",
}

# Assign weights to prioritize which joints to keep in the tree
# Higher weight are more likely to be excluded from tree
# Base weight by joint type
JOINT_TYPE_WEIGHTS = {
    "Fixed": 10.0,
    "Revolute": 1.0,
    "Slider": 2.0,
    "Cylindrical": 3.0,
    "Ball": 5.0,
    "Planar": 8.0,
}

##########################################
# Defaults
##########################################

DEFAULT_MESH_LINEAR_DEFLECTION: float = 0.5
DEFAULT_MESH_ANGULAR_DEFLECTION: float = 4.0
DEFAULT_MESH_EXPORT_FORMAT: Literal["STL", "OBJ"] = "OBJ"
DEFAULT_MJCF_TIMESTEP: float = 0.002
DEFAULT_MJCF_DAMPING: float = 10.0
DEFAULT_MJCF_ARMATURE: float = 1.0
DEFAULT_MJCF_INTEGRATOR: Literal["Euler", "implicit", "implicitfast", "RK4"] = (
    "implicitfast"
)
DEFAULT_MJCF_SOLVER: Literal["PGS", "CG", "Newton"] = "Newton"
