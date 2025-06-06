from typing import Literal

import FreeCAD as App

from freecad.assembly2mujoco.constants import WORKBENCH_NAME


__all__ = ["log_message"]


def log_message(
    message: str,
    *,
    level: Literal["info", "warning", "error"] = "info",
) -> None:
    formatted_message = WORKBENCH_NAME + ": " + message + "\n"
    if level == "info":
        App.Console.PrintMessage(formatted_message)
    elif level == "warning":
        App.Console.PrintWarning(formatted_message)
    elif level == "error":
        App.Console.PrintError(formatted_message)
    else:
        raise ValueError(f"Unrecognized log level '{level}'")
