from __future__ import annotations

from enum import IntEnum


class ControlStatus(IntEnum):
    STANDBY = 0x00
    CAN = 0x01
    TEACHING = 0x02
    ETHERNET = 0x03
    WIFI = 0x04
    REMOTE = 0x05
    LINKAGE_TEACHING_INPUT = 0x06
    OFFLINE_TRAJECTORY = 0x07
    TCP = 0x08  # Only supported on Nero
    UNKNOWN = 0xFF


class ArmStatus(IntEnum):
    NORMAL = 0x00
    EMERGENCY_STOP = 0x01
    NO_SOLUTION = 0x02
    SINGULARITY_POINT = 0x03
    TARGET_POS_EXCEEDS_LIMIT = 0x04
    JOINT_COMMUNICATION_ERR = 0x05
    JOINT_BRAKE_NOT_RELEASED = 0x06
    COLLISION_OCCURRED = 0x07
    OVERSPEED_DURING_TEACHING_DRAG = 0x08
    JOINT_STATUS_ERR = 0x09
    OTHER_ERROR = 0x0A
    TEACHING_RECORD = 0x0B
    TEACHING_EXECUTION = 0x0C
    TEACHING_PAUSE = 0x0D
    MAIN_CONTROLLER_NTC_OVER_TEMPERATURE = 0x0E
    RELEASE_RESISTOR_NTC_OVER_TEMPERATURE = 0x0F
    UNKNOWN = 0xFF


class MoveMode(IntEnum):
    POSITION = 0x00
    JOINT = 0x01
    LINE = 0x02
    CURVE = 0x03
    MIT = 0x04  # Piper firmware < v188, Nero firmware < 1.11
    CPV = 0x05
    UNKNOWN = 0xFF


class TeachingStatus(IntEnum):
    DISABLED = 0x00  # Stop
    START_RECORDING = 0x01  # Start recording (enter drag-to-teach mode)
    STOP_RECORDING = 0x02  # Stop recording (exit drag-to-teach mode)
    EXECUTE_TRAJECTORY = 0x03  # Execute trajectory (replay recorded trajectory)
    PAUSE_EXECUTION = 0x04  # Pause execution
    RESUME_EXECUTION = 0x05  # Resume execution (continue trajectory replay)
    TERMINATE_EXECUTION = 0x06  # Terminate execution
    MOVE_TO_START = 0x07  # Move to trajectory start point
    UNKNOWN = 0xFF


class MotionStatus(IntEnum):
    REACHED = 0x00
    MOVING = 0x01
    UNKNOWN = 0xFF
