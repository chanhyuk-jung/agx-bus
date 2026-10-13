import time
from dataclasses import dataclass
from enum import IntEnum, unique
from typing import Literal, cast

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .pyAgxArm import AgxArmFactory, create_agx_arm_config
from .pyAgxArm.protocols.can_protocol.drivers import (
    NeroDriverDefault,
    NeroDriverV111,
    PiperDriverDefault,
)


class ControlType(IntEnum):
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


class MoveType(IntEnum):
    POSITION = 0x00
    JOINT = 0x01
    LINE = 0x02
    CURVE = 0x03
    MIT = 0x04  # Piper firmware < v188, Nero firmware < 1.11
    CPV = 0x05
    UNKNOWN = 0xFF


class NewMoveType(IntEnum):
    POSITION = 0x00
    JOINT = 0x01
    LINE = 0x02
    CURVE = 0x03
    CPV = 0x05
    MIT = 0x06
    UNKNOWN = 0xFF


@unique
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


@unique
class MotionStatus(IntEnum):
    REACHED = 0x00
    MOVING = 0x01
    UNKNOWN = 0xFF


@dataclass(frozen=True)
class Status:
    arm: ArmStatus
    control: ControlType
    move: MoveType

    motion: MotionStatus
    teaching: TeachingStatus

    comm_errors: dict[int, bool]
    limit_exceeds: dict[int, bool]

    def is_standby(self) -> bool | None:
        if self.control == ControlType.STANDBY:
            return True
        elif self.control == ControlType.CAN:
            return False

        return None

    def reached(self) -> bool | None:
        if self.move == MotionStatus.REACHED:
            return True
        elif self.move == MotionStatus.MOVING:
            return False

        return None

    def comm_ok(self) -> bool:
        return not any(self.comm_errors)

    def limit_exceeded(self) -> bool:
        return any(self.limit_exceeds)


@dataclass(frozen=True)
class Joints:
    timestamp: NDArray[np.float64]

    position: NDArray[np.float64]
    velocity: NDArray[np.float64]

    torque: NDArray[np.float64]
    current: NDArray[np.float64]


class AgilexBus:
    def __init__(self, channel: str, *, name: str, version: str, timeout: float = 1.0):
        if name == "nero":
            if version not in ["default", "v111", "v112", "v120", "v121"]:
                raise NotImplementedError

            cfg = create_agx_arm_config(
                robot=name, firmeware_version=version, channel=channel
            )

            self.arm: NeroDriverDefault | NeroDriverV111 = AgxArmFactory.create_arm(cfg)
        else:
            if name not in ["piper", "piper_h", "piper_l", "piper_x"]:
                raise NotImplementedError

            if version not in ["default", "v183", "v188", "v189"]:
                raise NotImplementedError

            cfg = create_agx_arm_config(
                robot=name, firmeware_version=version, channel=channel
            )
            self.arm: PiperDriverDefault = AgxArmFactory.create_arm(cfg)

        self.timeout = timeout

    def connect(self) -> None:
        self.arm.connect()

        self.arm.set_speed_percent(percent=100)
        time.sleep(0.1)

    def disconnect(self) -> None:
        self.arm.disconnect()

    def enable_torque(self) -> None:
        def enable():
            result = set()

            for i in range(self.arm.joint_nums):
                result.add(self.arm.enable(self._cast_idx(i + 1)))

            enabled = False not in result
            return enabled

        t0 = time.perf_counter()

        enabled = enable()
        while not enabled:
            if time.perf_counter() - t0 >= self.timeout:
                raise TimeoutError

            time.sleep(0.01)

            enabled = enable()

    def disable_torque(self) -> None:
        def disable():
            result = set()
            for i in range(self.arm.joint_nums):
                result.add(self.arm.disable(self._cast_idx(i + 1)))

            disabled = False not in result
            return disabled

        t0 = time.perf_counter()

        disabled = disable()
        while not disabled:
            if time.perf_counter() - t0 >= self.timeout:
                raise TimeoutError

            time.sleep(0.01)

            disabled = disable()

    def move_joints(self, joints: ArrayLike) -> None:
        joints = np.asarray(joints, dtype=np.float64)
        self.arm.move_j(list(joints))

    def _cast_idx(self, index: int):
        if isinstance(self.arm, PiperDriverDefault):
            index = cast(Literal[1, 2, 3, 4, 5, 6], index)
            return index

        elif isinstance(self.arm, NeroDriverDefault):
            index = cast(Literal[1, 2, 3, 4, 5, 6, 7], index)
            return index

    def move_mit(
        self,
        pos: ArrayLike,
        vel: ArrayLike | None = None,
        kp: float = 10.0,
        kd: float = 0.5,
        t_ff: float = 0.0,
    ):
        pos = np.asarray(pos)

        if vel is None:
            vel = np.zeros_like(pos)
        else:
            vel = np.asarray(vel)

        for i in range(self.arm.joint_nums):
            self.arm.move_mit(self._cast_idx(i + 1), pos[i], vel[i], kp, kd, t_ff)

    def read_joints(self) -> Joints:
        def read():
            timestamp: list[float] = []
            position: list[float] = []
            velocity: list[float] = []
            torque: list[float] = []
            current: list[float] = []

            motor_states = [
                self.arm.get_motor_states(self._cast_idx(i + 1))
                for i in range(self.arm.joint_nums)
            ]

            for ms in motor_states:
                if ms is None:
                    return None

                timestamp.append(ms.timestamp)

                msg = ms.msg

                position.append(msg.position)
                velocity.append(msg.velocity)
                torque.append(msg.torque)
                current.append(msg.current)

            def float64(x: ArrayLike):
                return np.asarray(x, dtype=np.float64)

            return Joints(
                float64(timestamp),
                float64(position),
                float64(velocity),
                float64(torque),
                float64(current),
            )

        t0 = time.perf_counter()

        joints = read()
        while joints is None:
            if time.perf_counter() - t0 >= self.timeout:
                raise TimeoutError

            time.sleep(0)

            joints = read()

        return joints

    def read_status(self) -> Status:
        def read():
            status = self.arm.get_arm_status()

            if status is None:
                return None

            msg = status.msg

            error = msg.err_status

            comm_errors = {
                i: getattr(error, f"communication_status_joint_{i + 1}")
                for i in range(self.arm.joint_nums)
            }
            limit_exceeds = {
                i: getattr(error, f"joint_{i + 1}_angle_limit")
                for i in range(self.arm.joint_nums)
            }

            mode = msg.mode_feedback
            if mode == 0x06:
                mode = MoveType.MIT

            return Status(
                ArmStatus(msg.arm_status),
                ControlType(msg.ctrl_mode),
                MoveType(mode),
                MotionStatus(msg.motion_status),
                TeachingStatus(msg.teach_status),
                comm_errors,
                limit_exceeds,
            )

        t0 = time.perf_counter()

        status = read()
        while status is None:
            if time.perf_counter() - t0 >= self.timeout:
                raise TimeoutError

            time.sleep(0)

            status = read()

        return status

    def reset(self):
        self.arm.reset()

    def e_stop(self):
        self.arm.electronic_emergency_stop()

    def calibrate(self, index: int) -> None:
        supported_nero = isinstance(self.arm, NeroDriverV111)
        is_piper = isinstance(self.arm, PiperDriverDefault)

        if supported_nero or is_piper:
            self.arm.calibrate_joint(self._cast_idx(index + 1))
            return

        raise NotImplementedError
