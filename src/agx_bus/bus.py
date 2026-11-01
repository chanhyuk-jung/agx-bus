from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Literal, cast

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .constants import ArmStatus, ControlStatus, MotionStatus, MoveMode, TeachingStatus
from .pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile
from .pyAgxArm.protocols.can_protocol.drivers import (
    NeroDriverDefault,
    NeroDriverV111,
    PiperDriverDefault,
)
from .utils import (
    AgxBusException,
    BusNotConnectedError,
    TorqueNotEnabledError,
    poll_until,
)


@dataclass(frozen=True)
class State:
    arm: ArmStatus
    control: ControlStatus
    move_mode: MoveMode

    motion: MotionStatus
    teaching: TeachingStatus

    trajectory_index: int

    comm_errors: dict[int, bool]
    limit_exceeds: dict[int, bool]

    @property
    def is_standby(self) -> bool | None:
        if self.control == ControlStatus.STANDBY:
            return True
        elif self.control == ControlStatus.CAN:
            return False

        return None

    @property
    def reached(self) -> bool | None:
        if self.motion == MotionStatus.REACHED:
            return True
        elif self.motion == MotionStatus.MOVING:
            return False

        return None

    @property
    def not_teaching(self) -> bool | None:
        return self.teaching == TeachingStatus.DISABLED

    @property
    def is_arm_ok(self) -> bool:
        return self.arm == ArmStatus.NORMAL

    @property
    def is_comm_ok(self) -> bool:
        return not any(self.comm_errors.values())

    @property
    def limit_exceeded(self) -> bool:
        return any(self.limit_exceeds.values())


@dataclass(frozen=True)
class Joints:
    timestamp: NDArray[np.float64]

    position: NDArray[np.float64]
    velocity: NDArray[np.float64]

    torque: NDArray[np.float64]
    current: NDArray[np.float64]


@dataclass(frozen=True)
class DriverStates:
    timestamp: NDArray[np.float64]

    driver_voltage: NDArray[np.float64]
    driver_temperature: NDArray[np.float64]

    motor_temperature: NDArray[np.float64]
    bus_current: NDArray[np.float64]

    low_voltage: NDArray[np.bool]
    motor_overheating: NDArray[np.bool]

    overcurrent: NDArray[np.bool]
    overheating: NDArray[np.bool]

    collision: NDArray[np.bool]

    error: NDArray[np.bool]

    torque_enabled: NDArray[np.bool]
    stalled: NDArray[np.bool]

    @property
    def is_driver_ok(self) -> bool:
        return not np.any(
            np.stack(
                [
                    self.low_voltage,
                    self.overcurrent,
                    self.overheating,
                    self.collision,
                    self.error,
                    self.stalled,
                ]
            ),
        )

    @property
    def is_motor_ok(self) -> bool:
        return not np.any(self.motor_overheating)

    @property
    def is_torque_enabled(self):
        return np.all(self.torque_enabled).item()


class AgilexBus:
    def __init__(
        self,
        channel: str,
        interface: str = "socketcan",
        bitrate: int = 1_000_000,
        *,
        name: str,
        version: str = "default",
        timeout: float = 1.0,
    ):
        if name == "nero":
            if version not in ["default", "v111", "v112", "v120", "v121"]:
                raise NotImplementedError

            cfg = create_agx_arm_config(
                robot=name,
                firmeware_version="default",
                channel=channel,
                interface=interface,
                bitrate=bitrate,
            )

            self.robot: NeroDriverDefault | NeroDriverV111 = AgxArmFactory.create_arm(
                cfg
            )
        else:
            if name not in ["piper", "piper_h", "piper_l", "piper_x"]:
                raise NotImplementedError

            if version not in ["default", "v183", "v188", "v189"]:
                raise NotImplementedError

            cfg = create_agx_arm_config(
                robot=name,
                firmeware_version=version,
                channel=channel,
                interface=interface,
                bitrate=bitrate,
            )
            self.robot: PiperDriverDefault = AgxArmFactory.create_arm(cfg)

        self.name = name

        self.timeout = timeout

        self.is_connected = False
        self.is_torque_enabled = False

    def connect(self) -> None:
        if self.is_connected:
            raise AgxBusException("Bus is already connected.")

        self.robot.connect()
        self.is_connected = True

        self.robot.set_speed_percent(percent=100)

        motor_status = self.read_drivers()
        self.is_torque_enabled = motor_status.is_torque_enabled

    def disconnect(self) -> None:
        if self.is_connected:
            self.robot.disconnect()

        self.is_connected = False

    def enable_torque(self) -> None:
        if not self.is_connected:
            raise BusNotConnectedError

        def enable():
            result = set()

            for i in range(self.robot.joint_nums):
                result.add(self.robot.enable(self._cast_idx(i + 1)))

            return all(result)

        poll_until(enable, timeout=self.timeout, wait=0.01)
        self.is_torque_enabled = True

    def disable_torque(self) -> None:
        if self.is_torque_enabled:

            def disable():
                result = set()
                for i in range(self.robot.joint_nums):
                    result.add(self.robot.disable(self._cast_idx(i + 1)))

                return not any(result)

            poll_until(disable, timeout=self.timeout, wait=0.01)
            self.is_torque_enabled = False

    def move_joints(self, joints: ArrayLike) -> None:
        if not self.is_torque_enabled:
            raise TorqueNotEnabledError

        joints = np.asarray(joints, dtype=np.float64)
        self.robot.move_j(joints.tolist())

    def _cast_idx(self, index: int):
        if isinstance(self.robot, PiperDriverDefault):
            index = cast(Literal[1, 2, 3, 4, 5, 6], index)
            return index

        elif isinstance(self.robot, NeroDriverDefault):
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
        if not self.is_torque_enabled:
            raise TorqueNotEnabledError

        pos = np.asarray(pos)

        if vel is None:
            vel = np.zeros_like(pos)
        else:
            vel = np.asarray(vel)

        for i in range(self.robot.joint_nums):
            self.robot.move_mit(self._cast_idx(i + 1), pos[i], vel[i], kp, kd, t_ff)

    def read_joints(self) -> Joints:
        if not self.is_connected:
            raise BusNotConnectedError

        def read():
            floats = []
            motor_states = [
                self.robot.get_motor_states(self._cast_idx(i + 1))
                for i in range(self.robot.joint_nums)
            ]

            for ms in motor_states:
                if ms is None:
                    return None

                msg = ms.msg

                floats.append(
                    [ms.timestamp, msg.position, msg.velocity, msg.torque, msg.current]
                )

            floats = np.array(floats).T

            return Joints(
                timestamp=floats[0],
                position=floats[1],
                velocity=floats[2],
                torque=floats[3],
                current=floats[4],
            )

        return poll_until(read, timeout=self.timeout, wait=0)

    def read_state(self) -> State:
        if not self.is_connected:
            raise BusNotConnectedError

        def read():
            status = self.robot.get_arm_status()

            if status is None:
                return None

            msg = status.msg

            error = msg.err_status

            comm_errors = {
                i: getattr(error, f"communication_status_joint_{i + 1}")
                for i in range(self.robot.joint_nums)
            }
            limit_exceeds = {
                i: getattr(error, f"joint_{i + 1}_angle_limit")
                for i in range(self.robot.joint_nums)
            }

            mode = msg.mode_feedback

            # MIT = 0x06 for Piper firmware < v188, Nero firmware < 1.11
            if mode == 0x06:
                mode = MoveMode.MIT

            return State(
                ArmStatus(msg.arm_status),
                ControlStatus(msg.ctrl_mode),
                MoveMode(mode),
                MotionStatus(msg.motion_status),
                TeachingStatus(msg.teach_status),
                msg.trajectory_num,
                comm_errors,
                limit_exceeds,
            )

        return poll_until(read, timeout=self.timeout, wait=0)

    def read_drivers(self) -> DriverStates:
        if not self.is_connected:
            raise BusNotConnectedError

        def read():
            driver_states = [
                self.robot.get_driver_states(self._cast_idx(i + 1))
                for i in range(self.robot.joint_nums)
            ]

            floats = []
            bools = []

            for ds in driver_states:
                if ds is None:
                    return None

                msg = ds.msg

                floats.append(
                    [
                        ds.timestamp,
                        msg.vol,
                        msg.foc_temp,
                        msg.motor_temp,
                        msg.bus_current,
                    ]
                )

                status = msg.foc_status

                bools.append(
                    [
                        status.voltage_too_low,
                        status.motor_overheating,
                        status.driver_overcurrent,
                        status.driver_overheating,
                        status.collision_status,
                        status.driver_error_status,
                        status.driver_enable_status,
                        status.stall_status,
                    ]
                )

            floats = np.array(floats).T
            bools = np.array(bools).T

            return DriverStates(
                **dict(
                    zip(
                        [
                            "timestamp",
                            "driver_voltage",
                            "driver_temperature",
                            "motor_temperature",
                            "bus_current",
                        ],
                        floats,
                    )
                ),
                **dict(
                    zip(
                        [
                            "low_voltage",
                            "motor_overheating",
                            "overcurrent",
                            "overheating",
                            "collision",
                            "error",
                            "torque_enabled",
                            "stalled",
                        ],
                        bools,
                    )
                ),
            )

        return poll_until(read, timeout=self.timeout, wait=0.1)

    def read_info(self):
        info: dict = poll_until(self.robot.get_firmware, timeout=self.timeout, wait=0)

        hw_version = info.pop("hardware_version")
        info["hardware_version"] = hw_version

        fw_version = info.pop("software_version")
        info["firmware_version"] = fw_version
        info["driver_version"] = resolve_firmware_profile(self.name, fw_version)

        return info

    def reset(self):
        if not self.is_connected:
            raise BusNotConnectedError

        self.robot.reset()
        self.is_torque_enabled = False

        time.sleep(1)

        self.robot.set_speed_percent(percent=100)
        self.robot.set_speed_percent(percent=100)

    def e_stop(self):
        if not self.is_connected:
            raise BusNotConnectedError

        self.robot.electronic_emergency_stop()
        self.is_torque_enabled = False

    def clear_errors(self):
        if not self.is_connected:
            raise BusNotConnectedError

        def clear():
            return self.robot.clear_joint_error(255)

        poll_until(clear, timeout=self.timeout, wait=0.1)

    def set_zero(self, index: int) -> None:
        if not self.is_connected:
            raise BusNotConnectedError

        supported_nero = isinstance(self.robot, NeroDriverV111)
        is_piper = isinstance(self.robot, PiperDriverDefault)

        if supported_nero or is_piper:
            self.robot.calibrate_joint(self._cast_idx(index + 1))
            return

        raise NotImplementedError
