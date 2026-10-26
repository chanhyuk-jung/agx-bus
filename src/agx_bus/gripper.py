from dataclasses import dataclass
from typing import Literal, Self

from .bus import AgilexBus
from .pyAgxArm.protocols.can_protocol.drivers import AgxGripperDriverDefault
from .utils import poll_until


@dataclass(frozen=True)
class Status:
    low_voltage: bool
    motor_overheating: bool
    overcurrent: bool
    overheating: bool
    sensor: bool
    error: bool
    enabled: bool
    homing: bool

    def is_ok(self) -> bool:
        return not any(
            [
                self.low_voltage,
                self.motor_overheating,
                self.overcurrent,
                self.overheating,
                self.sensor,
                self.error,
            ]
        )


@dataclass(frozen=True)
class Gripper:
    timestamp: float

    value: float
    force: float

    mode: Literal["width", "angle"]

    status: Status

    @property
    def width(self) -> float:
        if self.mode == "width":
            return self.value
        else:
            raise ValueError

    @property
    def angle(self) -> float:
        if self.mode == "angle":
            return self.value
        else:
            raise ValueError


class AgilexGripper:
    def __init__(self, ee: AgxGripperDriverDefault, *, timeout: float = 1.0):
        self.ee = ee
        self.timeout = timeout

    @classmethod
    def attach_to(cls: type[Self], bus: AgilexBus, *, timeout: float = 1.0) -> Self:
        ee = bus.arm.init_effector("agx_gripper")
        return cls(ee, timeout=timeout)

    def move(
        self,
        *,
        width: float | None = None,
        angle: float | None = None,
        force: float = 1.0,
    ) -> None:
        if width is not None:
            self.ee.move_gripper_m(width, force)
        elif angle is not None:
            self.ee.move_gripper_deg(angle, force)
        else:
            raise ValueError("either width or angle must be specified")

    def read(self) -> Gripper:
        def read():
            gs = self.ee.get_gripper_status()

            if gs is None:
                return None

            msg = gs.msg

            mode = msg.mode
            if mode not in ["width", "angle"]:
                raise ValueError

            status = msg.foc_status

            status = Status(
                low_voltage=status.voltage_too_low,
                motor_overheating=status.motor_overheating,
                overcurrent=status.driver_overcurrent,
                overheating=status.driver_overheating,
                sensor=status.sensor_status,
                error=status.driver_error_status,
                enabled=status.driver_enable_status,
                homing=status.homing_status,
            )

            return Gripper(
                timestamp=gs.timestamp,
                value=msg.value,
                force=msg.force,
                mode=mode,
                status=status,
            )

        return poll_until(read, timeout=self.timeout, wait=0)

    def set_zero(self):
        self.ee.calibrate_gripper(timeout=self.timeout)
