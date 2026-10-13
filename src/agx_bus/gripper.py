import time
from dataclasses import dataclass
from typing import Self

from .bus import AgilexBus
from .pyAgxArm.protocols.can_protocol.drivers import AgxGripperDriverDefault


@dataclass(frozen=True)
class Gripper:
    timestamp: float

    width: float
    force: float


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
        width: float,
        *,
        force: float = 1.0,
    ) -> None:
        self.ee.move_gripper_m(width, force)

    def read(self) -> Gripper:
        def read():
            gs = self.ee.get_gripper_status()

            if gs is None:
                return None

            msg = gs.msg

            return Gripper(
                timestamp=gs.timestamp,
                width=msg.value,
                force=msg.force,
            )

        t0 = time.perf_counter()
        gripper = read()
        while gripper is None:
            if time.perf_counter() - t0 >= self.timeout:
                raise TimeoutError

            time.sleep(0)

            gripper = read()

        return gripper
