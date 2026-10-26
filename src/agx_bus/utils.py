import time
from typing import Any, Callable


class AgxBusException(IOError):
    """Base class for agx bus related exceptions."""


class BusNotConnectedError(AgxBusException):
    """Bus is not connected"""

    def __init__(self):
        super().__init__("Attempting to use a bus that is not connected")


class TorqueNotEnabledError(AgxBusException):
    """Torque is not enabled"""

    def __init__(self):
        super().__init__("Attempting to apply torque when torque is not enabled")


class PollingTimeoutException(AgxBusException):
    """Polling timeouts give an exception"""


def poll_until(fn: Callable[[], Any], *, timeout: float, wait: float):
    t0 = time.perf_counter()
    result = fn()
    while not result:
        if time.perf_counter() - t0 >= timeout:
            raise PollingTimeoutException

        time.sleep(wait)

        result = fn()

    return result
