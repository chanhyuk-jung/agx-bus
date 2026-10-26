import time
from typing import Any, Callable


def poll_until(fn: Callable[[], Any], *, timeout: float, wait: float):
    t0 = time.perf_counter()
    result = fn()
    while not result:
        if time.perf_counter() - t0 >= timeout:
            raise TimeoutError

        time.sleep(wait)

        result = fn()

    return result
