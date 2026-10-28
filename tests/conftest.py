import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--can",
        type=str,
        help="PiPER CAN bus channel",
    )

    parser.addoption("--name", type=str, default=None, help="PiPER series name")
    parser.addoption("--firmware", default="default", help="PiPER firmware version")


def pytest_configure(config):
    config.addinivalue_line("markers", "piper")


def pytest_collection_modifyitems(config, items):
    if config.getoption("--can"):
        return

    skip_piper = pytest.mark.skip(reason="Requires PiPER series arm to run")
    for item in items:
        if "piper" in item.keywords:
            item.add_marker(skip_piper)


@pytest.fixture(scope="module")
def channel(request):
    return request.config.getoption("--can")


@pytest.fixture(scope="module")
def name(request):
    return request.config.getoption("--name")


@pytest.fixture(scope="module")
def version(request):
    return request.config.getoption("--firmware")
