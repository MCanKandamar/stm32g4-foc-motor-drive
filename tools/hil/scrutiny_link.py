"""Scrutiny helpers: connect the Scrutiny server to the board and list watchable paths.

The Scrutiny GUI/server is v0.16.0; the global Python may have an old `scrutinydebugger`
that cannot talk to it, so the SDK is loaded from the wheel in the Scrutiny install folder.
The Scrutiny GUI (or `scrutiny server`) must be running on 127.0.0.1:8765.
"""
import sys

sys.path.insert(0, 'C:/Program Files (x86)/Scrutiny Debugger/scrutinydebugger-nocli-0.16.0-py3-none-any.whl')

from scrutiny import sdk                         # noqa: E402
from scrutiny.sdk.client import ScrutinyClient   # noqa: E402

BOARD_PORT = 'COM6'          # board USB CDC (VID 0x0483, PID 0x5710)


def connect():
    """Connect to the server; if it has no link to the board yet, open the serial link."""
    c = ScrutinyClient()
    c.connect('127.0.0.1', 8765)
    c.wait_server_status_update(3)
    if c.get_latest_server_status().device_comm_state != sdk.DeviceCommState.ConnectedReady:
        c.configure_device_link(sdk.DeviceLinkType.Serial,
                                sdk.SerialLinkConfig(port=BOARD_PORT, baudrate=115200, start_delay=0.0))
    c.wait_device_ready(15)
    return c


def all_paths(c):
    req = c.download_watchable_list()
    req.wait_for_completion(60)
    d = req.get()
    allp = list(d.var) + list(d.alias) + list(d.rpv)
    for f in d.var_factory.values():             # arrays: one path per element
        allp += [p for p, _ in f.iterate_possible_paths()]
    return allp


if __name__ == '__main__':
    # python tools/hil/scrutiny_link.py [text]: print the paths that contain the text
    key = sys.argv[1] if len(sys.argv) > 1 else ''
    c = connect()
    for p in sorted(all_paths(c)):
        if key in p:
            print(p)
    print(c.get_device_info().datalogging_capabilities)
    c.disconnect()
