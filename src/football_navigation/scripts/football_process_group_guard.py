#!/usr/bin/python3
"""Terminate a launch process group if its launch parent disappears abruptly."""

import os
import signal
import sys
import time


def main():
    parent_pid = os.getppid()
    process_group = os.getpgrp()

    def normal_shutdown(_signum, _frame):
        sys.exit(0)

    signal.signal(signal.SIGINT, normal_shutdown)
    signal.signal(signal.SIGTERM, normal_shutdown)

    while True:
        if parent_pid <= 1 or os.getppid() != parent_pid:
            # The launch process was killed before it could reap its children.
            # Ignore our own TERM long enough to escalate stubborn members.
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                os.killpg(process_group, signal.SIGTERM)
            except ProcessLookupError:
                return 0
            time.sleep(2.0)
            try:
                os.killpg(process_group, signal.SIGKILL)
            except ProcessLookupError:
                pass
            return 0
        time.sleep(0.25)


if __name__ == '__main__':
    raise SystemExit(main())
