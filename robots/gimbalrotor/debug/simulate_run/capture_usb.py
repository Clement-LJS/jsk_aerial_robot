#!/usr/bin/env python3
"""Optional USB trace of the selected FTDI adapter. Never opens its tty device.

Requires tcpdump and an already loaded usbmon module. Run separately with sudo
from the same working directory as bench_capture.py; the active run is found
automatically. Ctrl-C stops only tracing.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from capture_paths import usb_run


def locate(serial, root=Path('/sys/bus/usb/devices')):
    found = []
    for path in root.glob('*'):
        try:
            if (path/'idVendor').read_text().strip() != '0403':
                continue
            if (path/'serial').read_text().strip() != serial:
                continue
            found.append(dict(sysfs=str(path), serial=serial,
                              product=(path/'idProduct').read_text().strip(),
                              bus=int((path/'busnum').read_text()),
                              device=int((path/'devnum').read_text())))
        except OSError:
            continue
    if len(found) != 1:
        raise RuntimeError('Expected exactly one connected FTDI adapter with serial ' + serial)
    return found[0]


def capture_command(device, output):
    # LINKTYPE_USB_LINUX_MMAPPED: byte 11 is device address, byte 9 transfer type.
    # Keep bulk transfers (3), both directions and all event types (incl. errors).
    # See https://www.tcpdump.org/linktypes/LINKTYPE_USB_LINUX_MMAPPED.html
    return ['tcpdump', '-i', 'usbmon' + str(device['bus']), '-y', 'USB_LINUX_MMAPPED',
            '-s', '0', '-B', '4096', '-U', '-Z', 'root', '-w', str(output),
            'link[11] = ' + str(device['device']) + ' and link[9] = 3']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        help='Existing run directory (default: find the active ROS capture in/under the current directory)')
    parser.add_argument('--serial', default='AU0JZM07')
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('USB tracing needs privileges: use sudo python3 with this script')
    if not shutil.which('tcpdump'):
        parser.error('tcpdump is not installed')
    if not Path('/sys/module/usbmon').exists():
        parser.error('usbmon is not loaded. Run sudo modprobe usbmon, then retry; no driver was changed by this script')
    try:
        folder = usb_run(args.output)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    device = locate(args.serial)
    stem = datetime.now().strftime('usb-%Y-%m-%d-%H-%M-%S-%f')
    output = folder / (stem + '.pcap')
    metadata = folder / (stem + '.json')
    log_path = folder / (stem + '.log')
    command = capture_command(device, output)
    metadata.write_text(json.dumps(dict(device=device, command=command,
        note='Host USB transfer trace, not MCU-internal execution or electrical UART waveform'), indent=2)+'\n')
    process = None
    print('Recording only FTDI serial ' + args.serial + ' to ' + str(output), flush=True)
    print('USB adapter replug or low disk stops tracing only. Ctrl-C does not stop the robot.', flush=True)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            while process.poll() is None:
                time.sleep(1)
                if shutil.disk_usage(folder).free < 256*1024*1024:
                    print('Low disk: stopping USB trace', flush=True)
                    break
                if locate(args.serial) != device:
                    print('USB address changed: stopping trace to avoid capturing another device', flush=True)
                    break
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print('tcpdump has not finished; inspect its PID/log before disconnecting', flush=True)
        # Let the invoking user read/share artifacts produced by this root process.
        if 'SUDO_UID' in os.environ and 'SUDO_GID' in os.environ:
            uid, gid = int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID'])
            for path in (metadata, log_path, output):
                if path.is_file():
                    os.chown(path, uid, gid)
        print('USB trace log: ' + str(log_path), flush=True)
    if process is not None and process.poll() not in (0, None):
        raise SystemExit('tcpdump failed; inspect ' + str(log_path))


if __name__ == '__main__':
    main()
