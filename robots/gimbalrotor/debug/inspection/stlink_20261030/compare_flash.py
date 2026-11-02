#!/usr/bin/env python3
"""Offline comparison of the saved ST-Link flash read and existing local builds.

No hardware access, compilation, firmware writes, or ROS interaction.
"""
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p / 'aerial_robot_nerve').is_dir())
BASE = 0x08000000


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    flash = (HERE / 'flash.bin').read_bytes()
    if len(flash) != 0x200000:
        raise ValueError('Expected the complete 2 MiB flash read')
    boards = ROOT / 'aerial_robot_nerve/spinal/mcu_project/boards'
    result = {
        'branch': subprocess.check_output(['git', 'branch', '--show-current'], cwd=ROOT, text=True).strip(),
        'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'flash_bytes': len(flash), 'flash_sha256': sha(flash),
        'flash_reset_vector': hex(struct.unpack_from('<I', flash, 4)[0]),
        'comparisons': [], 'message_md5_strings': {}, 'markers': {},
    }
    for board in ('stm32H7', 'stm32H7_v2'):
        binary = boards / board / 'STM32CubeIDE/Debug/spinal.bin'
        data = binary.read_bytes()
        bad = [i for i, (a, b) in enumerate(zip(flash, data)) if a != b]
        elf = binary.with_suffix('.elf').read_bytes()
        if elf[:6] != b'\x7fELF\x01\x01':
            raise ValueError('Expected ELF32 little endian')
        phoff = struct.unpack_from('<I', elf, 28)[0]
        entsize, count = struct.unpack_from('<HH', elf, 42)
        segments = []
        for index in range(count):
            typ, off, _, addr, size, _, _, _ = struct.unpack_from('<8I', elf, phoff + index * entsize)
            if typ != 1 or not size or not BASE <= addr < BASE + len(flash):
                continue
            actual = flash[addr - BASE:addr - BASE + size]
            expected = elf[off:off + size]
            segments.append({'address': hex(addr), 'bytes': size,
                             'match': actual == expected,
                             'mismatch_bytes': sum(a != b for a, b in zip(actual, expected))})
        result['comparisons'].append({
            'board': board, 'binary': str(binary), 'bytes': len(data),
            'sha256': sha(data), 'match': flash[:len(data)] == data,
            'mismatch_bytes': len(bad),
            'first_mismatch': hex(BASE + bad[0]) if bad else None,
            'reset_vector': hex(struct.unpack_from('<I', data, 4)[0]),
            'elf_segments': segments,
        })
    for name in ('Imu', 'ServoStates', 'ServoState', 'Pwms'):
        header = boards / 'stm32H7_v2/ros_lib/spinal' / (name + '.h')
        md5 = re.search(r'getMD5\(\).*?return "([0-9a-f]+)"', header.read_text()).group(1)
        result['message_md5_strings'][name] = {'local_md5': md5, 'present_in_flash': md5.encode() + b'\0' in flash}
    for marker in ('servo/target_states_debug', 'perching/servo_neutral_mode', 'boards/stm32H7_v2/'):
        result['markers'][marker] = {'present_in_flash': marker.encode() in flash,
            'present_in_saved_h7_v2_binary': marker.encode() in data}
    (HERE / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
