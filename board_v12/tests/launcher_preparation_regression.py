#!/usr/bin/env python3
"""Run the launcher helper against isolated fake modules and /dev/null aliases.

All privileged commands are intercepted. No real kernel module/device is changed.
"""
from pathlib import Path
import os
import subprocess
import tempfile

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/prepare_pcie.sh'

def scenario(name, loaded=None, module=False, devices=1, fail_make=False,
             fail_insert=False, explicit=False, platform=True):
    with tempfile.TemporaryDirectory(prefix='board-launcher-test-') as directory:
        root = Path(directory)
        for folder in ['scripts', 'driver', 'bin', 'sys', 'dev', 'headers']:
            (root / folder).mkdir()
        (root / 'headers/Makefile').touch()
        (root / 'driver/Makefile').touch()
        if module:
            (root / 'driver/my_pciedriver.ko').touch()
        if loaded is not None:
            (root / 'sys/my_pciedriver').mkdir()
            (root / 'sys/my_pciedriver/version').write_text(loaded)
        if platform:
            (root / 'sys/rknpu').mkdir()
            (root / 'dev/rga').symlink_to('/dev/null')
        for index in range(devices):
            (root / ('dev/fpga_frame_' + str(index))).symlink_to('/dev/null')
        script = SOURCE.read_text()
        script = script.replace('PATH=/usr/sbin:/sbin:$PATH', 'PATH="'+str(root / 'bin')+':/usr/bin:/bin"')
        script = script.replace('/sys/module/my_pciedriver', str(root / 'sys/my_pciedriver'))
        script = script.replace('/sys/module/rknpu', str(root / 'sys/rknpu'))
        script = script.replace('/dev/fpga_frame_', str(root / 'dev/fpga_frame_'))
        script = script.replace('/dev/rga', str(root / 'dev/rga'))
        (root / 'scripts/prepare_pcie.sh').write_text(script)
        stub = '''#!/bin/sh
set -eu
name=${0##*/}
printf '%s %s\n' "$name" "$*" >> "$TEST_ROOT/calls"
case "$name" in
  sudo) [ "$1" != -- ] || shift; exec "$@" ;;
  uname) echo test-kernel ;;
  modinfo)
    case "$2" in version) echo 8.2 ;; vermagic) echo 'test-kernel SMP' ;; depends) echo ;; esac ;;
  make) [ "$FAIL_MAKE" = 0 ]; touch "$TEST_ROOT/driver/my_pciedriver.ko" ;;
  insmod)
    [ "$FAIL_INSERT" = 0 ]
    mkdir -p "$TEST_ROOT/sys/my_pciedriver"
    echo 8.2 > "$TEST_ROOT/sys/my_pciedriver/version" ;;
  modprobe)
    case "$1" in
      rga) ln -s /dev/null "$TEST_ROOT/dev/rga" ;;
      rknpu) mkdir -p "$TEST_ROOT/sys/rknpu" ;;
      *) exit 99 ;;
    esac ;;
  udevadm|chgrp|chmod) : ;;
  *) exit 98 ;;
esac
'''
        for command in ['sudo', 'uname', 'modinfo', 'make', 'insmod', 'modprobe', 'udevadm', 'chgrp', 'chmod']:
            p = root / 'bin' / command
            p.write_text(stub)
            p.chmod(0o755)
        env = dict(os.environ, TEST_ROOT=str(root), KERNEL_DIR=str(root / 'headers'),
                   FAIL_MAKE=str(int(fail_make)), FAIL_INSERT=str(int(fail_insert)))
        env.pop('PCIE_DEVICE', None)
        if explicit:
            env['PCIE_DEVICE'] = str(root / 'dev/fpga_frame_1')
        result = subprocess.run(['sh', str(root / 'scripts/prepare_pcie.sh')],
                                env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, timeout=10)
        calls = (root / 'calls').read_text() if (root / 'calls').exists() else ''
        success = loaded in (None, '8.2') and devices > 0 and (devices == 1 or explicit) and not fail_make and not fail_insert
        assert (result.returncode == 0) == success, (name, result.stderr, calls)
        if success:
            expected = root / ('dev/fpga_frame_1' if explicit else 'dev/fpga_frame_0')
            assert result.stdout.strip() == str(expected), (name, result.stdout)
        if loaded is not None:
            assert 'insmod ' not in calls and 'make ' not in calls, (name, calls)
        elif fail_make:
            assert 'insmod ' not in calls, calls
        elif success:
            assert 'insmod ' in calls, calls
            assert ('make ' in calls) == (not module), calls
        assert 'rmmod' not in script
        print('PASS:', name)

scenario('reuse loaded compatible driver', loaded='8.2')
scenario('reject loaded old driver without unloading', loaded='8.0')
scenario('build missing module and load')
scenario('load existing matching module', module=True)
scenario('build failure does not attempt insertion', fail_make=True)
scenario('module insertion failure stops launch', fail_insert=True)
scenario('missing PCIe device stops launch', loaded='8.2', devices=0)
scenario('multiple devices require explicit selection', loaded='8.2', devices=2)
scenario('explicit device with multiple cards', loaded='8.2', devices=2, explicit=True)
scenario('load missing RGA and RKNN platform modules', loaded='8.2', platform=False)
print('All 10 launcher preparation checks passed; no real drivers were changed.')
