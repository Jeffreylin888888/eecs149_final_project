#!/usr/bin/env python3
"""Read live diagnostic telemetry without halt/reset/resume or motion commands."""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / 'firmware/msp432'
STATES = ['WAIT_RELEASE', 'WAIT_PRESS', 'WAIT_START_RELEASE', 'LEFT_RUN',
          'GAP', 'RIGHT_RUN', 'DONE', 'ABORTED']
FIELDS = {'firmware_version': 32, 'diagnostic_status': 32, 'ticks': 32,
          'start_presses': 32, 'motor_runs_started': 32, 'stop_inputs_now': 16,
          'stop_inputs_seen': 16, 'abort_reason': 16}

def reasons(mask):
    names = [f'bump P4.{bit}' for bit in (0, 2, 3, 5, 6, 7) if mask & (1 << bit)]
    if mask & 0x100: names.append('S2/P1.4')
    if mask & 0x200: names.append('motor enable interlock rejected start')
    return ', '.join(names) or 'none'

def main():
    symbols = {}
    for line in subprocess.check_output(['arm-none-eabi-nm', '-n', 'build/wheels.elf'], cwd=ROOT, text=True).splitlines():
        parts = line.split()
        if len(parts) == 3: symbols[parts[2]] = int(parts[0], 16)
    image = (ROOT / 'build/wheels.bin').read_bytes()
    (ROOT / 'logs').mkdir(exist_ok=True)
    readback = ROOT / 'logs/status-readback.bin'
    # Remove an earlier observation so a failed read cannot appear successful.
    readback.unlink(missing_ok=True)
    cmd = ['openocd', '-f', 'openocd.cfg', '-c', 'init', '-c',
           f'dump_image logs/status-readback.bin 0 {len(image)}']
    for name, width in FIELDS.items():
        cmd += ['-c', f'echo "STATUS {name} [lindex [read_memory 0x{symbols[name]:x} {width} 1] 0]"']
    cmd += ['-c', 'shutdown']
    run = subprocess.run(cmd, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
    (ROOT / 'logs/status-live.log').write_text(run.stdout)
    if run.returncode or not readback.exists():
        raise SystemExit(run.stdout)
    if readback.read_bytes() != image:
        raise SystemExit('Board image differs from the current wheel build. Refusing to decode using mismatched symbol addresses. See logs/status-live.log.')
    data = {name: int(value, 0) for name, value in re.findall(r'^STATUS (\w+) (\S+)$', run.stdout, re.M)}
    if data.keys() != FIELDS.keys(): raise SystemExit('Incomplete telemetry; inspect logs/status-live.log.')
    state = data['diagnostic_status']
    print('Flash readback matches current build. Live samples (not an atomic snapshot):')
    print('State:', STATES[state] if state < len(STATES) else f'INVALID {state}')
    print('Start-button edges:', data['start_presses'])
    print('Motor commands started (not movement evidence):', data['motor_runs_started'], '(bit0=left, bit1=right)')
    for field in ('stop_inputs_now', 'stop_inputs_seen', 'abort_reason'):
        print(field + ':', reasons(data[field]))
    print('Firmware version:', data['firmware_version'], 'uptime ms:', data['ticks'])

if __name__ == '__main__':
    main()
