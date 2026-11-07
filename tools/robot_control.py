#!/usr/bin/env python3
"""MSP432 serial client. No motion on connection; physical S1 arms each session."""
import argparse
import json
import time
from pathlib import Path


class Robot:
    def __init__(self, port, clock=time.monotonic, sleep=time.sleep):
        self.port, self.clock, self.sleep = port, clock, sleep

    def exchange(self, command):
        self.port.write((command + '\n').encode('ascii'))
        deadline = self.clock() + .3
        while self.clock() < deadline:
            line = self.port.readline().decode('ascii', errors='strict').strip()
            if line.startswith('READY') or not line:
                continue
            if line == 'ERR':
                raise RuntimeError('Firmware rejected: ' + command)
            if line == 'OK' or line.startswith(('STATUS ', 'FEEDBACK ')):
                return line
            raise RuntimeError('Unexpected serial response: ' + repr(line))
        raise TimeoutError('No response to ' + command)

    def ok(self, command):
        if self.exchange(command) != 'OK':
            raise RuntimeError('Protocol mismatch for ' + command)

    def status(self):
        result = self.exchange('STATUS').split()
        if len(result) != 8 or result[0] != 'STATUS':
            raise RuntimeError('Unexpected firmware version/status')
        return dict(zip(('armed', 'fault', 'left', 'right', 'move', 'overruns', 'settled'), map(int, result[1:])))

    def healthy(self):
        self.ok('PING')
        state = self.status()
        if not state['armed'] or state['fault'] or state['overruns']:
            raise RuntimeError('Robot stopped: ' + str(state))
        return state

    def wait(self, seconds):
        until = self.clock() + seconds
        while self.clock() < until:
            self.healthy()
            self.sleep(min(.05, max(0, until-self.clock())))

    def arm(self):
        self.ok('STOP')
        self.ok('CLEAR')
        print('Release S1, then press and release S1 to arm. S2 or any bump stops motion.', flush=True)
        until = self.clock() + 30
        while self.clock() < until:
            self.ok('PING')
            s = self.status()
            if s['fault']:
                raise RuntimeError('Fault while arming: ' + str(s))
            if s['armed']:
                return
            self.sleep(.05)
        raise TimeoutError('Physical S1 arming timed out')

    def move(self, left, right, timeout):
        self.ok(f'MOVE {left} {right} {timeout}')
        until = self.clock() + timeout/1000 + .5
        while self.clock() < until:
            s = self.healthy()
            if s['move'] == 2:
                return
            if s['move'] != 1:
                raise RuntimeError('Move failed: ' + str(s))
            self.sleep(.05)
        raise TimeoutError('Move completion not received')

    def pose(self, index, hold_ms):
        self.ok(f'POSE {index}')
        until = self.clock() + 10
        while self.clock() < until:
            if self.healthy()['settled']:
                self.wait(hold_ms/1000)
                return
            self.sleep(.05)
        raise TimeoutError('Servo pulse slew timed out')


def integer(v, low, high):
    return type(v) is int and low <= v <= high


def load_recipe(path):
    data = json.loads(Path(path).read_text())
    if data.get('calibrated') is not True:
        raise ValueError('Recipe is disabled until physical calibration is recorded')
    steps = data.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= 100:
        raise ValueError('Recipe needs 1..100 steps')
    for step in steps:
        if not isinstance(step, dict):
            raise ValueError('Step must be an object')
        op = step.get('op')
        if op == 'pose':
            valid = integer(step.get('index'), 0, 4) and integer(step.get('hold_ms'), 100, 5000)
        elif op == 'move':
            valid = all(integer(step.get(k), -100000, 100000) for k in ('left', 'right')) and integer(step.get('timeout_ms'), 100, 30000)
        elif op == 'feedback':
            valid = integer(step.get('channel'), 0, 2) and integer(step.get('min'), 0, 16383) and integer(step.get('max'), 0, 16383) and step['min'] <= step['max']
        elif op == 'wait':
            valid = integer(step.get('ms'), 0, 5000)
        else:
            valid = False
        if not valid:
            raise ValueError('Invalid step: ' + repr(step))
    return steps


def run_recipe(robot, steps):
    for step in steps:
        robot.healthy()
        op = step['op']
        if op == 'pose':
            robot.pose(step['index'], step['hold_ms'])
        elif op == 'move':
            robot.move(step['left'], step['right'], step['timeout_ms'])
        elif op == 'wait':
            robot.wait(step['ms']/1000)
        else:
            fields = robot.exchange('FEEDBACK').split()
            if len(fields) != 4 or fields[0] != 'FEEDBACK':
                raise RuntimeError('Invalid feedback response')
            value = int(fields[1 + step['channel']])
            if not step['min'] <= value <= step['max']:
                raise RuntimeError('Feedback outside calibrated range; stopping sequence')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='XDS110 application/backchannel serial port')
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('status')
    sub.add_parser('lab12', help='20%% forward/backward/left/right, 200 ms each; wheels lifted')
    p = sub.add_parser('pwm'); p.add_argument('left', type=int); p.add_argument('right', type=int); p.add_argument('ms', type=int)
    p = sub.add_parser('move'); p.add_argument('left', type=int); p.add_argument('right', type=int); p.add_argument('ms', type=int)
    p = sub.add_parser('servo'); p.add_argument('channel', type=int); p.add_argument('us', type=int)
    p = sub.add_parser('pose'); p.add_argument('index', type=int)
    p = sub.add_parser('recipe'); p.add_argument('file')
    args = parser.parse_args()
    steps = load_recipe(args.file) if args.action == 'recipe' else None
    if args.action == 'pwm' and not (integer(args.left,-25,25) and integer(args.right,-25,25) and integer(args.ms,1,250)):
        parser.error('PWM bounds: +/-25 percent and 1..250 ms')
    if args.action == 'move' and not (integer(args.left,-100000,100000) and integer(args.right,-100000,100000) and integer(args.ms,100,30000)):
        parser.error('MOVE bounds: +/-100000 ticks and 100..30000 ms')
    if args.action == 'servo' and not (integer(args.channel,0,2) and integer(args.us,400,2600)):
        parser.error('Invalid servo channel/pulse')
    if args.action == 'pose' and not integer(args.index,0,4):
        parser.error('Pose index must be 0..4')
    import serial  # Optional dependency; unit tests need no serial installation.
    with serial.Serial(args.port, 9600, timeout=.04, write_timeout=.15) as port:
        port.reset_input_buffer()
        robot = Robot(port)
        try:
            if args.action == 'status':
                print(robot.status()); return
            robot.arm()
            if args.action == 'lab12':
                for name,l,r in [('forward',20,20),('backward',-20,-20),('left',-20,20),('right',20,-20)]:
                    print(name, flush=True)
                    robot.ok(f'PWM {l} {r} 200'); robot.wait(1.2)
            elif args.action == 'pwm':
                robot.ok(f'PWM {args.left} {args.right} {args.ms}'); robot.wait(args.ms/1000+.2)
            elif args.action == 'move':
                robot.move(args.left,args.right,args.ms)
            elif args.action == 'servo':
                robot.ok(f'SERVO {args.channel} {args.us}'); robot.wait(5)
            elif args.action == 'pose':
                robot.pose(args.index,500)
            else:
                run_recipe(robot,steps)
        finally:
            # If USB disappears, firmware's 500 ms watchdog independently stops.
            try: robot.ok('STOP')
            except (OSError, RuntimeError, TimeoutError): pass


if __name__ == '__main__':
    main()
