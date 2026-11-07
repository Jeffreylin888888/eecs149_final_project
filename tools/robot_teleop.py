#!/usr/bin/env python3
"""Local keyboard teleoperation for the existing MSP432 control firmware."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import signal
import threading
import time
import webbrowser

from robot_control import Robot

UI_TIMEOUT = .35
PWM_LEASE_MS = 200


class Teleop:
    """One serialized owner for serial I/O; browser commands never queue motion."""
    def __init__(self, robot, speed=20, clock=time.monotonic):
        if type(speed) is not int or not 1 <= speed <= 25:
            raise ValueError('Speed must be an integer from 1 to 25 percent')
        self.robot, self.speed, self.clock = robot, speed, clock
        self.lock = threading.RLock()
        self.mode, self.message = 'disarmed', 'Click Enable, then press and release S1.'
        self.owner = None
        self.seen = {}
        self.generation = 0
        self.want = (0, 0)
        self.sent = (0, 0)
        self.last_ui = self.last_io = self.last_pwm = float('-inf')
        self.pending_arm = False
        self.state = {'armed': 0, 'fault': 0, 'left': 0, 'right': 0, 'overruns': 0}

    def start(self):
        self.robot.ok('STOP')
        self.state = self.robot.status()

    def snapshot(self):
        with self.lock:
            return dict(mode=self.mode, message=self.message, speed=self.speed,
                        generation=self.generation, direction=list(self.sent), **self.state)

    def submit(self, data):
        with self.lock:
            if not isinstance(data, dict):
                raise ValueError('Expected an object')
            action, owner, seq = data.get('action'), data.get('client'), data.get('seq')
            if action not in ('arm', 'pulse', 'stop') or not isinstance(owner, str) or not 1 <= len(owner) <= 64 or type(seq) is not int or seq < 0:
                raise ValueError('Invalid control request')
            # A delayed stop is always honored; stale movement can never resume.
            if action == 'stop':
                self.seen[owner] = max(seq, self.seen.get(owner, -1))
                self.stop('Stopped. Click Enable and press S1 to re-arm.')
                return
            if seq <= self.seen.get(owner, -1):
                raise ValueError('Stale command ignored')
            if owner not in self.seen and len(self.seen) >= 32:
                raise ValueError('Restart the control panel session')
            self.seen[owner] = seq
            if action == 'arm':
                if self.mode == 'offline':
                    raise ValueError('Reconnect USB and restart the launcher')
                if self.owner and self.owner != owner:
                    raise ValueError('Another panel controls this session; press Stop first')
                if self.mode in ('waiting', 'armed'):
                    return
                self.owner, self.pending_arm = owner, True
                self.want = (0, 0)
                self.last_ui = self.clock()
                return
            if owner != self.owner:
                raise ValueError('Enable this panel first')
            vector = data.get('direction')
            if not isinstance(vector, list) or len(vector) != 2 or any(type(n) is not int or n not in (-1, 0, 1) for n in vector):
                self.stop('Invalid direction; stopped.')
                raise ValueError('Invalid direction')
            self.last_ui = self.clock()
            # Arming changes generation. A held/stale key from before S1 is discarded.
            self.want = tuple(vector) if self.mode == 'armed' and data.get('generation') == self.generation else (0, 0)

    def stop(self, message, mode='disarmed'):
        self.pending_arm = False
        self.owner = None
        self.want = self.sent = (0, 0)
        self.generation += 1
        self.mode, self.message = mode, message
        self.state['armed'] = 0
        self.robot.ok('STOP')

    def step(self):
        with self.lock:
            now = self.clock()
            try:
                if self.pending_arm:
                    self.pending_arm = False
                    self.robot.ok('STOP')
                    self.robot.ok('CLEAR')
                    self.mode, self.message = 'waiting', 'Press and release S1 on the LaunchPad.'
                    self.last_io = float('-inf')
                if self.mode not in ('waiting', 'armed'):
                    return
                if now - self.last_ui >= UI_TIMEOUT:
                    self.stop('Control panel connection lost; stopped.')
                    return
                if now - self.last_io >= .12:
                    self.robot.ok('PING')
                    self.state = self.robot.status()
                    self.last_io = self.clock()
                    if self.state['fault'] or self.state['overruns']:
                        self.stop(f"Robot fault {self.state['fault']}; release S2/bumps and re-enable.", 'fault')
                        return
                    if self.mode == 'waiting' and self.state['armed']:
                        self.mode, self.message = 'armed', 'Ready. Hold WASD or arrow keys; release to stop.'
                        self.generation += 1
                        self.want = self.sent = (0, 0)
                    elif self.mode == 'armed' and not self.state['armed']:
                        self.stop('Robot disarmed; enable again.', 'fault')
                        return
                # Serial I/O may have consumed the browser lease; recheck before PWM.
                now = self.clock()
                if now - self.last_ui >= UI_TIMEOUT:
                    self.stop('Control panel connection lost; stopped.')
                    return
                if self.mode == 'armed':
                    if self.want != (0, 0) and (self.want != self.sent or now - self.last_pwm >= .10):
                        l, r = (n * self.speed for n in self.want)
                        self.robot.ok(f'PWM {l} {r} {PWM_LEASE_MS}')
                        self.sent, self.last_pwm = self.want, self.clock()
                    elif self.want == (0, 0) and self.sent != (0, 0):
                        self.robot.ok('PWM 0 0 1')
                        self.sent = (0, 0)
            except (OSError, RuntimeError, TimeoutError, UnicodeError) as error:
                try:
                    self.stop('Connection stopped: ' + str(error), 'offline')
                except (OSError, RuntimeError, TimeoutError, UnicodeError):
                    pass  # MCU motor lease and heartbeat expire independently.


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def respond(self, status, data, content_type='application/json'):
        payload = data if isinstance(data, bytes) else json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(payload)

    def valid_host(self):
        return self.headers.get('Host') == self.server.address

    def do_GET(self):
        if not self.valid_host():
            self.respond(403, {'error': 'Local access only'}); return
        if self.path == '/':
            html = (Path(__file__).parent/'teleop.html').read_text().replace('__SESSION_TOKEN__', json.dumps(self.server.token))
            self.respond(200, html.encode(), 'text/html; charset=utf-8')
        elif self.path == '/api/status':
            self.respond(200, self.server.controller.snapshot())
        else:
            self.respond(404, {'error': 'Not found'})

    def do_POST(self):
        if not self.valid_host() or self.headers.get('Origin') != 'http://' + self.server.address or self.headers.get('X-Control-Token') != self.server.token:
            self.respond(403, {'error': 'Control request rejected'}); return
        if self.path != '/api/control':
            self.respond(404, {'error': 'Not found'}); return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 1024 or self.headers.get('Content-Type') != 'application/json':
                raise ValueError('Invalid request body')
            self.server.controller.submit(json.loads(self.rfile.read(size)))
            self.respond(200, self.server.controller.snapshot())
        except (ValueError, UnicodeError) as error:
            self.respond(400, {'error': str(error), **self.server.controller.snapshot()})
        except (OSError, RuntimeError, TimeoutError) as error:
            self.respond(503, {'error': str(error)})


def choose_port():
    from serial.tools import list_ports
    ports = [p.device for p in list_ports.comports() if p.vid == 0x0451 and p.pid == 0xBEF3 and p.device.endswith('1')]
    if len(ports) != 1:
        raise RuntimeError('Specify --port with the XDS110 application serial port')
    return ports[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Default: detect one XDS110 application UART')
    parser.add_argument('--speed', type=int, default=20, choices=range(1,26), metavar='1..25')
    parser.add_argument('--http-port', type=int, default=0, help='0 chooses an available local port')
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    import serial
    port_name = args.port or choose_port()
    done = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    with serial.Serial(port_name, 9600, timeout=.04, write_timeout=.15, exclusive=True) as port:
        port.reset_input_buffer()
        control = Teleop(Robot(port), args.speed)
        control.start()
        server = ThreadingHTTPServer(('127.0.0.1', args.http_port), Handler)
        server.controller, server.token = control, secrets.token_urlsafe(32)
        server.address = f'127.0.0.1:{server.server_port}'
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .1}, daemon=True)
        thread.start()
        url = 'http://' + server.address
        print(f'Teleop ready: {url}\nSerial: {port_name}\nMotors disarmed. Ctrl-C stops and closes the server.', flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        try:
            while not done.wait(.015):
                control.step()
        except KeyboardInterrupt:
            pass
        finally:
            with control.lock:
                try: control.stop('Server closed.')
                except (OSError, RuntimeError, TimeoutError): pass
            server.shutdown(); server.server_close(); thread.join(timeout=1)


if __name__ == '__main__':
    main()
