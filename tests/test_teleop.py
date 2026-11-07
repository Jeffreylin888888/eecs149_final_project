import http.client
import importlib.util
import json
from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]/'tools'))
from robot_teleop import Teleop, Handler, ThreadingHTTPServer


class Clock:
    def __init__(self): self.t = 0.
    def now(self): return self.t
    def advance(self, n): self.t += n


class Robot:
    def __init__(self):
        self.commands = []
        self.state = dict(armed=0, fault=0, left=0, right=0, move=0, overruns=0, settled=0)
        self.fail_pwm = False
        self.on_status = lambda: None
    def ok(self, cmd):
        self.commands.append(cmd)
        if cmd == 'STOP': self.state['armed'] = 0
        if cmd == 'CLEAR': self.state['fault'] = 0
        if self.fail_pwm and cmd.startswith('PWM'): raise OSError('USB unplugged')
    def status(self):
        self.on_status()
        return dict(self.state)


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock, self.robot = Clock(), Robot()
        self.c = Teleop(self.robot, clock=self.clock.now)
        self.seq = 0
        self.c.start()
    def send(self, action, direction=(0,0), **extra):
        self.seq += 1
        self.c.submit(dict(action=action, client='test', seq=self.seq, direction=list(direction), generation=self.c.generation, **extra))
    def arm(self):
        self.send('arm'); self.c.step()
        self.robot.state['armed'] = 1
        self.clock.advance(.13); self.send('pulse'); self.c.step()
        self.assertEqual(self.c.mode, 'armed')
    def test_no_motion_before_physical_arm(self):
        self.send('arm'); self.c.step(); self.send('pulse',(1,1)); self.c.step()
        self.assertFalse(any(x.startswith('PWM') for x in self.robot.commands))
    def test_vectors_release_and_stop(self):
        self.arm()
        for vector,cmd in [((1,1),'PWM 20 20 200'),((-1,-1),'PWM -20 -20 200'),((-1,1),'PWM -20 20 200'),((1,-1),'PWM 20 -20 200')]:
            self.send('pulse',vector);self.c.step();self.assertEqual(self.robot.commands[-1],cmd)
        self.send('pulse');self.c.step();self.assertEqual(self.robot.commands[-1],'PWM 0 0 1')
        self.assertEqual(self.c.mode,'armed')
        self.send('stop');self.assertEqual(self.robot.commands[-1],'STOP');self.assertEqual(self.c.mode,'disarmed')
    def test_browser_loss_disarms(self):
        self.arm();self.send('pulse',(1,1));self.c.step();self.clock.advance(.351);self.c.step()
        self.assertEqual(self.robot.commands[-1],'STOP');self.assertEqual(self.c.sent,(0,0))
        self.assertEqual(self.c.mode,'disarmed')
    def test_expiry_during_serial_call_does_not_enable(self):
        self.arm();self.clock.advance(.13);self.send('pulse',(1,1))
        self.robot.on_status=lambda: self.clock.advance(.36)
        self.c.step();self.assertFalse(any(x.startswith('PWM') for x in self.robot.commands))
        self.assertEqual(self.robot.commands[-1],'STOP')
    def test_old_generation_discarded(self):
        self.arm();self.seq+=1
        self.c.submit(dict(action='pulse',client='test',seq=self.seq,generation=self.c.generation-1,direction=[1,1]))
        self.c.step();self.assertFalse(any(x.startswith('PWM') for x in self.robot.commands))
    def test_stale_motion_after_stop_rejected(self):
        self.arm();old=self.seq;self.send('stop')
        with self.assertRaises(ValueError):self.c.submit(dict(action='pulse',client='test',seq=old,generation=self.c.generation,direction=[1,1]))
        self.c.step();self.assertEqual(self.c.mode,'disarmed')
    def test_other_panel_cannot_take_over(self):
        self.arm()
        with self.assertRaises(ValueError):self.c.submit(dict(action='arm',client='second',seq=1))
    def test_invalid_vector_stops(self):
        self.arm()
        with self.assertRaises(ValueError):self.send('pulse',(True,1))
        self.assertEqual(self.robot.commands[-1],'STOP')
    def test_serial_failure_disarms_no_auto_reconnect(self):
        self.arm();self.robot.fail_pwm=True;self.send('pulse',(1,1));self.c.step()
        self.assertEqual(self.c.mode,'offline');self.assertEqual(self.robot.commands[-1],'STOP')
        with self.assertRaises(ValueError):self.send('arm')
    def test_hardware_fault_disarms(self):
        self.arm();self.clock.advance(.13);self.robot.state['fault']=256;self.send('pulse',(1,1));self.c.step()
        self.assertEqual(self.c.mode,'fault');self.assertEqual(self.robot.commands[-1],'STOP')
    def test_speed_bounds(self):
        for value in (0,26,True):
            with self.assertRaises(ValueError):Teleop(self.robot,value)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.c=Teleop(Robot())
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.server.address=f'127.0.0.1:{self.server.server_port}'
        self.server.token='test-token';self.server.controller=self.c
        self.thread=threading.Thread(target=self.server.serve_forever,kwargs={'poll_interval':.01},daemon=True);self.thread.start()
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()
    def request(self, method, path, data=None, headers=None):
        conn=http.client.HTTPConnection('127.0.0.1',self.server.server_port,timeout=2)
        conn.request(method,path,body=json.dumps(data) if data is not None else None,headers=headers or {})
        r=conn.getresponse();result=(r.status,r.read());conn.close();return result
    def test_page_and_status(self):
        code,body=self.request('GET','/');self.assertEqual(code,200);self.assertIn(b'test-token',body)
        code,body=self.request('GET','/api/status');self.assertEqual(json.loads(body)['mode'],'disarmed')
    def test_cross_origin_token_and_host_guards(self):
        body=dict(action='arm',client='test',seq=1)
        base={'Origin':'http://'+self.server.address,'X-Control-Token':'test-token','Content-Type':'application/json'}
        for override in ({'Origin':'https://example.com'},{'X-Control-Token':'bad'},{'Host':'evil.example'}):
            self.assertEqual(self.request('POST','/api/control',body,{**base,**override})[0],403)
        self.assertFalse(self.c.pending_arm)
        self.assertEqual(self.request('POST','/api/control',body,base)[0],200)
        self.assertTrue(self.c.pending_arm)
    def test_bad_body_rejected(self):
        headers={'Origin':'http://'+self.server.address,'X-Control-Token':'test-token','Content-Type':'application/json'}
        self.assertEqual(self.request('POST','/api/control',{'action':'run'},headers)[0],400)

if __name__=='__main__':unittest.main()
