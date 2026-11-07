import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('robot_control',Path(__file__).parents[1]/'tools/robot_control.py')
client=importlib.util.module_from_spec(spec);spec.loader.exec_module(client)

class Clock:
    def __init__(self): self.t=0
    def now(self): return self.t
    def sleep(self, t): self.t+=t

class Port:
    def __init__(self, clock, responses): self.clock=clock;self.responses=list(responses);self.writes=[]
    def write(self, data): self.writes.append(data.decode().strip())
    def readline(self):
        self.clock.sleep(.01)
        return (self.responses.pop(0) if self.responses else b'')

class TestControl(unittest.TestCase):
    def robot(self, responses):
        clock=Clock();port=Port(clock,responses)
        return client.Robot(port,clock.now,clock.sleep),port
    def test_banner_status(self):
        r,p=self.robot([b'READY MSP432 CONTROL 1\n', b'STATUS 0 0 3 -2 0 0 0\n'])
        self.assertEqual(r.status()['right'],-2)
    def test_errors_and_timeout(self):
        for response,exc in [(b'ERR\n',RuntimeError),(b'garbage\n',RuntimeError),(b'',TimeoutError)]:
            r,p=self.robot([response])
            with self.assertRaises(exc):r.ok('PING')
    def test_stop_detection(self):
        for status in [b'STATUS 0 0 0 0 0 0 0\n',b'STATUS 1 256 0 0 0 0 0\n',b'STATUS 1 0 0 0 0 1 0\n']:
            r,p=self.robot([b'OK\n',status])
            with self.assertRaises(RuntimeError): r.healthy()
    def test_motion_completion(self):
        r,p=self.robot([b'OK\n',b'OK\n',b'STATUS 1 0 1 1 1 0 0\n',b'OK\n',b'STATUS 1 0 10 10 2 0 0\n'])
        r.move(10,10,1000)
        self.assertEqual(p.writes,['MOVE 10 10 1000','PING','STATUS','PING','STATUS'])
    def test_motion_failure(self):
        r,p=self.robot([b'OK\n',b'OK\n',b'STATUS 1 0 0 0 4 0 0\n'])
        with self.assertRaises(RuntimeError):r.move(10,10,1000)
    def test_feedback_failure_halts_recipe(self):
        r,p=self.robot([b'OK\n',b'STATUS 1 0 0 0 0 0 1\n',b'FEEDBACK 500 600 700\n'])
        with self.assertRaises(RuntimeError):
            client.run_recipe(r,[{'op':'feedback','channel':2,'min':800,'max':900},{'op':'move','left':10,'right':10,'timeout_ms':1000}])
        self.assertFalse(any(x.startswith('MOVE') for x in p.writes))
    def test_recipe_preflight(self):
        cases=[{'calibrated':False,'steps':[{'op':'wait','ms':0}]},
               {'calibrated':True,'steps':[{'op':'pose','index':8,'hold_ms':100}]},
               {'calibrated':True,'steps':[{'op':'move','left':True,'right':0,'timeout_ms':100}]},
               {'calibrated':True,'steps':[{'op':'feedback','channel':2,'min':900,'max':800}]}]
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'recipe.json'
            for case in cases:
                p.write_text(json.dumps(case))
                with self.assertRaises(ValueError):client.load_recipe(p)
            p.write_text(json.dumps({'calibrated':True,'steps':[{'op':'wait','ms':100}]}))
            self.assertEqual(len(client.load_recipe(p)),1)
    def test_shipped_recipes_disabled(self):
        for path in (Path(__file__).parents[1]/'config').glob('*.json'):
            with self.assertRaises(ValueError):client.load_recipe(path)

if __name__=='__main__':unittest.main()
