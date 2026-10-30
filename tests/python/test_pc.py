import importlib,os,struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'PC'))
p=importlib.import_module('missing_pc' if os.environ.get('COURSE_RED') else 'course_protocol')
class ProtocolTests(unittest.TestCase):
    def test_crc(self):self.assertEqual(p.crc(b'123456789'),0x29B1)
    def test_stream(self):
        raw=p.encode(5,0x1234,struct.pack('<ff',1.25,-2.5));self.assertEqual(len(raw),18)
        stream=p.Parser();frames=[]
        for b in b'junk'+raw+raw:frames+=stream.feed(bytes([b]),1)
        self.assertEqual(frames,[(5,0x1234,struct.pack('<ff',1.25,-2.5))]*2)
    def test_bad_and_timeout(self):
        raw=p.encode(1,7);bad=bytearray(raw or b'x'*10);bad[-1]^=1;s=p.Parser()
        self.assertEqual(s.feed(bad,0),[]);self.assertEqual(s.feed(raw[:5],1),[])
        self.assertEqual(s.feed(raw[5:],2),[]);self.assertEqual(s.feed(raw,2.1),[(1,7,b'')])
    def test_limits(self):
        with self.assertRaises(ValueError):p.encode(1,0,b'x'*97)
        with self.assertRaises(ValueError):p.command_values(float('nan'),0)
        with self.assertRaises(ValueError):p.decode_telemetry(b'x'*87)
    def test_telemetry(self):
        raw=bytearray(88);raw[:8]=bytes([1,3,1,0,5,3,2,2]);struct.pack_into('<I',raw,8,1000)
        struct.pack_into('<ffff',raw,48,2.5,.3,.4,.1);struct.pack_into('<I',raw,84,11);t=p.decode_telemetry(raw)
        self.assertEqual(t['pitch_deg'],2.5);self.assertEqual(t['mode'],3);self.assertAlmostEqual(t['x_m'],.4)
        self.assertEqual(t.get('pose_valid'),1);self.assertEqual(t.get('direction_assumed_mask'),1);self.assertEqual(t.get('coasting_mask'),1)
    def test_no_auto_connect(self):
        m=p.MonitorModel();self.assertFalse(m.connected);self.assertEqual(m.pending,{})
        seq,raw=m.command(1);self.assertTrue(raw);self.assertIn(seq,m.pending)
        m.receive((0x80,seq,bytes([1,0,0])));self.assertEqual(m.pending,{})
    def test_replay(self):
        m=p.MonitorModel();self.assertEqual(m.receive((0x90,1,b'bad')),None);self.assertEqual(len(m.samples),0)
        packet=p.demo_telemetry(1);self.assertEqual(len(packet),88);m.receive((0x90,1,packet));self.assertEqual(len(m.samples),1)
    def test_mission_start_carries_arm(self):
        # Firmware disarms an armed-but-idle app after 500 ms (app.c:177), so ARM and
        # START must leave the PC together; two separate operator clicks cannot.
        self.assertEqual(p.mission_start_commands(1,False),[(4,b'\xde\xc0'),(6,bytes([1]))])
        self.assertEqual(p.mission_start_commands(7,True),[(6,bytes([7]))])
        with self.assertRaises(ValueError):p.mission_start_commands(0,False)
        with self.assertRaises(ValueError):p.mission_start_commands(8,True)
    def test_manual_start_carries_arm(self):
        # Same 500 ms rule: the first manual frame has to carry the ARM lock-in.
        body=struct.pack('<ff',0.4,0.4)
        self.assertEqual(p.manual_command_sequence(10,body,False),[(4,b'\xde\xc0'),(10,body)])
        self.assertEqual(p.manual_command_sequence(5,body,True),[(5,body)])
        with self.assertRaises(ValueError):p.manual_command_sequence(7,body,True)
if __name__=='__main__':unittest.main()
