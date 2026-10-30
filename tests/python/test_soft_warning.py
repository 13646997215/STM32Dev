import importlib, pathlib, struct, sys, unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'PC'))
p=importlib.import_module('course_protocol')
smart=importlib.import_module('smart_analysis')

class SoftWarningTests(unittest.TestCase):
 def test_firmware_warning_bits_reach_diagnostic(self):
  data=bytearray(p.demo_telemetry(1))
  struct.pack_into('<I',data,84,(1<<8)|(4<<8))
  decoded=p.decode_telemetry(data)
  self.assertEqual(decoded['warnings'],5)
  self.assertEqual(decoded['pose_valid'],0)
  alerts=smart.SmartAnalyzer().feed(decoded,now=1)['alerts']
  self.assertTrue(any('FG 脉冲异常' in a['title'] for a in alerts))
  self.assertTrue(any('IMU 曾不可用' in a['title'] for a in alerts))

if __name__=='__main__':unittest.main()
