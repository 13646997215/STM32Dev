"""Execute the real C codec and compare both directions with Python."""
import os,struct,subprocess,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'PC'))
from course_protocol import encode,Parser
@unittest.skipIf(bool(os.environ.get('COURSE_RED')),'Separate C/Python interoperability green test')
class InteropTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.exe=ROOT/'build/host/wire_fixture.exe';cls.exe.parent.mkdir(parents=True,exist_ok=True)
        subprocess.run([os.environ.get('COURSE_HOST_CC',str(ROOT.parents[1]/'environment/tcc/win32/tcc.exe')),'-I'+str(ROOT/'App/Inc'),str(ROOT/'tests/host/wire_fixture.c'),str(ROOT/'App/Src/wire.c'),'-o',str(cls.exe)],check=True,capture_output=True)
    def test_c_encoding(self):
        raw=bytes.fromhex(subprocess.check_output([str(self.exe)],text=True).strip());payload=struct.pack('<ff',1.25,-2.5)
        self.assertEqual(raw,encode(5,0x1234,payload));self.assertEqual(Parser().feed(raw,0),[(5,0x1234,payload)])
    def test_c_decoding(self):
        raw=encode(5,0x2345,struct.pack('<ff',-3.75,2.25))
        self.assertEqual(subprocess.check_output([str(self.exe),raw.hex()],text=True).strip(),'5 9029 -3.750 2.250')
