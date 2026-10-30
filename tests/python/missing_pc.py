"""Test-only absent-behavior implementation for the recorded red run."""
def crc(data):return 0
def encode(t,s,p=b''):return b''
def command_values(a,b):return b''
def decode_telemetry(data):return {'pitch_deg':0,'mode':0,'x_m':0}
def demo_telemetry(t):return b''
class Parser:
    def feed(self,data,now):return []
class MonitorModel:
    def __init__(self):self.connected=False;self.pending={};self.samples=[]
    def command(self,t):return 1,b''
    def receive(self,f):return None
