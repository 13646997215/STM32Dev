"""Binary protocol/model shared by the GUI and offline tests. No serial I/O here."""
from collections import deque
import math,struct,time
MAGIC=b'\xa7\xc3'
PROFILES={1:'任务1：Pitch调平到零（本轮目标）',2:'任务2：自主上板',3:'任务3/4：平衡10s后返A',4:'任务5：带一砝码平衡并返回',5:'任务6：平衡后等待扰动再平衡',6:'预测评：倾覆后5s返A',7:'附加7：顺时针及逆时针矩形'}
STATES=['IDLE','ALIGN','APPROACH','CLIMB','SEEK','BALANCE','HOLD','RETURN','ARRIVED','PATH','DOCK','DONE','FAULT']
FAULTS=['NONE','CONFIG','IMU','FEEDBACK','TIMING','BOUNDARY','TIMEOUT','TILT','MOTOR','UART','STOPPED','POSE']
RESULTS=['OK','FORMAT','RANGE','LOCKED','NOT_READY','DUPLICATE','FAULT']
def crc(data):
    result=0xffff
    for b in data:
        result^=b<<8
        for _ in range(8):result=((result<<1)^0x1021 if result&0x8000 else result<<1)&0xffff
    return result
def encode(kind,seq,payload=b''):
    if len(payload)>96 or not 0<=kind<=255 or not 0<=seq<=65535:raise ValueError('Frame range')
    data=MAGIC+struct.pack('<BBHH',1,kind,len(payload),seq)+payload
    return data+struct.pack('<H',crc(data))
def command_values(a,b):
    if not all(math.isfinite(x) and abs(x)<=1e6 for x in (a,b)):raise ValueError('Finite, bounded values required')
    return struct.pack('<ff',a,b)
class Parser:
    def __init__(self):self.buffer=bytearray();self.last=None;self.bad=0
    def feed(self,data,now=None):
        now=time.monotonic() if now is None else now
        if self.last is not None and now-self.last>.1:self.buffer.clear()
        self.last=now;frames=[]
        # Bytewise processing bounds the buffer even for a huge corrupt read.
        for b in data:
            self.buffer.append(b)
            while len(self.buffer)>=2:
                if self.buffer[:2]!=MAGIC:del self.buffer[0];continue
                if len(self.buffer)<8:break
                version,kind,n,seq=struct.unpack_from('<BBHH',self.buffer,2)
                if version!=1 or n>96:self.bad+=1;del self.buffer[0];continue
                if len(self.buffer)<n+10:break
                packet=self.buffer[:n+10]
                if crc(packet[:-2])!=struct.unpack_from('<H',packet,n+8)[0]:self.bad+=1;del self.buffer[0];continue
                frames.append((kind,seq,bytes(packet[8:-2])));del self.buffer[:n+10]
        return frames
def decode_telemetry(data):
    if len(data)!=88 or data[0]!=1:raise ValueError('Unknown telemetry schema')
    result=dict(zip(['version','mode','armed','fault','state','profile','phase_l','phase_r'],data[:8]))
    for name,offset in [('ms',8),('cal_flags',12),('imu_age_ms',16),('imu_good',20),('fg_l',24),('fg_r',28),('imu_bad',72),('rejected',76),('weight_events',80),('pose_valid',84)]:result[name]=struct.unpack_from('<I',data,offset)[0]
    for name,offset in [('hz_l',32),('hz_r',36),('duty_l',40),('duty_r',44),('pitch_deg',48),('yaw_rad',52),('x_m',56),('y_m',60),('manual_l',64),('manual_r',68)]:
        value=struct.unpack_from('<f',data,offset)[0]
        if not math.isfinite(value):raise ValueError('Non-finite telemetry')
        result[name]=value
    quality=result['pose_valid'];result['pose_valid']=quality&1
    result['direction_assumed_mask']=(quality>>1)&3;result['coasting_mask']=(quality>>3)&3
    result['warnings']=(quality>>8)&0x1ff
    return result
def decode_imu(data):
    if len(data)!=66:raise ValueError('Unknown IMU schema')
    euler=struct.unpack_from('<fff',data);raw=struct.unpack_from('<9h',data,12);q=struct.unpack_from('<4f',data,30)
    return {'euler_deg':euler,'accel_raw':raw[:3],'gyro_raw':raw[3:6],'mag_raw':raw[6:],'quaternion':q,'timestamps_ms':struct.unpack_from('<4I',data,46),'euler_seq':struct.unpack_from('<I',data,62)[0]}
def demo_telemetry(t):
    packet=bytearray(88);packet[:8]=bytes([1,0,0,0,0,0,0,0]);struct.pack_into('<I',packet,8,int(t*1000)&0xffffffff)
    struct.pack_into('<I',packet,16,20)
    for off,value in [(32,15+10*math.sin(t)),(36,15+10*math.cos(t)),(48,3*math.sin(t)),(52,.2*math.cos(t)),(56,.4+.2*math.sin(t)),(60,.02*math.cos(t))]:struct.pack_into('<f',packet,off,value)
    return bytes(packet)
def mission_start_commands(profile_code,armed):
    """Commands that must leave the PC together to start one task.

    app.c:177 stops an armed-but-idle app once 500 ms pass without a new command,
    and no harmless command refreshes that timer (PING/IMU-query/STATUS return
    before last_cmd_ms is touched), so ARM and START cannot be two operator
    clicks: one click has to emit both, ARM first.
    """
    code=int(profile_code)
    if code not in PROFILES:raise ValueError('unknown profile')
    start=(6,bytes([code]))
    return [start] if armed else [(4,b'\xde\xc0'),start]
def manual_command_sequence(kind,payload,armed):
    """Same rule for the first manual frame: ARM has to ride with it.

    Clicking ARM and then starting the manual stream seconds later always fails,
    because app.c:177 disarms an armed-but-idle app after 500 ms.
    """
    if kind not in (5,10):raise ValueError('unknown manual command')
    body=[(kind,payload)]
    return body if armed else [(4,b'\xde\xc0')]+body
class MonitorModel:
    def __init__(self):
        self.connected=False;self.pending={};self.sequence=int(time.monotonic()*1000)&0xffff;self.samples=deque(maxlen=10000);self.latest={};self.imu={}
    def command(self,kind,payload=b''):
        if len(self.pending)>=64:raise ValueError('Too many unacknowledged commands')
        self.sequence=(self.sequence+1)&0xffff
        while self.sequence in self.pending:self.sequence=(self.sequence+1)&0xffff
        raw=encode(kind,self.sequence,payload);self.pending[self.sequence]=(kind,time.monotonic());return self.sequence,raw
    def receive(self,frame):
        kind,seq,data=frame
        try:
            if kind==0x90:self.latest=decode_telemetry(data);self.samples.append(self.latest.copy());return self.latest
            if kind==0x91:self.imu=decode_imu(data);return self.imu
            if kind in (0x80,0x81) and len(data)==3:
                pending=self.pending.get(seq)
                if pending and pending[0]==data[0]:self.pending.pop(seq);return tuple(data)
        except (ValueError,struct.error):return None
        return None
    def expired(self,now=None):
        now=time.monotonic() if now is None else now
        expired=[s for s,(_,t) in self.pending.items() if now-t>1]
        for seq in expired:self.pending.pop(seq)
        return expired
