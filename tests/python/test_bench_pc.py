"""Real monitor handlers with a memory-only serial endpoint. No Tk or COM access."""
import struct, sys, unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'PC'))
import monitor
from course_protocol import MonitorModel, Parser, PROFILES, demo_telemetry

class Value:
    def __init__(self, value=''): self.value = value
    def set(self, value): self.value = value
    def get(self): return self.value

class Serial:
    in_waiting = 0
    def __init__(self): self.writes = []; self.closed = False
    def write(self, raw): self.writes.append(raw); return len(raw)
    def read(self, n): return b''
    def close(self): self.closed = True

class Root:
    def after(self, *args): pass
    def destroy(self): self.destroyed = True

class BenchMonitorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.clock = patch.object(monitor.time, 'monotonic', side_effect=lambda: self.now)
        self.clock.start(); self.addCleanup(self.clock.stop)
        self.m = m = monitor.Monitor.__new__(monitor.Monitor)
        m.demo = False; m.serial = Serial(); m.model = MonitorModel(); m.parser = Parser()
        m.manual = None; m.record = None; m.config_jobs = []; m.config_wait = None
        m.bench_pending = m.bench_seq = None; m.last_bench = self.now
        m.last_manual = m.last_plot = m.start = self.now; m.last_rx = 0
        m.notes = []; m.note = m.notes.append; m.draw = lambda: None; m.root = Root()
        m.status = Value(); m.summary = Value(); m.imu_text = Value(); m.profile = Value(PROFILES[1])
    def frames(self): return Parser().feed(b''.join(self.m.serial.writes), self.now)
    def start_ack(self):
        self.m.start_bench(); seq = self.frames()[0][1]
        self.m.frame((0x80, seq, bytes([11, 0, 0]))); return seq
    def test_explicit_start_waits_for_ack(self):
        self.m.start_bench(); frames = self.frames()
        self.assertEqual(len(frames), 1); self.assertEqual(frames[0][0], 11)
        self.assertEqual(frames[0][2], b'T1LV'); self.assertIsNone(self.m.bench_seq)
        self.now += .2; self.m.poll(); self.assertEqual(len(self.frames()), 1)
    def test_keepalive_is_bound_to_acknowledged_start(self):
        seq = self.start_ack(); self.now += .11; self.m.poll()
        self.assertEqual(self.frames()[-1][0], 12)
        self.assertEqual(self.frames()[-1][2], struct.pack('<H', seq))
        self.assertFalse(any(f[0] in (4, 6) for f in self.frames()))
    def test_stop_cancels_keepalive(self):
        self.start_ack(); self.m.stop(); self.assertIsNone(self.m.bench_seq)
        self.assertEqual(self.frames()[-1][0], 7)
        self.now += .3; self.m.poll(); self.assertEqual(self.frames()[-1][0], 7)
    def test_disconnect_and_reconnect_do_not_restart(self):
        self.start_ack(); old = self.m.serial; self.m.disconnect()
        self.assertTrue(old.closed); self.assertIsNone(self.m.bench_seq)
        self.m.serial = Serial(); self.now += .3; self.m.poll()
        self.assertEqual(self.frames(), [])
    def test_terminal_telemetry_cancels_keepalive(self):
        self.start_ack(); raw = bytearray(demo_telemetry(1)); raw[1] = 4; raw[2] = 1
        self.m.frame((0x90, 1, bytes(raw))); self.assertIsNotNone(self.m.bench_seq)
        raw[1] = raw[2] = 0; raw[4] = 11
        self.m.frame((0x90, 2, bytes(raw))); self.assertIsNone(self.m.bench_seq)
    def test_rejected_start_never_keeps_alive(self):
        self.m.start_bench(); seq = self.frames()[0][1]
        self.m.frame((0x81, seq, bytes([11, 4, 0])))
        self.now += .2; self.m.poll(); self.assertEqual(len(self.frames()), 1)
        self.assertIsNone(self.m.bench_pending); self.assertIsNone(self.m.bench_seq)
    def test_ack_timeout_cancels_bench(self):
        self.start_ack(); self.now += .11; self.m.poll(); self.now += 1.2; self.m.poll()
        self.assertIsNone(self.m.bench_seq)
    def test_close_sends_stop_for_pending_bench(self):
        self.m.start_bench(); serial = self.m.serial; self.m.close()
        frames = Parser().feed(b''.join(serial.writes), self.now)
        self.assertEqual(frames[-1][0], 7); self.assertTrue(serial.closed)
    def test_other_start_controls_do_not_take_over(self):
        self.start_ack(); self.m.start_mission(); self.m.manual_start()
        self.assertEqual([f[0] for f in self.frames()], [11])
    def test_normal_keepalive_does_not_flood_operator_log(self):
        self.start_ack(); self.m.notes.clear(); self.now += .11; self.m.poll()
        seq = self.frames()[-1][1]; self.m.frame((0x80, seq, bytes([12, 0, 0])))
        self.assertEqual(self.m.notes, [])
    def test_tilt_fault_explains_recovery_without_restart(self):
        self.start_ack(); self.m.notes.clear(); raw = bytearray(demo_telemetry(1))
        raw[3] = 7; raw[4] = 12; struct.pack_into('<f', raw, 48, 20.5)
        self.m.frame((0x90, 1, bytes(raw))); self.m.frame((0x90, 2, bytes(raw)))
        detail = '\n'.join(self.m.notes)
        self.assertIn('倾角保护', detail); self.assertIn('20.50', detail); self.assertIn('清除故障', detail)
        self.assertEqual(len(self.m.notes), 1); self.assertIsNone(self.m.bench_seq)
        self.assertEqual([f[0] for f in self.frames()], [11])

if __name__ == '__main__': unittest.main()
