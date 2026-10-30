import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'PC'))
from smart_analysis import SmartAnalyzer, analyze_round


def sample(**values):
    base=dict(ms=1000,mode=2,armed=1,fault=0,state=3,profile=1,
              imu_age_ms=20,imu_good=10,imu_bad=0,pose_valid=1,
              direction_assumed_mask=0,coasting_mask=0,fg_l=100,fg_r=100,
              hz_l=100.,hz_r=100.,duty_l=.55,duty_r=.55,
              pitch_deg=0.,yaw_rad=0.,x_m=0.,y_m=0.)
    base.update(values)
    return base


class SmartAnalysisTests(unittest.TestCase):
    def test_extreme_fg_is_warning_not_a_proven_cause(self):
        result=SmartAnalyzer().feed(sample(hz_r=7246),now=1)
        text=' '.join(str(x) for x in result['alerts'])
        self.assertIn('右',text)
        self.assertIn('FG',text)
        self.assertNotIn('确定是',text)

    def test_stale_imu_and_fault_are_distinct(self):
        result=SmartAnalyzer().feed(sample(imu_age_ms=5000,fault=7),now=1)
        text=' '.join(str(x) for x in result['alerts'])
        self.assertIn('IMU',text)
        self.assertIn('TILT',text)

    def test_link_staleness_and_recovery(self):
        a=SmartAnalyzer()
        a.feed(sample(),now=5)
        self.assertTrue(a.check_link(now=7,connected=True))
        a.feed(sample(),now=7)
        self.assertFalse(a.check_link(now=7,connected=True))
        self.assertTrue(a.check_link(now=8,connected=False))

    def test_missing_values_are_not_invented(self):
        result=analyze_round([{'state':5},{'state':8}])
        self.assertIsNone(result['duration_s'])
        self.assertIsNone(result['pitch_peak_deg'])
        self.assertIsNone(result['distance_m'])
        self.assertIn('不足',result['summary'])

    def test_round_time_wrap_and_state_durations(self):
        rows=[sample(ms=0xfffffff0,state=3,x_m=0),sample(ms=34,state=5,x_m=.1),sample(ms=1034,state=8,x_m=.2)]
        r=analyze_round(rows)
        self.assertAlmostEqual(r['duration_s'],1.05)
        self.assertAlmostEqual(r['state_durations']['上板'],.05)
        self.assertAlmostEqual(r['state_durations']['平衡'],1.)
        self.assertAlmostEqual(r['distance_m'],.2)
        self.assertIn('估计',r['summary'])

    def test_reverse_motion_displacement_not_path_distance(self):
        r=analyze_round([sample(ms=0,x_m=0),sample(ms=1000,x_m=1),sample(ms=2000,x_m=0)])
        self.assertAlmostEqual(r['distance_m'],2.)
        self.assertIn('轨迹',r['summary'])

    def test_stall_requires_persistence_and_armed(self):
        a=SmartAnalyzer()
        first=a.feed(sample(hz_l=0),now=1)
        self.assertNotIn('反馈缺失',str(first['alerts']))
        a.feed(sample(hz_l=0),now=1.4)
        a.feed(sample(hz_l=0),now=1.8)
        second=a.feed(sample(hz_l=0),now=2.1)
        self.assertIn('反馈',str(second['alerts']))
        stopped=a.feed(sample(hz_l=0,armed=0),now=3)
        self.assertNotIn('反馈缺失',str(stopped['alerts']))

    def test_gap_cannot_be_counted_as_continuous_feedback_loss(self):
        a=SmartAnalyzer()
        a.feed(sample(hz_l=0),now=1)
        r=a.feed(sample(hz_l=0),now=5)
        self.assertNotIn('反馈缺失',str(r['alerts']))

    def test_clock_reset_is_not_a_forty_day_run(self):
        result=analyze_round([sample(ms=120000),sample(ms=500)])
        self.assertIsNone(result['duration_s'])
        self.assertIn('重置',result['summary'])

    def test_balance_metrics_exclude_long_gaps_and_reset_streak(self):
        rows=[sample(ms=0,state=5,pitch_deg=.5),sample(ms=100,state=5,pitch_deg=2),
              sample(ms=200,state=5,pitch_deg=.3),sample(ms=1200,state=5,pitch_deg=.2),
              sample(ms=1300,state=8)]
        r=analyze_round(rows)
        self.assertAlmostEqual(r['balance_observed_s'],.3)
        self.assertAlmostEqual(r['balance_in_band_s'],.2)
        self.assertAlmostEqual(r['balance_longest_s'],.1)
        self.assertIn('丢失',r['summary'])


if __name__=='____main__':
    unittest.main()
