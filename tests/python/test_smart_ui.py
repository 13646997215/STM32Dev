"""Meaningful geometry and round-recording checks without a serial port."""
import importlib,sys,unittest,tempfile,csv
import tkinter as tk
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'PC'))
monitor=importlib.import_module('monitor')
class PlotTests(unittest.TestCase):
 def test_round_report_includes_action_and_no_completion_claim(self):
  report=monitor.format_round_report({'summary':'结束状态 ARRIVED','alerts':[{'title':'FG 异常','detail':'观测值较大','action':'检查线路'}]})
  self.assertIn('检查线路',report);self.assertIn('FG 异常',report)
  self.assertIn('不代表',report)
 def test_xy_projection_preserves_aspect_ratio_and_positive_up(self):
  points,bounds=monitor.trajectory_projection([(0,0),(1,0),(1,1)],600,300)
  self.assertAlmostEqual(abs(points[2]-points[0]),abs(points[5]-points[3]),places=5)
  self.assertLess(points[5],points[3])
  self.assertTrue(all(0<=points[i]<=600 for i in (0,2,4)))
  self.assertTrue(all(0<=points[i]<=300 for i in (1,3,5)))
 def test_single_stationary_point_does_not_divide_by_zero(self):
  points,bounds=monitor.trajectory_projection([(0,0)]*3,400,240)
  self.assertEqual(len(points),6)
  self.assertGreater(bounds[1],bounds[0]);self.assertGreater(bounds[3],bounds[2])
 def test_trajectory_ignores_nonfinite_coordinates(self):
  points,_=monitor.trajectory_projection([(float('nan'),1),(0,0)],400,240)
  self.assertEqual(len(points),2)
 def test_bounds_match_pixel_projection_and_aspect(self):
  points,(xmin,xmax,ymin,ymax)=monitor.trajectory_projection([(0,0),(2,1)],600,300)
  self.assertAlmostEqual((points[0]/600)*(xmax-xmin)+xmin,0)
  self.assertAlmostEqual(ymax-(points[1]/300)*(ymax-ymin),0)
 def test_start_size_accounts_for_display_scale(self):
  w,h=monitor.initial_window_size(1800,1020,1.25)
  self.assertLessEqual(w*1.25,1800);self.assertLessEqual(h*1.25,940)
 def test_small_canvas_draws_within_real_visible_bounds(self):
  root=tk.Tk()
  try:
   app=monitor.Monitor(root,True);root.geometry('900x620');root.update()
   for t in range(3):app.frame((0x90,0,monitor.demo_telemetry(t)))
   app.draw();root.update()
   for canvas in (app.canvas,app.map_canvas):
    box=canvas.bbox('all');self.assertIsNotNone(box)
    self.assertGreaterEqual(box[0],0);self.assertGreaterEqual(box[1],0)
    self.assertLessEqual(box[2],canvas.winfo_width());self.assertLessEqual(box[3],canvas.winfo_height())
  finally:root.destroy()
 def test_small_map_labels_are_readable_without_text_collisions(self):
  root=tk.Tk()
  try:
   app=monitor.Monitor(root,True);root.geometry('900x620');root.update()
   for t in range(3):app.frame((0x90,0,monitor.demo_telemetry(t)))
   app.draw();root.update();canvas=app.map_canvas
   labels=[i for i in canvas.find_all() if canvas.type(i)=='text']
   self.assertTrue(any('Y (m)' in canvas.itemcget(i,'text') for i in labels))
   self.assertTrue(any('X (m)' in canvas.itemcget(i,'text') for i in labels))
   for pos,a in enumerate(labels):
    x1,y1,x2,y2=canvas.bbox(a)
    for b in labels[pos+1:]:
     X1,Y1,X2,Y2=canvas.bbox(b)
     self.assertFalse(x1<X2 and X1<x2 and y1<Y2 and Y1<y2,
                      f"overlap: {canvas.itemcget(a,'text')} / {canvas.itemcget(b,'text')}")
  finally:root.destroy()
 def test_regular_map_keeps_detailed_coordinates_and_all_ticks(self):
  root=tk.Tk()
  try:
   app=monitor.Monitor(root,True);root.geometry('1440x900');root.update()
   for t in range(3):app.frame((0x90,0,monitor.demo_telemetry(t)))
   app.draw();root.update()
   texts=[app.map_canvas.itemcget(i,'text') for i in app.map_canvas.find_all() if app.map_canvas.type(i)=='text']
   self.assertTrue(any('当前 x=' in text for text in texts))
   self.assertGreaterEqual(len([text for text in texts if text.replace('-','').replace('.','').isdigit()]),6)
  finally:root.destroy()
 def test_many_long_diagnostics_preserve_dashboard_and_full_details(self):
  root=tk.Tk()
  try:
   app=monitor.Monitor(root,True);app.demo=False;root.geometry('900x620');root.update()
   alerts=[{'severity':'warning','title':f'故障{i}','detail':'异常详情'*20,'action':'检查接线和电源'*8} for i in range(3)]
   app.show_diagnosis({'state_text':'运行','fault_text':'FEEDBACK','alerts':alerts,'pose_note':'估计坐标'})
   root.update()
   self.assertTrue(app.canvas.winfo_viewable());self.assertGreaterEqual(app.canvas.winfo_height(),70)
   self.assertTrue(app.control_dock.winfo_viewable());self.assertTrue(app.log.winfo_viewable())
   self.assertTrue(app.diagnostic_button.winfo_viewable())
   self.assertIn('故障0',app.insight.get());self.assertIn('检查接线',app.insight.get())
   app.show_diagnostics();root.update()
   full=app.diagnostics_text.get('1.0','end')
   for alert in alerts:self.assertIn(alert['title'],full);self.assertIn(alert['detail'],full);self.assertIn(alert['action'],full)
   self.assertIn('100',app.control_hint.cget('text'));self.assertIn('500',app.control_hint.cget('text'))
  finally:root.destroy()
 def test_small_window_has_stop_and_controls_and_log_visible_with_charts(self):
  root=tk.Tk()
  try:
   app=monitor.Monitor(root,True);root.geometry('900x620');root.update()
   self.assertTrue(app.stop_button.winfo_viewable())
   self.assertLess(app.stop_button.winfo_rooty()-root.winfo_rooty(),160)
   for widget in (app.control_dock,app.control_hint,app.arm_button,app.dock_stop_button,app.clear_button,
                  app.profile_box,app.start_button,app.left_entry,app.right_entry,app.unit_box,
                  app.manual_button,app.bench_button,app.log_heading,app.log,app.canvas,app.map_canvas):
    self.assertTrue(widget.winfo_viewable(),str(widget))
    self.assertGreaterEqual(widget.winfo_rooty(),root.winfo_rooty())
    self.assertLessEqual(widget.winfo_rooty()+widget.winfo_height(),root.winfo_rooty()+root.winfo_height())
    self.assertGreaterEqual(widget.winfo_rootx(),root.winfo_rootx())
    self.assertLessEqual(widget.winfo_rootx()+widget.winfo_width(),root.winfo_rootx()+root.winfo_width())
   self.assertIn('运行日志',app.log_heading.cget('text'))
   self.assertIn('离线演示已启动，未打开串口',app.log.get('1.0','end'))
   self.assertIsNotNone(app.log.dlineinfo('1.0'));self.assertIsNotNone(app.log.dlineinfo('2.0'))
   app.frame((0x90,0,monitor.demo_telemetry(1)))
   self.assertIn('ARM 0',app.metric_meta['state'].get())
   self.assertIn('IMU age',app.metric_meta['attitude'].get())
   self.assertIn('PWM',app.metric_meta['feedback'].get())
   app.disconnect()
   self.assertIn('数值已过期',app.metric_meta['attitude'].get())
   self.assertIn('AUTO',app.status.get())
  finally:root.destroy()
class RoundRecordingTests(unittest.TestCase):
 def test_interrupted_round_is_saved_without_claiming_arrival(self):
  m=monitor.Monitor.__new__(monitor.Monitor);m.demo=False;m.replaying=False
  m.round_name='partial';m.round_rows=[{'state':5,'pitch_deg':2.0,'pc_time':1.0}];m.notes=[];m.note=m.notes.append
  with tempfile.TemporaryDirectory() as d:
   with patch.object(monitor,'__file__',str(Path(d)/'monitor.py')):
    m.finish_round('连接关闭，未观察到终态')
    path=Path(d)/'runs/partial.csv';self.assertTrue(path.exists())
    with path.open(encoding='utf-8-sig',newline='') as f:self.assertEqual(list(csv.DictReader(f))[0]['state'],'5')
    self.assertIn('未观察到终态',m.notes[0]);self.assertIsNone(m.round_rows)
 def test_replay_never_writes_new_round_csv(self):
  m=monitor.Monitor.__new__(monitor.Monitor);m.demo=False;m.replaying=True;m.round_name='replay';m.round_rows=[{'state':8}]
  with tempfile.TemporaryDirectory() as d:
   with patch.object(monitor,'__file__',str(Path(d)/'monitor.py')):
    m.finish_round('ARRIVED');self.assertFalse((Path(d)/'runs').exists())
if __name__=='__main__':unittest.main()
