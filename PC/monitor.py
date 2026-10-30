"""Course monitor. A COM port is opened only by clicking Connect."""
import argparse,csv,json,math,struct,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog,messagebox
from course_protocol import MonitorModel,Parser,PROFILES,STATES,FAULTS,RESULTS,command_values,demo_telemetry,mission_start_commands,manual_command_sequence
from smart_analysis import SmartAnalyzer,analyze_round

def trajectory_projection(coordinates,width,height,padding=35):
    """Project estimated metres with equal scales and positive Y upward."""
    valid=[(float(x),float(y)) for x,y in coordinates if math.isfinite(x) and math.isfinite(y)]
    if not valid:return [],(-.1,.1,-.1,.1)
    xmin,xmax=min(x for x,y in valid),max(x for x,y in valid)
    ymin,ymax=min(y for x,y in valid),max(y for x,y in valid)
    span=max(xmax-xmin,ymax-ymin,.10)*1.18
    cx,cy=(xmin+xmax)/2,(ymin+ymax)/2
    scale=min(max(width-2*padding,1),max(height-2*padding,1))/span
    points=[]
    for x,y in valid:points.extend([width/2+(x-cx)*scale,height/2-(y-cy)*scale])
    return points,(cx-width/(2*scale),cx+width/(2*scale),cy-height/(2*scale),cy+height/(2*scale))

def initial_window_size(screen_width,screen_height,display_scale=1):
    """Fit the usable display; Tk's screen size is already in Tk coordinates."""
    scale=max(float(display_scale),1.0)
    return (max(1,min(1440,int(screen_width/scale)-32)),
            max(1,min(900,int((screen_height-80)/scale))))

def format_round_report(report):
    lines=[str(report.get('summary','数据不足。')),'\n结果基于已收到的遥测，不代表课程验收通过。']
    for a in report.get('alerts',[]):
        lines.append(f"\n{a.get('title','提示')}：{a.get('detail','')}\n建议：{a.get('action','')}")
    return '\n'.join(lines)

class Monitor:
    def __init__(self,root,demo=False):
        self.root=root;self.demo=demo;self.model=MonitorModel();self.parser=Parser();self.serial=None
        self.record=None;self.manual=None;self.last_manual=0;self.start=time.monotonic();self.last_rx=0
        self.bench_pending=self.bench_seq=None;self.last_bench=0;self.last_control_status=None
        self.config_jobs=[];self.config_wait=None;self.proposed={};self.last_plot=0
        self.analyzer=SmartAnalyzer();self.last_diagnosis=None
        root.title('F401 · 课程调试工作台'+(' — 离线演示' if demo else ''))
        width,height=initial_window_size(root.winfo_screenwidth(),root.winfo_screenheight())
        root.geometry(f'{width}x{height}+0+0');root.minsize(min(900,width),min(600,height))
        style=ttk.Style(root)
        if 'clam' in style.theme_names():style.theme_use('clam')
        BG='#edf2f7'; SURFACE='#ffffff'; INK='#19314a'; MUTED='#647b91'; BLUE='#1769d2'
        root.configure(bg=BG)
        style.configure('TFrame',background=BG)
        style.configure('Surface.TFrame',background=SURFACE)
        style.configure('TLabel',background=BG,foreground=INK,font=('Microsoft YaHei UI',9))
        style.configure('Surface.TLabel',background=SURFACE,foreground=INK,font=('Microsoft YaHei UI',9))
        style.configure('Caption.TLabel',background=SURFACE,foreground=MUTED,font=('Microsoft YaHei UI',8))
        style.configure('Metric.TLabel',background=SURFACE,foreground=INK,font=('Microsoft YaHei UI',12,'bold'))
        style.configure('TButton',padding=(7,4),font=('Microsoft YaHei UI',9),background='#ffffff',foreground=INK,borderwidth=1)
        style.map('TButton',background=[('active','#e8f1fb')],foreground=[('disabled','#a1afbd')])
        style.configure('Primary.TButton',background=BLUE,foreground='white',padding=(9,4),font=('Microsoft YaHei UI',9,'bold'))
        style.map('Primary.TButton',background=[('active','#1155ac')],foreground=[('disabled','#d3deec')])
        style.configure('Emergency.TButton',background='#b42336',foreground='white',font=('Microsoft YaHei UI',9,'bold'),padding=(10,5))
        style.map('Emergency.TButton',background=[('active','#8d1829')])
        style.configure('TNotebook',background=BG,borderwidth=0)
        style.configure('TNotebook.Tab',padding=(12,5),font=('Microsoft YaHei UI',9),background='#e0e8f0',foreground='#526b82')
        style.map('TNotebook.Tab',background=[('selected',SURFACE)],foreground=[('selected',BLUE)])
        header=tk.Frame(root,bg='#102c46',height=48);header.pack(fill='x');header.pack_propagate(False)
        brand=tk.Frame(header,bg='#102c46');brand.pack(side='left',padx=16)
        tk.Label(brand,text='ROBOT LAB',bg='#102c46',fg='#69d8ed',font=('Segoe UI',12,'bold')).pack(side='left')
        tk.Label(brand,text='  /  课程调试工作台',bg='#102c46',fg='white',font=('Microsoft YaHei UI',12,'bold')).pack(side='left')
        self.stop_button=ttk.Button(header,text='■ STOP  停止',style='Emergency.TButton',command=self.stop);self.stop_button.pack(side='right',padx=14,pady=6)
        top=ttk.Frame(root,style='Surface.TFrame',padding=(12,5));top.pack(fill='x',padx=8,pady=(6,0))
        self.port=tk.StringVar();ttk.Label(top,text='串口',style='Surface.TLabel').pack(side='left');self.portbox=ttk.Combobox(top,textvariable=self.port,width=9);self.portbox.pack(side='left',padx=(5,4))
        ttk.Button(top,text='刷新',command=self.refresh_ports).pack(side='left',padx=2)
        self.connect_button=ttk.Button(top,text='连接 115200',style='Primary.TButton',command=self.connect);self.connect_button.pack(side='left',padx=2)
        ttk.Button(top,text='断开',command=self.disconnect).pack(side='left',padx=2)
        ttk.Button(top,text='记录原始帧',command=self.record_toggle).pack(side='left',padx=2)
        ttk.Button(top,text='回放',command=self.replay).pack(side='left',padx=2)
        ttk.Button(top,text='导出 CSV',command=self.export).pack(side='left',padx=2)
        self.status=tk.StringVar(value='离线演示数据，不代表实物性能' if demo else '未连接。启动软件不会自动打开串口。')
        statusbar=ttk.Frame(root,style='Surface.TFrame',padding=(12,0,12,5));statusbar.pack(fill='x',padx=8)
        ttk.Label(statusbar,text='●',style='Caption.TLabel',foreground='#2c8ca5').pack(side='left',padx=(0,6))
        ttk.Label(statusbar,textvariable=self.status,style='Caption.TLabel').pack(side='left',fill='x',expand=True)
        # Metrics contain telemetry only; -- placeholders remain until a valid frame arrives.
        metrics=ttk.Frame(root);metrics.pack(fill='x',padx=8,pady=(6,2))
        self.metric_vars={key:tk.StringVar(value='--') for key in ('state','attitude','feedback','position')}
        self.metric_meta={key:tk.StringVar(value='等待遥测') for key in ('state','attitude','feedback','position')}
        for key,title,sub in [('state','运行状态','模式 / 故障'),('attitude','姿态','Pitch / Yaw'),('feedback','轮速反馈','左 / 右  ·  Hz'),('position','估计位置','X / Y  ·  m')]:
            card=ttk.Frame(metrics,style='Surface.TFrame',padding=(10,4));card.pack(side='left',fill='both',expand=True,padx=(0,4))
            ttk.Label(card,text=title+'  ·  '+sub,style='Caption.TLabel').pack(anchor='w')
            ttk.Label(card,textvariable=self.metric_vars[key],style='Metric.TLabel').pack(anchor='w')
            ttk.Label(card,textvariable=self.metric_meta[key],style='Caption.TLabel').pack(anchor='w')
        self.summary=tk.StringVar(value='等待遥测')
        # Dock is packed before the expanding notebook so its safety controls and log stay visible.
        dock=ttk.Frame(root,style='Surface.TFrame',padding=(10,5));dock.pack(side='bottom',fill='x',padx=8,pady=(3,0));self.control_dock=dock
        bar=ttk.Frame(dock,style='Surface.TFrame');bar.pack(fill='x')
        ttk.Label(bar,text='控制台',style='Surface.TLabel',font=('Microsoft YaHei UI',10,'bold')).pack(side='left')
        self.control_hint=ttk.Label(bar,text='  ·  手动 100ms 续发 / 500ms 超时停  |  AUTO 断链可能继续  |  STOP 看 ACK / 遥测',style='Caption.TLabel');self.control_hint.pack(side='left')
        line1=ttk.Frame(dock,style='Surface.TFrame');line1.pack(fill='x',pady=(3,1))
        self.arm_button=ttk.Button(line1,text='ARM 允许控制',command=lambda:self.send(4,b'\xde\xc0'));self.arm_button.pack(side='left',padx=(0,4))
        self.dock_stop_button=ttk.Button(line1,text='STOP 停止',style='Emergency.TButton',command=self.stop);self.dock_stop_button.pack(side='left',padx=(0,4))
        self.clear_button=ttk.Button(line1,text='清除故障',command=lambda:self.send(8));self.clear_button.pack(side='left',padx=(0,6))
        self.profile=tk.StringVar(value=PROFILES[1]);self.profile_box=ttk.Combobox(line1,textvariable=self.profile,values=list(PROFILES.values()),state='readonly',width=29);self.profile_box.pack(side='left',fill='x',expand=True,padx=(0,5))
        self.start_button=ttk.Button(line1,text='启动选定任务',style='Primary.TButton',command=self.start_mission);self.start_button.pack(side='left')
        line2=ttk.Frame(dock,style='Surface.TFrame');line2.pack(fill='x',pady=(2,1))
        self.left=tk.StringVar(value='0');self.right=tk.StringVar(value='0');self.unit=tk.StringVar(value='轮速 RPM')
        ttk.Label(line2,text='左 / 右',style='Surface.TLabel').pack(side='left',padx=(0,4))
        self.left_entry=ttk.Entry(line2,textvariable=self.left,width=7);self.left_entry.pack(side='left',padx=(0,3))
        self.right_entry=ttk.Entry(line2,textvariable=self.right,width=7);self.right_entry.pack(side='left',padx=(0,5))
        self.unit_box=ttk.Combobox(line2,textvariable=self.unit,values=['轮速 RPM','占空比 -1..1'],state='readonly',width=13);self.unit_box.pack(side='left',padx=(0,5))
        self.manual_button=ttk.Button(line2,text='持续发送手动目标',command=self.manual_start);self.manual_button.pack(side='left',padx=(0,5))
        self.bench_button=ttk.Button(line2,text='任务1台架（双轮架空）',command=self.start_bench);self.bench_button.pack(side='left')
        self.log_heading=ttk.Label(dock,text='运行日志  ·  ACK / 状态 / 故障',style='Caption.TLabel');self.log_heading.pack(anchor='w',pady=(2,0))
        self.log=tk.Text(dock,height=3,state='disabled',font=('Consolas',9),bg='#f4f8fc',fg='#334b63',relief='flat',highlightthickness=0,wrap='none')
        self.log.pack(fill='x',pady=(1,0))
        self.note('离线演示已启动，未打开串口。' if demo else '上位机已启动，未打开串口。')
        self.note('演示数据不代表实物。' if demo else '选择串口并点击连接后才开始通信。')
        self.tabs=ttk.Notebook(root);self.tabs.pack(fill='both',expand=True,padx=8,pady=(2,0))
        self.chart_tab=ttk.Frame(self.tabs);self.analysis_tab=ttk.Frame(self.tabs);self.config_tab=ttk.Frame(self.tabs);self.imu_tab=ttk.Frame(self.tabs)
        self.tabs.add(self.chart_tab,text='实时驾驶舱');self.tabs.add(self.analysis_tab,text='每轮分析');self.tabs.add(self.config_tab,text='参数与标定');self.tabs.add(self.imu_tab,text='IMU 原始观测')
        self.insight=tk.StringVar(value='等待有效遥测 · 诊断建议仅供定位问题，不代表硬件验收')
        diagnostic_row=ttk.Frame(self.chart_tab);diagnostic_row.pack(fill='x',padx=6)
        self.diagnostic_button=ttk.Button(diagnostic_row,text='查看诊断详情',command=self.show_diagnostics);self.diagnostic_button.pack(side='right',padx=(4,0),pady=2)
        self.insight_label=ttk.Label(diagnostic_row,textvariable=self.insight,padding=(4,3),font=('Microsoft YaHei UI',9),justify='left',wraplength=580)
        self.insight_label.pack(side='left',fill='x',expand=True)
        charts=ttk.Frame(self.chart_tab);charts.pack(fill='both',expand=True,padx=4,pady=(0,3))
        self.canvas=tk.Canvas(charts,bg='#122239',height=140,highlightthickness=0);self.canvas.pack(side='left',fill='both',expand=True,padx=(0,3))
        self.map_canvas=tk.Canvas(charts,bg='#122239',width=300,height=140,highlightthickness=0);self.map_canvas.pack(side='left',fill='both',expand=True,padx=(3,0))
        analysis_tools=ttk.Frame(self.analysis_tab,padding=8);analysis_tools.pack(fill='x')
        ttk.Label(analysis_tools,text='本轮回顾',font=('Microsoft YaHei UI',12,'bold')).pack(side='left')
        ttk.Button(analysis_tools,text='打开 CSV 分析',command=self.analyze_csv).pack(side='right')
        self.report=tk.Text(self.analysis_tab,font=('Microsoft YaHei UI',10),bg='white',fg=INK,relief='flat',padx=16,pady=12,wrap='word',state='disabled');self.report.pack(fill='both',expand=True,padx=8,pady=(0,6))
        self.show_report('等待一轮运行结束。也可打开之前保存的 CSV；分析结果基于遥测，不能替代实测。')
        cfg_top=ttk.Frame(self.config_tab,padding=8);cfg_top.pack(fill='x')
        self.key=tk.StringVar(value='1');self.value=tk.StringVar(value='0.067')
        ttk.Label(cfg_top,text='参数 key').pack(side='left');ttk.Entry(cfg_top,textvariable=self.key,width=8).pack(side='left',padx=5)
        ttk.Label(cfg_top,text='value（SI 单位，角度零偏除外）').pack(side='left');ttk.Entry(cfg_top,textvariable=self.value,width=15).pack(side='left',padx=5)
        ttk.Button(cfg_top,text='发送单项',command=self.set_config).pack(side='left');ttk.Button(cfg_top,text='保存已输入参数',command=self.save_config).pack(side='left',padx=5)
        ttk.Button(cfg_top,text='加载并逐项发送',command=self.load_config).pack(side='left')
        ttk.Label(self.config_tab,text='上电默认 cal_flags=0；按键启动流程可能自动载入 Config_ApplyMeasured，包含其他标志和限值。\n参数保存在 RAM，复位后恢复默认值；JSON 是你输入的参数记录，不是设备读回值。\n轮径 key 1=0.067 m；轮距 key 2 记录为 0.195 m；PPR key 5/6=738（教师例程 82×9）。完整表见 docs/configuration.md。',padding=12).pack(anchor='w')
        self.imu_text=tk.StringVar(value='等待传感器数据（GY953 的 raw 值保持原始单位）');ttk.Label(self.imu_tab,textvariable=self.imu_text,padding=14,font=('Consolas',11),justify='left').pack(anchor='w')
        ttk.Label(self.imu_tab,text='新版固件自动以 50Hz 查询 Euler，无需点击启动；下方按钮只读取一次，不切换连续输出。',padding=12).pack(anchor='w')
        queries=ttk.Frame(self.imu_tab);queries.pack(anchor='w',padx=12)
        for code,label in enumerate(['读取一次 Acc','读取一次 Mag','读取一次 Euler']):ttk.Button(queries,text=label,command=lambda c=code:self.send(9,bytes([c]))).pack(side='left',padx=5)
        root.protocol('WM_DELETE_WINDOW',self.close);root.after(25,self.poll)
    def note(self,text):
        self.log.configure(state='normal');self.log.insert('end',time.strftime('%H:%M:%S ')+str(text)+'\n')
        if int(self.log.index('end-1c').split('.')[0])>500:self.log.delete('1.0','100.0')
        self.log.see('end');self.log.configure(state='disabled')
    def show_report(self,text):
        if not hasattr(self,'report'):return
        self.report.configure(state='normal');self.report.delete('1.0','end');self.report.insert('1.0',str(text));self.report.configure(state='disabled')
    def analyze_csv(self):
        path=filedialog.askopenfilename(filetypes=[('遥测 CSV','*.csv')])
        if not path:return
        try:
            if Path(path).stat().st_size>20000000:raise ValueError('CSV 超过 20 MB，请选取单轮文件。')
            rows=[]
            with open(path,encoding='utf-8-sig',newline='') as f:
                for row in csv.DictReader(f):
                    if len(rows)>=100000:raise ValueError('CSV 超过 100000 条，请分段分析。')
                    numeric={}
                    for key,value in row.items():
                        try:numeric[key]=float(value)
                        except (ValueError,TypeError):numeric[key]=value
                    rows.append(numeric)
            if not rows:raise ValueError('CSV 没有遥测数据。')
            report=analyze_round(rows)
            self.show_report('离线文件：'+str(path)+'\n\n'+format_round_report(report))
            self.note(f'已分析 {len(rows)} 条遥测；坐标与方向为估计值。')
        except (OSError,ValueError,KeyError,csv.Error) as exc:self.note('分析失败：'+str(exc))
    def show_diagnosis(self,result):
        if not hasattr(self,'insight'):return
        self.last_diagnosis=result
        alerts=result.get('alerts',[])
        if alerts:
            first=alerts[0]
            def short(value,limit):
                value=str(value).replace('\n',' ')
                return value if len(value)<=limit else value[:limit-1]+'…'
            advice=f"{short(first.get('title','诊断'),22)}：{short(first.get('action','请查看详情'),42)}"
            if len(alerts)>1:advice+=f'（共 {len(alerts)} 项，查看详情）'
        else:advice='暂无明显遥测异常；持续观察现场情况。'
        self.insight.set(f"{str(result.get('state_text','等待状态'))[:16]}  |  {str(result.get('fault_text',''))[:16]}\n{advice}")
    def show_diagnostics(self):
        result=getattr(self,'last_diagnosis',None) or {}
        alerts=result.get('alerts',[])
        lines=[f"打开时的诊断快照：{time.strftime('%H:%M:%S')}\n状态：{result.get('state_text','等待遥测')}  |  故障：{result.get('fault_text','--')}"]
        if alerts:
            for alert in alerts:
                lines.append(f"\n{alert.get('title','提示')}\n现象：{alert.get('detail','')}\n建议：{alert.get('action','')}")
        else:lines.append('\n暂无诊断；请确认有持续更新的有效遥测。')
        lines.append('\n'+str(result.get('pose_note','估计轨迹非独立定位。')))
        window=tk.Toplevel(self.root);window.title('诊断详情（打开时快照）');window.geometry('700x420')
        text=tk.Text(window,wrap='word',font=('Microsoft YaHei UI',10),padx=14,pady=12)
        scroll=ttk.Scrollbar(window,orient='vertical',command=text.yview);text.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right',fill='y');text.pack(side='left',fill='both',expand=True)
        text.insert('1.0','\n'.join(lines));text.configure(state='disabled')
        self.diagnostics_text=text
    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            self.portbox['values']=[p.device for p in list_ports.comports()]
        except ImportError:self.note('当前 Python 缺少 pyserial；请使用已配置的项目 Python。')
    def connect(self):
        if self.demo:self.note('离线演示模式禁止连接硬件。');return
        if self.serial or not self.port.get():return
        try:
            import serial
            self.serial=serial.Serial(self.port.get(),115200,timeout=0,write_timeout=.02)
            self.model=MonitorModel();self.model.connected=True;self.parser=Parser();self.analyzer=SmartAnalyzer();self.last_rx=0;self.round_prev_state=None;self.note('连接成功，等待设备状态。');self.status.set('已连接 '+self.port.get())
            if hasattr(self,'metric_vars'):
                for value in self.metric_vars.values():value.set('--')
                for value in self.metric_meta.values():value.set('等待遥测')
        except (OSError,ValueError,ImportError) as exc:self.note(exc);self.serial=None
    def disconnect(self):
        self.finish_round('连接关闭，未观察到终态')
        self.manual=None;self.config_jobs=[];self.config_wait=None
        self.bench_pending=self.bench_seq=None
        if self.serial:self.serial.close();self.serial=None
        self.model.connected=False;self.status.set('已断开。AUTO 仍由固件自主执行；停止任务需 STOP 或独立断电。')
        if hasattr(self,'metric_meta'):
            for value in self.metric_meta.values():value.set('连接断开 · 数值已过期')
    def send(self,kind,payload=b''):
        if not self.serial:self.note('未连接；命令未发送。');return None
        try:
            seq,raw=self.model.command(kind,payload)
            if self.serial.write(raw)!=len(raw):raise OSError('Incomplete write')
            detail=f' selector={payload[0]}' if kind==9 and len(payload)==1 else ''
            if kind!=12:self.note(f'TX #{seq} type={kind}{detail}')
            return seq
        except (OSError,ValueError) as exc:self.note(exc);self.disconnect();return None
    def stop(self):
        self.manual=None;self.config_jobs=[];self.bench_pending=self.bench_seq=None;self.send(7)
    def blocked_by_fault(self):
        """A latched fault makes ARM/START/manual all return FAULT on the device."""
        f=getattr(self,'fault_now',0)
        if not f:return False
        name=FAULTS[f] if f<len(FAULTS) else str(f)
        self.note(f'设备当前有故障 {name}({f})，ARM/启动/手动都会被拒；请先点「清除故障」。')
        return True
    def start_bench(self):
        if self.blocked_by_fault():return
        if self.bench_pending is not None or self.bench_seq is not None:
            self.note('台架已启动或正在等待应答；结束请点STOP。');return
        self.manual=None;self.config_jobs=[];self.config_wait=None
        self.bench_pending=self.send(11,b'T1LV')
    def start_mission(self):
        if self.blocked_by_fault():return
        if self.bench_pending is not None or self.bench_seq is not None:
            self.note('台架运行中，请先STOP。');return
        self.manual=None
        for k,v in PROFILES.items():
            if v!=self.profile.get():continue
            for kind,payload in mission_start_commands(k,getattr(self,'armed_now',False)):
                if self.send(kind,payload) is None:return
            return
    def manual_start(self):
        if self.blocked_by_fault():return
        if self.bench_pending is not None or self.bench_seq is not None:
            self.note('台架运行中，请先STOP。');return
        try:
            kind=5 if self.unit.get()=='轮速 RPM' else 10;payload=command_values(float(self.left.get()),float(self.right.get()))
            for k,pl in manual_command_sequence(kind,payload,getattr(self,'armed_now',False)):
                if self.send(k,pl) is None:return
            self.manual=(kind,payload);self.last_manual=time.monotonic()
        except ValueError as exc:self.note(exc)
    def set_config(self):
        try:
            key=int(self.key.get());value=float(self.value.get())
            if not 0<=key<=65535 or not math.isfinite(value):raise ValueError('参数范围错误')
            self.send(3,struct.pack('<Hf',key,value));self.proposed[str(key)]=value
        except (ValueError,struct.error) as exc:self.note(exc)
    def save_config(self):
        path=filedialog.asksaveasfilename(defaultextension='.json')
        if path:Path(path).write_text(json.dumps({'schema':1,'parameters':self.proposed},indent=2),encoding='utf-8')
    def load_config(self):
        path=filedialog.askopenfilename(filetypes=[('参数 JSON','*.json')])
        if not path:return
        try:
            data=json.loads(Path(path).read_text(encoding='utf-8'))
            if data.get('schema')!=1 or not isinstance(data.get('parameters'),dict) or len(data['parameters'])>100:raise ValueError('参数文件格式错误')
            jobs=[(int(k),float(v)) for k,v in data['parameters'].items()]
            if any(not 0<=k<=65535 or not math.isfinite(v) for k,v in jobs):raise ValueError('参数非法')
            if not self.serial:raise ValueError('请先连接，设备必须解除 ARM')
            self.config_jobs=sorted(jobs,key=lambda kv:kv[0]==100);self.config_wait=None;self.proposed=data['parameters'];self.note('按 ACK 逐项发送，标定标志最后发送。')
        except (ValueError,OSError,TypeError) as exc:self.note(exc)
    def frame(self,frame):
        value=self.model.receive(frame);kind,seq,data=frame;self.last_rx=time.monotonic()
        if kind in (0x80,0x81) and value is not None:
            result=value[1]
            if data[0]!=12 or result!=0:self.note(f'ACK #{seq} {RESULTS[result] if result<len(RESULTS) else result}, fault={value[2]}')
            if data[0]==11 and seq==self.bench_pending:
                self.bench_pending=None
                if result==0:self.bench_seq=seq;self.last_bench=time.monotonic()
            if data[0]==12 and result!=0:self.bench_pending=self.bench_seq=None
            if seq==self.config_wait:
                self.config_wait=None
                if result!=0:self.config_jobs=[];self.note('参数发送已停止，请修正被拒绝的参数。')
            if result not in (0,5):
                if self.manual is not None:self.note(f'手动目标被拒（{RESULTS[result] if result<len(RESULTS) else result}），已停止续发。')
                self.manual=None
        if kind==0x90 and value:
            t=value;state=STATES[t['state']] if t['state']<len(STATES) else str(t['state']);fault=FAULTS[t['fault']] if t['fault']<len(FAULTS) else str(t['fault'])
            if hasattr(self,'metric_vars'):
                self.metric_vars['state'].set(f"{state}  /  {fault}")
                self.metric_vars['attitude'].set(f"{t['pitch_deg']:+.1f}°  /  {math.degrees(t['yaw_rad']):+.1f}°")
                self.metric_vars['feedback'].set(f"{t['hz_l']:.1f}  /  {t['hz_r']:.1f}")
                self.metric_vars['position'].set(f"{t['x_m']:+.3f}  /  {t['y_m']:+.3f}")
                self.metric_meta['state'].set(f"mode {t['mode']}  ·  ARM {t['armed']}  ·  flags {t['cal_flags']:02X}")
                age='未收到' if t['imu_age_ms']==4294967295 else f"{t['imu_age_ms']} ms"
                self.metric_meta['attitude'].set(f'IMU age  {age}')
                self.metric_meta['feedback'].set(f"PWM  {t['duty_l']:.2f}  /  {t['duty_r']:.2f}")
                self.metric_meta['position'].set(f"pose_valid {t['pose_valid']}  ·  方向为假设")
            status=(t['mode'],t['armed'],t['state'],t['fault'])
            if status!=getattr(self,'last_control_status',None):
                if t['fault']==7:
                    self.note(f"倾角保护 TILT(7)：遥测Pitch={t['pitch_deg']:.2f}°；限值取决于当前配置，遥测不提供其值。放回允许姿态→清除故障→重新启动；不会自动恢复。")
                elif t['fault']:
                    self.note(f"停止：{fault}({t['fault']})，Pitch={t['pitch_deg']:.2f}°。处理原因后清除故障再启动。")
                elif t['mode']==4:
                    self.note('任务1台架：'+('近零停止，仍监测Pitch；再次倾斜会恢复调平。' if t['state']==6 else 'Pitch调平运行；Yaw仅观察，换向前两轮先停车。'))
                elif t['state']==11:self.note('任务完成 DONE：PWM停止并解除ARM。')
                self.last_control_status=status
            self.summary.set(f"mode={t['mode']} ARM={t['armed']} {state} fault={fault} flags={t['cal_flags']:02X} IMU age={t['imu_age_ms']}ms\nPitch={t['pitch_deg']:.2f}° Yaw={math.degrees(t['yaw_rad']):.2f}°  FG={t['hz_l']:.1f}/{t['hz_r']:.1f}Hz  **ΔFG={t['hz_l']-t['hz_r']:+.1f}Hz**  PWM={t['duty_l']:.2f}/{t['duty_r']:.2f}  x/y={t['x_m']:.3f}/{t['y_m']:.3f}m\npose_valid={t['pose_valid']} 方向假设位={t['direction_assumed_mask']} 滑行/换向位={t['coasting_mask']}（均非独立方向测量）")
            if t['mode']==3 or not t['armed']:
                if self.manual is not None:self.note(f'设备已解除 ARM（mode={t["mode"]} armed={t["armed"]} fault={fault}），手动续发停止。')
                self.manual=None
            self.armed_now=bool(t['armed'])
            self.fault_now=int(t['fault'])
            if hasattr(self,'analyzer'):
                self.last_diagnosis=self.analyzer.feed(t,now=time.monotonic());self.show_diagnosis(self.last_diagnosis)
                if self.serial:self.status.set('已连接 '+self.port.get()+' · 遥测更新中')
            self.round_track(t)
            if t['mode']==4:self.summary.set('任务1台架：Pitch→双轮前后；Yaw仅观察；HOLD为近零停车（非地面平衡验收）\n'+self.summary.get())
            else:self.bench_seq=None
        if kind==0x91 and value:self.imu_text.set('\n\n'.join(f'{k}: {v}' for k,v in value.items()))
    def round_track(self,t):
        """Save one CSV per unified round, without any operator action.

        A round starts when the mission leaves idle/finished and ends on ARRIVED or
        a fault, so every run leaves a file to analyse afterwards.
        """
        st=t['state'];prev=getattr(self,'round_prev_state',None)
        self.round_prev_state=st
        if t.get('mode')!=4 and st in (1,3,5,7) and prev in (None,0,8,11,12) and getattr(self,'round_rows',None) is None:
            self.round_rows=[];self.round_name=time.strftime('run-%Y%m%d-%H%M%S')+f'-{time.time_ns()%1000000000:09d}'
            self.note(f'开始记录本轮→ PC/runs/{self.round_name}.csv')
        rows=getattr(self,'round_rows',None)
        if rows is not None:
            row=dict(t);row['pc_time']=round(time.monotonic()-self.start,3)
            row['dfg']=round(t['hz_l']-t['hz_r'],1)
            rows.append(row)
        if rows and st in (8,11,12) and prev!=st:self.finish_round(STATES[st])
        elif rows and st==0 and prev not in (None,0):self.finish_round('回到 IDLE，未观察到终态')
    def finish_round(self,reason):
        rows=getattr(self,'round_rows',None)
        if not rows:return
        if hasattr(self,'report'):
            analysis=analyze_round(rows);self.show_report('记录结束：'+reason+'\n\n'+format_round_report(analysis))
        if getattr(self,'replaying',False) or getattr(self,'demo',False):
            self.round_rows=None;return
        folder=Path(__file__).resolve().parent/'runs'
        try:
            folder.mkdir(exist_ok=True)
            path=folder/(self.round_name+'.csv')
            with open(path,'x',newline='',encoding='utf-8-sig') as file:
                writer=csv.DictWriter(file,fieldnames=list(rows[-1]));writer.writeheader();writer.writerows(rows)
            self.note(f'本轮已保存：PC/runs/{path.name}（{len(rows)} 条，{reason}）')
        except OSError as exc:
            self.note(f'本轮保存失败（数据仍在内存中，可重试断开）：{exc}');return
        self.round_rows=None
    def poll(self):
        now=time.monotonic()
        if self.demo:self.frame((0x90,0,demo_telemetry(now-self.start)))
        if self.serial:
            try:
                raw=self.serial.read(min(self.serial.in_waiting,4096))
                if raw:
                    if self.record:self.record.write(json.dumps({'t':now-self.start,'hex':raw.hex()})+'\n');self.record.flush()
                    for frame in self.parser.feed(raw,now):self.frame(frame)
                if self.manual and now-self.last_manual>=.1:self.send(*self.manual);self.last_manual=now
                if self.bench_seq is not None and now-self.last_bench>=.1:
                    self.send(12,struct.pack('<H',self.bench_seq));self.last_bench=now
                if self.config_jobs and self.config_wait is None:
                    k,v=self.config_jobs.pop(0);self.config_wait=self.send(3,struct.pack('<Hf',k,v))
                expired=self.model.expired(now)
                if expired:
                    self.note('ACK 超时 '+str(expired)+'；停止手动/台架续发，不自动重试启动。')
                    self.manual=None;self.config_jobs=[];self.config_wait=None;self.bench_pending=self.bench_seq=None
                if hasattr(self,'analyzer'):
                    warnings=self.analyzer.check_link(now=now,connected=True)
                    if warnings:
                        alert=warnings[0]
                        self.status.set(alert['title']+'：'+alert['detail'])
                        if hasattr(self,'insight'):self.show_diagnosis({'state_text':'链路异常','fault_text':'','alerts':warnings,'pose_note':'遥测过期时位置不可作为当前状态。'})
                        if hasattr(self,'metric_meta'):
                            for value in self.metric_meta.values():value.set('遥测未更新 · 数值不可作当前状态')
                elif self.last_rx and now-self.last_rx>1:self.status.set('遥测已过期，请检查链路。AUTO 不因遥测中断而取消。')
            except (OSError,ValueError) as exc:self.note(exc);self.disconnect()
        if now-self.last_plot>.1:self.draw();self.last_plot=now
        self.root.after(25,self.poll)
    def draw(self):
        c=self.canvas;c.delete('all');w=max(c.winfo_width(),1);h=max(c.winfo_height(),1);mid=h/2
        if w<100 or h<70:return
        for y in [h*.25,mid,h*.75]:c.create_line(18,y,w-18,y,fill='#30445e')
        samples=list(self.model.samples)[-300:]
        for key,color,scale in [('pitch_deg','#ffb47d',10),('yaw_rad','#62d8e9',.5),('duty_l','#b4e58a',1)]:
            points=[]
            for i,s in enumerate(samples):points.extend([18+i*max(w-36,1)/max(len(samples)-1,1),max(35,min(h-18,mid-s[key]/scale*h*.38))])
            if len(points)>3:c.create_line(*points,fill=color,width=2)
        c.create_text(18,12,anchor='nw',fill='#e4eef9',font=('Microsoft YaHei UI',10,'bold'),text='实时曲线  ·  最近 300 帧')
        c.create_text(18,h-8,anchor='sw',fill='#a9bdd2',text='Pitch ±10° 橙  /  Yaw ±0.5 rad 青  /  左 PWM ±1 绿（超范围截断）')
        if not hasattr(self,'map_canvas'):return
        m=self.map_canvas;m.delete('all');mw=max(m.winfo_width(),1);mh=max(m.winfo_height(),1)
        if mw<100 or mh<70:return
        compact=mh<190
        m.create_text(18,12,anchor='nw',fill='#e4eef9',font=('Microsoft YaHei UI',10,'bold'),text='估计坐标轨迹  ·  m')
        track=getattr(self,'round_rows',None) or list(self.model.samples)
        if track:
            points,bounds=trajectory_projection([(s['x_m'],s['y_m']) for s in track if 'x_m' in s and 'y_m' in s],mw,mh,50)
            xmin,xmax,ymin,ymax=bounds
            for frac in ((.5,) if compact else (.25,.5,.75)):
                gx=mw*frac;gy=mh*frac
                m.create_line(gx,40,gx,mh-36,fill='#30445e');m.create_line(44,gy,mw-20,gy,fill='#30445e')
                m.create_text(gx,mh-32,anchor='n',fill='#a9bdd2',font=('Consolas',8),text=f'{xmin+frac*(xmax-xmin):.2f}')
                m.create_text(40,gy,anchor='e',fill='#a9bdd2',font=('Consolas',8),text=f'{ymax-frac*(ymax-ymin):.2f}')
            m.create_text(mw-10,mh-8,anchor='se',fill='#e4eef9',text='X (m)')
            m.create_text(8,35 if compact else 55,anchor='nw',fill='#e4eef9',text='Y (m)')
            if len(points)>3:m.create_line(*points,fill='#58d6d1',width=2)
            if len(points)>=2:
                x,y=points[-2:];m.create_oval(x-5,y-5,x+5,y+5,fill='#ffd27e',outline='')
                if not compact:
                    s=track[-1];m.create_text(18,32,anchor='nw',fill='#a9bdd2',text=f"当前 x={s['x_m']:.3f}  y={s['y_m']:.3f}  ·  单路 FG 方向为假设")
        else:m.create_text(mw/2,mh/2,fill='#9db4cb',text='等待有效坐标遥测')
    def record_toggle(self):
        if self.record:self.record.close();self.record=None;self.note('已结束记录。');return
        path=filedialog.asksaveasfilename(defaultextension='.jsonl')
        if path:self.record=open(path,'w',encoding='utf-8');self.note('开始记录 RX 原始帧。')
    def replay(self):
        if self.serial or self.demo:self.note('请断开串口并关闭演示模式后回放。');return
        path=filedialog.askopenfilename(filetypes=[('原始帧','*.jsonl')])
        if not path:return
        try:
            if Path(path).stat().st_size>5000000:raise ValueError('回放文件上限 5 MB；请分段读取。')
            parser=Parser();self.replaying=True
            for line in Path(path).read_text(encoding='utf-8').splitlines():
                item=json.loads(line)
                for f in parser.feed(bytes.fromhex(item['hex']),float(item['t'])):self.frame(f)
            self.note('离线回放完成，不发送任何命令。');self.draw()
        except (ValueError,KeyError,OSError) as exc:self.note(exc)
        finally:
            self.finish_round('回放结束，未观察到终态');self.replaying=False
    def export(self):
        if not self.model.samples:return
        path=filedialog.asksaveasfilename(defaultextension='.csv')
        if path:
            with open(path,'w',newline='',encoding='utf-8-sig') as file:
                writer=csv.DictWriter(file,fieldnames=list(self.model.samples[-1]));writer.writeheader();writer.writerows(self.model.samples)
    def close(self):
        if self.manual or self.bench_pending is not None or self.bench_seq is not None:self.stop()
        self.disconnect()
        if self.record:self.record.close()
        self.root.destroy()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--demo',action='store_true');parser.add_argument('--smoke',action='store_true');a=parser.parse_args()
    root=tk.Tk();Monitor(root,a.demo)
    if a.smoke:root.after(500,root.destroy)
    root.mainloop()
if __name__=='__main__':main()
