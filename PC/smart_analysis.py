"""Advisory telemetry analysis; never sends commands or changes configuration."""
import math
import time

STATES={0:'空闲',1:'对准',2:'接近',3:'上板',4:'寻坡',5:'平衡',6:'保持',7:'返回',8:'到达',9:'路径',10:'泊位',11:'完成',12:'故障'}
FAULTS={1:('CONFIG','检查参数范围及标定标志。'),2:('IMU','检查 IMU 供电、连线和数据年龄。'),3:('FEEDBACK','检查速度反馈及连线。'),4:('TIMING','检查控制周期和系统负载。'),5:('BOUNDARY','检查坐标估计和场地边界。'),6:('TIMEOUT','检查所处阶段、目标条件与超时设置。'),7:('TILT','检查倾角及配置的倾角限值。'),8:('MOTOR','检查电机供电、驱动及 FG 反馈。'),9:('UART','检查串口线、波特率与供电。'),10:('STOPPED','确认停止来源。'),11:('POSE','检查位置有效标志与方向假设。')}


def _number(value):
    try:
        number=float(value)
        return number if math.isfinite(number) else None
    except (ValueError, TypeError, OverflowError):
        return None


def _int(value, default=None):
    n=_number(value)
    return int(n) if n is not None and n.is_integer() else default


def _alert(severity,title,detail,action):
    return dict(severity=severity,title=title,detail=detail,action=action)


class SmartAnalyzer:
    def __init__(self):
        self.last_rx=None
        self.low_feedback={'l':None,'r':None}

    def feed(self, sample, now=None):
        now=time.monotonic() if now is None else float(now)
        if self.last_rx is not None and (now-self.last_rx>.5 or now<self.last_rx):
            self.low_feedback={'l':None,'r':None}
        self.last_rx=now
        s=sample if isinstance(sample,dict) else {}
        state=_int(s.get('state'))
        fault=_int(s.get('fault'))
        alerts=[]
        warnings=_int(s.get('warnings'),0)
        for mask,title,detail,action in [
            (1,'FG 脉冲异常','异常脉冲区间未计入位置，位置已不可靠。','检查左右 FG 接线、干扰及实际轮速。'),
            (2,'任务计时超限','流程仍在尝试，不代表按时完成。','检查当前状态及现场计时。'),
            (4,'IMU 曾不可用','运行中出现姿态数据不可用；是否已恢复请看当前 IMU age。','检查 IMU 供电、连接与数据更新。'),
            (8,'控制周期异常','运行周期曾超出预期。','检查系统负载与运行记录。'),
            (16,'边界阈值超出','估计位置超出参考边界。','观察车体，必要时手动停止。'),
            (32,'倾角曾超限','运行中出现倾角超限，请结合当前姿态判断是否已恢复。','观察车体并检查倾角配置。'),
            (64,'串口错误','运行过程中发生串口错误。','检查通讯、供电与线路。'),
            (128,'位置估计丢失','计数已重新对齐，位置不再可信。','检查 FG 并核对实际位置。'),
            (256,'换向未就绪','保留换向互锁，正在从零输出重试。','观察轮子，检查反馈与机械阻力。')]:
            if warnings&mask:alerts.append(_alert('warning',title,detail,action))
        if fault is not None and fault!=0:
            name,advice=FAULTS.get(fault,('未知故障','记录故障码并核查固件定义。'))
            alerts.append(_alert('error',f'固件故障 {name}({fault})','这是固件上报的故障码，当前遥测并非故障发生瞬间。',advice+' 排除原因后再清除故障。'))
        age=_number(s.get('imu_age_ms'))
        if age is not None and age>200:
            age_text='尚未收到有效姿态' if age>=4294967295 else f'最近姿态距今 {age:.0f} ms'
            alerts.append(_alert('error','IMU 姿态数据过期',age_text+'；200 ms 为提示线，实际限值以固件配置为准。','先检查供电、接线及姿态数据更新；不要据旧姿态判断运动。'))
        for side,label in [('l','左'),('r','右')]:
            hz=_number(s.get('hz_'+side));duty=_number(s.get('duty_'+side))
            if hz is not None and abs(hz)>500:
                alerts.append(_alert('warning',f'{label}轮 FG 频率异常',f'观测到 {hz:.1f} Hz，超过 500 Hz 经验提示线；与电机型号和负载有关。','检查 FG 接线、供电、采样和干扰；先排除异常再依赖位置和速度，不能仅凭频率断定原因。'))
            active=_int(s.get('armed'),0)==1 and duty is not None and abs(duty)>=.4 and hz is not None and abs(hz)<1
            if active:
                if self.low_feedback[side] is None:self.low_feedback[side]=now
                if now-self.low_feedback[side]>=1.0:
                    alerts.append(_alert('warning',f'{label}轮反馈缺失或很低',f'PWM ≥ 40% 且 FG < 1 Hz 持续约 {now-self.low_feedback[side]:.1f} 秒；可能是轮未转、反馈掉线或测量异常。','观察轮子及 FG 线路；必要时停止，排除后再测。'))
            else:self.low_feedback[side]=None
        if _int(s.get('pose_valid'))==0:
            alerts.append(_alert('warning','位置估计无效','固件标记 pose_valid=0。','不要依靠该坐标做位置结论，检查姿态和反馈。'))
        if _int(s.get('coasting_mask'),0):
            alerts.append(_alert('info','存在滑行或换向','单路 FG 不独立测方向。','查看方向假设与实际轮向；位置是估计值。'))
        pose_note='坐标为单路 FG 与姿态推算，方向含假设、易受打滑及噪声影响；非外部定位。'
        return {'state_text':STATES.get(state,'未知' if state is None else f'未知状态 {state}'),
                'fault_text':FAULTS.get(fault,(('无' if fault==0 else '未知'),''))[0],
                'alerts':alerts,'pose_note':pose_note}

    def check_link(self, now=None, connected=False):
        now=time.monotonic() if now is None else float(now)
        if not connected:
            return [_alert('warning','连接已断开','上位机无法确认车辆当前状态；自主任务可能仍在执行。','按现场安全步骤处理；通信恢复后重新核对状态。')]
        if self.last_rx is None:
            return [_alert('warning','等待遥测','已连接但尚未收到有效遥测。','确认端口、蓝牙和串口配置。')]
        if now-self.last_rx>1:
            return [_alert('warning','遥测已过期',f'最近有效遥测已过去 {now-self.last_rx:.1f} 秒；自主任务可能仍在执行。','检查链路；不要把界面最后数值视为当前状态。')]
        return []


def analyze_round(rows):
    """Summarize sampled telemetry without interpolating missing observations."""
    rows=list(rows)
    analyzer=SmartAnalyzer();alerts=[];seen=set();durations={};elapsed=0.;valid_clock=len(rows)>=2;reset=False
    balance_observed=0.;balance_inside=0.;streak=0.;longest=0.;gap_count=0
    for index,row in enumerate(rows):
        if not isinstance(row,dict):continue
        if index:
            a=_int(rows[index-1].get('ms')) if isinstance(rows[index-1],dict) else None
            b=_int(row.get('ms'))
            if a is None or b is None or not 0<=a<=0xffffffff or not 0<=b<=0xffffffff:valid_clock=False
            else:
                delta=(b-a)&0xffffffff
                if delta>0x7fffffff:valid_clock=False;reset=True
                else:
                    dt=delta/1000.;elapsed+=dt
                    previous=_int(rows[index-1].get('state'))
                    label=STATES.get(previous, '未知状态')
                    durations[label]=durations.get(label,0.)+dt
                    if dt>.25:
                        gap_count+=1;streak=0.
                    elif previous==5:
                        pitch_previous=_number(rows[index-1].get('pitch_deg'))
                        if pitch_previous is None:streak=0.
                        else:
                            balance_observed+=dt
                            if abs(pitch_previous)<=1.0:
                                balance_inside+=dt;streak+=dt;longest=max(longest,streak)
                            else:streak=0.
                    else:streak=0.
        for alert in analyzer.feed(row,now=elapsed)['alerts']:
            if alert['title'] not in seen:
                seen.add(alert['title']);alerts.append(alert)
    if not valid_clock:durations={}
    pitch=[abs(n) for r in rows if isinstance(r,dict) for n in [_number(r.get('pitch_deg'))] if n is not None]
    fg=[abs(n) for r in rows if isinstance(r,dict) for k in ('hz_l','hz_r') for n in [_number(r.get(k))] if n is not None]
    xy=[(_number(r.get('x_m')),_number(r.get('y_m'))) for r in rows if isinstance(r,dict)]
    distance=0.;segments=0;prev=None
    for x,y in xy:
        if x is None or y is None:prev=None;continue
        if prev is not None:distance+=math.hypot(x-prev[0],y-prev[1]);segments+=1
        prev=(x,y)
    distance=distance if segments else None
    peak_pitch=max(pitch) if pitch else None;peak_fg=max(fg) if fg else None
    fmt=lambda value,unit:('数据不足' if value is None else f'{value:.2f} {unit}')
    summary=[f'样本数：{len(rows)}；持续时间：{fmt(elapsed if valid_clock else None,"s")}。',
             f'Pitch 最大绝对值：{fmt(peak_pitch,"°")}；FG 最大频率：{fmt(peak_fg,"Hz")}。',
             f'估计轨迹累计长度：{fmt(distance,"m")}（单路 FG 方向假设，非独立定位）。']
    if durations:summary.append('阶段采样时长：'+'，'.join(f'{k} {v:.1f}s' for k,v in durations.items()))
    if balance_observed and valid_clock:
        summary.append(f'平衡阶段观测：±1° 内约 {100*balance_inside/balance_observed:.1f}%，最长连续约 {longest:.2f}s（按采样估算，非设备实际稳定判据）。')
    if gap_count:summary.append(f'存在 {gap_count} 个超过 0.25s 的采样间隔，可能丢失遥测；平衡统计已排除这些间隔，阶段时长仍包含间隔。')
    if reset:summary.append('检测到设备时间回退或重置，本轮时长和阶段时长不可用。')
    if alerts:summary.append('提示：'+'；'.join(a['title'] for a in alerts))
    if not rows:summary.append('没有样本，数据不足。')
    return {'sample_count':len(rows),'duration_s':elapsed if valid_clock else None,'state_durations':durations,
            'pitch_peak_deg':peak_pitch,'fg_peak_hz':peak_fg,'distance_m':distance,'alerts':alerts,
            'balance_observed_s':balance_observed if valid_clock else None,
            'balance_in_band_s':balance_inside if valid_clock else None,
            'balance_longest_s':longest if valid_clock else None,
            'summary':'\n'.join(summary)}
