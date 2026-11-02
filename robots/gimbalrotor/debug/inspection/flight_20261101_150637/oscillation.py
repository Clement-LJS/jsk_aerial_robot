"""Offline window metrics and target/feedback comparison; not a control change."""
from pathlib import Path
import json,sys
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).parent.parent/'rosserial_20261101'))
from audit import clock
from analyze import ROOT,PATHS
P=Path(__file__).parent;s=np.load(P/'series.npz');base=1793513197.256713
def at(time):return datetime.fromisoformat('2026-11-01T'+time).replace(tzinfo=ZoneInfo('Asia/Tokyo')).timestamp()
windows=[(at(a),at(b)) for a,b in [('15:07:05','15:07:20'),('15:07:30','15:07:35'),
    ('15:07:35','15:07:39'),('15:07:39','15:07:42'),('15:07:42','15:07:44.900'),('15:07:44.985','15:07:47.190')]]
def subset(k,a,b):
    x=s[k];return x[(x[:,0]>=a)&(x[:,0]<b)]
result={'windows':[],'spectra':{},'tracking':{},'historical_errors':[],'gaps':{}}
for a,b in windows:
    imu=subset('imu',a,b);j=subset('joints',a,b);tar=subset('target',a,b);od=subset('uav/cog/odom',a,b);pid=subset('pid',a,b)
    result['windows'].append(dict(start=clock(a),end=clock(b),
      roll_deg=[float(v) for v in np.degrees([od[:,6].min(),od[:,6].max()])],
      yaw_deg=[float(v) for v in np.degrees([od[:,8].min(),od[:,8].max()])],
      gyro_roll_peak=float(np.abs(imu[:,5]).max()),
      gimbal4_feedback_deg=np.degrees([j[:,4].min(),j[:,4].max()]).tolist(),
      gimbal4_target_deg=np.degrees((np.array([tar[:,4].min(),tar[:,4].max()])-2047)*.001534).tolist(),
      yaw_target_deg=np.degrees([pid[:,42].min(),pid[:,42].max()]).tolist()))
a,b=at('15:07:39.700'),at('15:07:44.900')
grid=np.arange(a,b,.005)
for name,k,col in [('roll_rate','imu',5),('yaw_rate','imu',7),('gimbal4_feedback','joints',4),('gimbal4_target','target',4)]:
    x=s[k];y=np.interp(grid,x[:,0],x[:,col]);y-=np.mean(y)
    power=np.abs(np.fft.rfft(y*np.hanning(len(y))))**2;freq=np.fft.rfftfreq(len(y),.005)
    mask=(freq>=.5)&(freq<=5);pick=np.flatnonzero(mask)[np.argmax(power[mask])]
    result['spectra'][name]={'start':clock(a),'end':clock(b),'peak_hz':float(freq[pick]),'frequency_resolution_hz':float(freq[1])}
for i in range(4):
    j=s['servo'];tar=s['target'];grid=np.arange(a+.25,b-.25,.01)
    actual=np.interp(grid,j[:,0],j[:,i+2]);lags=np.arange(-.2,.201,.005)
    correlations=[float(np.corrcoef(actual,np.interp(grid-lag,tar[:,0],tar[:,i+1]))[0,1]) for lag in lags]
    best=int(np.argmax(correlations));target_now=np.interp(grid,tar[:,0],tar[:,i+1])
    error_deg=(target_now-actual)*.001534*180/np.pi
    result['tracking'][str(i+1)]=dict(best_lag_s=float(lags[best]),correlation=correlations[best],
        rms_unshifted_error_deg=float(np.sqrt(np.mean(error_deg**2))),max_abs_unshifted_error_deg=float(np.max(np.abs(error_deg))),
        caveat='Bag-arrival alignment and periodic correlation; approximate feedback delay, not isolated physical servo latency.')
for path in PATHS:
    gaps={};last={};count={};seqgaps=[]
    with rosbag.Bag(str(path)) as bag:
        for topic,m,t in bag.read_messages(topics=['/gimbalrotor/mocap/pose','/gimbalrotor/imu','/gimbalrotor/debug/pose/pid']):
            stamp=getattr(m,'stamp',getattr(getattr(m,'header',None),'stamp',None)).to_sec()
            key=topic;row=(t.to_sec(),stamp)
            if key in last:
                gap=row[0]-last[key][0];sgap=row[1]-last[key][1]
                if gap>.05 or sgap>.05:gaps.setdefault(key,[]).append(dict(start=clock(last[key][0]),end=clock(row[0]),receipt_gap=gap,source_gap=sgap))
            last[key]=row
    result['gaps'][str(path)]=gaps
meta=json.loads((ROOT/'1_realflight/metadata_end.json').read_text())
for line in meta.get('bridge_log_tail',[]):
    if 'checksum' in line.lower():result['historical_errors'].append(dict(time=clock(float(line.split()[0])),text=line.rsplit('] ',1)[-1]))
result['recording_host']={'kernel':meta['kernel'],'log_source':meta.get('bridge_log_source'),'nodes':meta['nodes']}
(P/'oscillation.json').write_text(json.dumps(result,indent=2)+'\n')
fig,ax=plt.subplots(4,1,sharex=True,figsize=(12,10),layout='constrained')
start,end=at('15:07:10'),at('15:07:47.300')
j=subset('joints',start,end);tar=subset('target',start,end);imu=subset('imu',start,end);pid=subset('pid',start,end);od=subset('uav/cog/odom',start,end)
ax[0].plot(tar[:,0]-start,(tar[:,4]-2047)*.001534*180/np.pi,label='Gimbal 4 target')
ax[0].plot(j[:,0]-start,j[:,4]*180/np.pi,label='Gimbal 4 feedback')
ax[0].set_ylabel('Angle (deg)')
ax[1].plot(imu[:,0]-start,imu[:,5],label='Roll gyro');ax[1].plot(imu[:,0]-start,imu[:,7],label='Yaw gyro');ax[1].set_ylabel('Rate (rad/s)')
ax[2].plot(od[:,0]-start,np.degrees(od[:,8]),label='Measured yaw');ax[2].plot(pid[:,0]-start,np.degrees(pid[:,42]),ls='--',label='Yaw target');ax[2].set_ylabel('Yaw (deg)')
for axis,offset in [('roll',26),('yaw',42)]:
    ax[3].plot(pid[:,0]-start,pid[:,offset+7],label=axis+' D term')
ax[3].set_ylabel('Recorded pose PID D output');ax[-1].set_xlabel('Seconds after 15:07:10 JST')
for x in ax:
    x.grid(alpha=.2);x.legend(loc='upper left',ncol=2)
    x.axvline(at('15:07:44.985')-start,c='grey',ls='--');x.axvline(at('15:07:47.190')-start,c='red',ls='--')
fig.suptitle('Growing commanded/actual gimbal oscillation; grey = land, red = halt')
fig.savefig(P/'oscillation_detail.png',dpi=150)
print(json.dumps({k:v for k,v in result.items() if k not in ('gaps','historical_errors')},indent=2))
