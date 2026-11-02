"""Read-only bag input; write timing evidence and overview beside this script."""
import json
import math
from pathlib import Path
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit import ROOT, clock

output=Path(__file__).parent
evidence=json.loads((output/'evidence.json').read_text())
timing={}
series={}
for path in ROOT.rglob('*.bag'):
    data={'mocap_gaps_over_50ms':[], 'mocap_sequence_discontinuities':[], 'imu_near_backwards':[]}
    previous=None
    samples={k:[] for k in ('pwm','gyro','mocap','gap','state')}
    with rosbag.Bag(str(path)) as bag:
        for topic,m,t in bag.read_messages(topics=['/gimbalrotor/'+s for s in ('mocap/pose','imu','motor_pwms','flight_state')]):
            ts=t.to_sec()
            if topic.endswith('/mocap/pose'):
                row=(ts,m.header.stamp.to_sec(),m.header.seq)
                samples['mocap'].append((ts,m.pose.position.z))
                if previous:
                    delta=row[0]-previous[0]
                    samples['gap'].append((ts,delta*1000))
                    if delta>.05:
                        data['mocap_gaps_over_50ms'].append(dict(start=clock(previous[0]),end=clock(ts),receipt_gap_ms=delta*1000,
                            source_gap_ms=(row[1]-previous[1])*1000,seq_delta=row[2]-previous[2]))
                    if row[2]!=previous[2]+1:data['mocap_sequence_discontinuities'].append(clock(ts))
                previous=row
            elif topic.endswith('/imu'):
                import struct
                # Use the validated raw field layout from the offline audit.
                # Generated Imu uses acc_data and gyro_data arrays.
                import io
                buff=io.BytesIO();m.serialize(buff)
                v=struct.unpack('<II13f',buff.getvalue())
                samples['gyro'].append((ts,math.sqrt(sum(x*x for x in v[5:8]))))
                if 1793511590.69<ts<1793511590.74:
                    data['imu_near_backwards'].append(dict(receipt=clock(ts),stamp_ns=m.stamp.to_nsec()))
            elif topic.endswith('/motor_pwms'):samples['pwm'].append((ts,list(m.motor_value)))
            elif topic.endswith('/flight_state'):samples['state'].append((ts,m.data))
    timing[str(path)]=data
    if path.parent.name!='all':series=samples
usb=evidence['usb'][0]
back=usb['streams']['in']['backwards_imu'][0]
responses=usb['streams']['out']['timesync']
response=max((r for r in responses if r['time']<back['time']),key=lambda r:r['time'])
timing['usb_time_sync_relation']={'last_host_time_reply':response,'backwards_imu':back,
    'interpretation':'Backward timestamp is already in a valid USB serial frame just after sync; clock adjustment is the leading explanation, not proof of exact MCU execution.'}
(output/'timing.json').write_text(json.dumps(timing,indent=2)+'\n')
start=1793511494.331182
fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True,layout='constrained')
for i in range(4):axes[0].plot([t-start for t,v in series['pwm']],[v[i] for t,v in series['pwm']],label='Motor '+str(i+1),lw=1)
axes[0].set_ylabel('PWM equivalent');axes[0].legend(ncol=4,fontsize=8)
axes[1].plot([t-start for t,v in series['gyro']],[v for t,v in series['gyro']],lw=.7)
axes[1].set_ylabel('Gyro norm (rad/s)')
axes[2].plot([t-start for t,v in series['mocap']],[v for t,v in series['mocap']],lw=.8)
axes[2].set_ylabel('Mocap height (m)')
axes[3].plot([t-start for t,v in series['gap']],[v for t,v in series['gap']],lw=.5)
axes[3].set_ylabel('Mocap arrival gap (ms)');axes[3].set_xlabel('Seconds after 14:38:14.331 JST, November 1')
marks=[(1793511552.5136,'Arm'),(1793511553.5043,'Takeoff'),(1793511573.2942,'Hover'),(1793511583.0778,'Halt'),(1793511590.7160,'IMU time step')]
for ax in axes:
    for t,label in marks:ax.axvline(t-start,color='grey',ls='--',lw=.6)
    ax.grid(alpha=.2)
for t,label in marks:
    if label=='Takeoff':continue
    if label=='Arm':label='Arm / takeoff'
    axes[0].text(t-start,1.01,label,rotation=55,fontsize=8,transform=axes[0].get_xaxis_transform())
fig.suptitle('Nov 1 bench recording: controller active, no checksum-corruption event captured',fontsize=12)
fig.savefig(output/'overview.png',dpi=150)
print(output/'timing.json',output/'overview.png')
