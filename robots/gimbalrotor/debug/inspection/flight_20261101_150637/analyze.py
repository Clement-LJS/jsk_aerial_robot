"""Offline flight investigation; writes only beside this script."""
from pathlib import Path
import sys,json,math
from collections import defaultdict
import numpy as np
import rosbag
sys.path.insert(0,str(Path(__file__).parent.parent/'rosserial_20261101'))
from audit import bags, clock

ROOT=Path('/home/clement/rosbag/year2026/11Nov/261101')
PATHS=[ROOT/'all/1_realflight.bag',ROOT/'1_realflight/2026-11-01-15-06-40_0.bag']
OUT=Path(__file__).parent


def euler(q):
    x,y,z,w=q
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1,min(1,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]


def qlist(q):return [q.x,q.y,q.z,q.w]


def series():
    s=defaultdict(list);messages=defaultdict(list)
    with rosbag.Bag(str(PATHS[0])) as bag:
        for topic,m,ts in bag.read_messages():
            t=ts.to_sec();k=topic.removeprefix('/gimbalrotor/')
            if k=='imu':
                import io,struct
                b=io.BytesIO();m.serialize(b);v=struct.unpack('<II13f',b.getvalue())
                s['imu'].append([t,m.stamp.to_sec(),*v[2:8],*euler(v[11:15])])
            elif k=='motor_pwms':s['pwm'].append([t,*m.motor_value])
            elif k in ('mocap/pose','uav/baselink/odom','uav/cog/odom'):
                pose=m.pose.pose if hasattr(m.pose,'pose') else m.pose
                row=[t,m.header.stamp.to_sec(),m.header.seq,pose.position.x,pose.position.y,pose.position.z,*euler(qlist(pose.orientation))]
                if hasattr(m,'twist'):row.extend([m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z])
                s[k].append(row)
            elif k=='servo/states':
                row=[t,m.stamp.to_sec()]+[servo.angle for servo in m.servos]
                s['servo'].append(row)
            elif k=='servo/target_states_debug':
                values=dict(zip(m.index,m.angles))
                if all(i in values for i in range(4)):
                    s['target'].append([t,*[values[i] for i in range(4)]])
                messages['servo_target_index_sets'].append(list(m.index))
            elif k=='joint_states':s['joints'].append([t,*m.position])
            elif k=='debug/pose/pid':
                row=[t,m.header.stamp.to_sec()]
                for ax in ('x','y','z','roll','pitch','yaw'):
                    v=getattr(m,ax);row.extend([v.target_p,v.err_p,v.target_d,v.err_d,*[sum(getattr(v,p)) for p in ('total','p_term','i_term','d_term')]])
                s['pid'].append(row)
            elif k=='four_axes/command':s['command'].append([t,*m.angles,*m.base_thrust])
            elif k=='torque_allocation_matrix_inv':s['allocation'].append([t,*[v for row in m.rows for v in (row.x,row.y,row.z)]])
            elif k=='battery_voltage_status':s['voltage'].append([t,m.data])
            elif k=='flight_state':s['state'].append([t,m.data])
            elif k=='uav/nav':messages['nav'].append(dict(time=clock(t),data=str(m)))
            elif k=='perching/roll_yaw/state':messages['perching'].append(m.data)
            elif 'parameter_updates' in k:
                messages[k].append({p.name:p.value for p in m.doubles})
    return {k:np.asarray(v) for k,v in s.items()},messages


if __name__=='__main__':
    audit,_=bags(PATHS)
    (OUT/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    s,messages=series()
    np.savez_compressed(OUT/'series.npz',**s)
    result={'nav':messages['nav'],'perching_states':sorted(set(messages['perching'])),
            'gains':{k:v for k,v in messages.items() if 'parameter_updates' in k},'peaks':{},'mocap_gaps':[],'phases':{}}
    for k,a in s.items():
        if len(a):
            result['peaks'][k]=[dict(column=i,min=float(a[:,i].min()),max=float(a[:,i].max()),
                max_at=clock(a[np.argmax(a[:,i]),0]),min_at=clock(a[np.argmin(a[:,i]),0])) for i in range(1,a.shape[1])]
    for k in ('mocap/pose','imu','servo','uav/baselink/odom','pid','command'):
        a=s[k];gaps=np.diff(a[:,0]);idx=np.argsort(gaps)[-10:][::-1]
        result[k+'_largest_receipt_gaps']=[dict(start=clock(a[i,0]),end=clock(a[i+1,0]),gap=float(gaps[i]),
            source_gap=float(a[i+1,1]-a[i,1]) if k in ('mocap/pose','imu','servo','uav/baselink/odom','pid') else None) for i in idx]
    for start,end in [(0,15),(15,27),(27,38),(38,48),(48,58),(58,65),(65,70),(70,76)]:
        b=1793513197.256713
        phase={}
        for k in ('imu','pwm','mocap/pose','uav/cog/odom','pid'):
            a=s[k];a=a[(a[:,0]>=b+start)&(a[:,0]<b+end)]
            if len(a):phase[k]={'min':a[:,1:].min(axis=0).tolist(),'max':a[:,1:].max(axis=0).tolist()}
        result['phases'][str((start,end))]=phase
    (OUT/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(OUT/'metrics.json')
