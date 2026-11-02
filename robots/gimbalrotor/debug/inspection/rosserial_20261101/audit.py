"""Offline audit of Nov 1 bags and FT232R USB capture; never accesses hardware.

PCAP layout: https://www.tcpdump.org/linktypes/LINKTYPE_USB_LINUX_MMAPPED.html
FT232R IN packets have two status bytes per 64-byte USB packet. Endpoint size
was checked at /sys/bus/usb/devices/3-6:1.0/ep_81/wMaxPacketSize (0040).
USB OUT submissions and IN completions are counted once; submissions' -115
is EINPROGRESS, not a failed completion. Boundary fragments are kept separate.
"""
import bisect
from collections import Counter, defaultdict
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import sys
from zoneinfo import ZoneInfo
import rosbag

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'simulate_run'))
from summarize_capture import summarize
from payload_checks import inspect_payload

ROOT = Path('/home/clement/rosbag/year2026/11Nov/261101')


def clock(t):
    return datetime.fromtimestamp(t, ZoneInfo('Asia/Tokyo')).isoformat(timespec='microseconds')


def floats(value, prefix=''):
    if isinstance(value, float):
        yield prefix, value
    elif isinstance(value, (list, tuple)):
        for i, item in enumerate(value):
            yield from floats(item, prefix + '[' + str(i) + ']')
    elif hasattr(value, '__slots__'):
        for key in value.__slots__:
            yield from floats(getattr(value, key), prefix + '.' + key)


def bags(paths=None):
    result = {}
    payload_topics = defaultdict(Counter)
    for path in sorted(ROOT.rglob('*.bag') if paths is None else paths):
        info = dict(audit=summarize([path]), timeline=[], nonfinite=[], decode_errors=[],
                    stamp_backwards=[], logs=[], phase={}, servo={}, joints={}, topics={})
        state = 0
        values = defaultdict(list)
        by_phase = defaultdict(lambda: defaultdict(list))
        stamps = {}
        previous = {}
        arrivals = defaultdict(list)
        servo_errors = Counter()
        with rosbag.Bag(str(path)) as bag:
            info['start'], info['end'] = clock(bag.get_start_time()), clock(bag.get_end_time())
            for topic, raw, ts in bag.read_messages(raw=True):
                t = ts.to_sec()
                arrivals[topic].append(t)
                payload = raw[1]
                if len(payload) <= 2048 and not topic.startswith('/rosout'):
                    payload_topics[payload][topic] += 1
                try:
                    m = raw[-1]().deserialize(payload)
                except Exception as exc:
                    info['decode_errors'].append(dict(topic=topic, time=clock(t), error=str(exc)))
                    continue
                if 'parameter_' not in topic:
                    for field, v in floats(m):
                        if not math.isfinite(v) and len(info['nonfinite']) < 100:
                            info['nonfinite'].append(dict(topic=topic, field=field, time=clock(t), value=str(v)))
                if topic == '/rosout' and not m.name.startswith('/record_'):
                    info['logs'].append(dict(time=clock(t), node=m.name, level=m.level, text=m.msg))
                kind = topic.removeprefix('/gimbalrotor/')
                if kind == 'flight_state':
                    if not info['timeline'] or state != m.data:
                        info['timeline'].append(dict(time=clock(t), state=m.data))
                    state = m.data
                if kind.startswith('teleop_command/') or kind in ('flight_config_cmd', 'flight_config_ack'):
                    info['timeline'].append(dict(time=clock(t), topic=kind, message=str(m)))
                stamp = getattr(m, 'stamp', getattr(getattr(m, 'header', None), 'stamp', None))
                if stamp is not None and stamp.to_nsec():
                    s = stamp.to_nsec()
                    if topic in stamps and s < stamps[topic]:
                        info['stamp_backwards'].append(dict(topic=topic, time=clock(t), delta_ns=s-stamps[topic]))
                    stamps[topic] = s
                if kind == 'imu':
                    parsed = inspect_payload(kind, payload)
                    for k in ('gyro_norm', 'acc_norm', 'quaternion_norm'):
                        if k in parsed:
                            values[k].append(parsed[k]); by_phase[str(state)][k].append(parsed[k])
                if kind == 'motor_pwms':
                    p = inspect_payload(kind, payload)
                    for i, v in enumerate(p.get('values', [])):
                        values['pwm'+str(i)].append(v); by_phase[str(state)]['pwm'+str(i)].append(v)
                    info.setdefault('pwm_examples', [])
                    v = tuple(p.get('values', []))
                    if v != previous.get(kind) and len(info['pwm_examples']) < 15:
                        info['pwm_examples'].append(dict(time=clock(t), values=v))
                    previous[kind] = v
                if kind == 'battery_voltage_status': values['voltage'].append(m.data)
                if kind == 'servo/states':
                    for s in m.servos:
                        for k in ('angle', 'temp', 'load'):
                            values['servo'+str(s.index)+'_'+k].append(getattr(s,k))
                        if s.error:servo_errors[str(s.index)+':'+str(s.error)] += 1
                if kind in ('mocap/pose', 'uav/baselink/odom'):
                    pose = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
                    for axis in ('x','y','z'):
                        values[kind+'_'+axis].append(getattr(pose.position,axis))
                    q=pose.orientation
                    values[kind+'_quaternion_norm'].append(math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w))
                if kind == 'joint_states':
                    for name, pos in zip(m.name, m.position):values['joint_'+name].append(pos)
                if kind == 'servo/target_states_debug':
                    for idx, angle in zip(m.index,m.angles):values['target'+str(idx)].append(angle)
        def stats(v):return dict(count=len(v), minimum=min(v), maximum=max(v), mean=sum(v)/len(v))
        info['ranges']={k:stats(v) for k,v in values.items() if v}
        info['phase']={p:{k:stats(v) for k,v in d.items() if v} for p,d in by_phase.items()}
        info['servo_errors']=dict(servo_errors)
        for topic, times in arrivals.items():
            gaps=[b-a for a,b in zip(times,times[1:])]
            info['topics'][topic]=dict(count=len(times), max_receipt_gap_s=max(gaps,default=None),
                                     rate_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None)
        result[str(path)] = info
    return result, payload_topics


def frames(data, ends, times, matches):
    pos=0; valid=0; first=True; lead=0; skipped=0; bad_header=[]; bad_checksum=[]
    counts=Counter(); maps=defaultdict(Counter); backwards=[]; stamps={}; timesync=[]; unmatched_imu=[]
    while pos+8<=len(data):
        start=data.find(b'\xff\xfe', pos)
        if start<0:break
        if first:lead+=start-pos
        else:skipped+=start-pos
        pos=start
        if pos+8>len(data):break
        length=data[pos+2] | data[pos+3]<<8
        t=times[bisect.bisect_right(ends,pos)]
        if sum(data[pos+2:pos+5]) & 255 != 255:
            bad_header.append(dict(time=clock(t),offset=pos));pos+=1;continue
        finish=pos+8+length
        if finish>len(data):break
        topic=struct.unpack_from('<H',data,pos+5)[0]
        payload=data[pos+7:finish-1]
        if sum(data[pos+5:finish]) & 255 != 255:
            bad_checksum.append(dict(time=clock(t),offset=pos,topic_id=topic,length=length))
        else:
            valid+=1;counts[(topic,length)]+=1
            for name,n in matches.get(payload,{}).items():maps[topic][name]+=1
            if topic == 10:timesync.append(dict(time=clock(t),length=length,payload_hex=payload.hex()))
            if topic == 153 and len(payload)==60 and '/gimbalrotor/imu' not in matches.get(payload,{}):
                unmatched_imu.append(dict(time=clock(t),payload_hex=payload.hex()))
            if '/gimbalrotor/imu' in matches.get(payload,{}) and len(payload)==60:
                sec,ns=struct.unpack_from('<II',payload);stamp=sec*10**9+ns
                if topic in stamps and stamp<stamps[topic]:backwards.append(dict(time=clock(t),topic_id=topic,delta_ns=stamp-stamps[topic]))
                stamps[topic]=stamp
        pos=finish;first=False
    return dict(valid_frames=valid,leading_fragment_bytes=lead,trailing_bytes=len(data)-pos,
                interior_skipped_bytes=skipped,bad_header=bad_header,bad_checksum=bad_checksum,
                counts={str(k):v for k,v in counts.items()},
                exact_payload_topic_matches={str(k):dict(v) for k,v in maps.items()},
                backwards_imu=backwards,timesync=timesync,unmatched_imu=unmatched_imu)


def usb(path,matches):
    chunks=defaultdict(list);ends=defaultdict(list);times=defaultdict(list)
    totals=Counter(); statuses=Counter(); line_status=Counter();errors=[]; trunc=[]; n=0; first=None;last=None
    with path.open('rb') as f:
        header=f.read(24)
        if header[:4]!=b'\xd4\xc3\xb2\xa1' or struct.unpack_from('<I',header,20)[0]!=220:
            raise ValueError('Expected little-endian microsecond PCAP, linktype 220')
        while True:
            h=f.read(16)
            if not h:break
            if len(h)!=16:raise ValueError('Truncated PCAP record header')
            sec,usec,size,original=struct.unpack('<IIII',h)
            b=f.read(size)
            if len(b)!=size or size<64:raise ValueError('Truncated USB record')
            t=sec+usec/1e6;first=t if first is None else first;last=t;n+=1
            ev,transfer,ep,device=b[8:12]
            status,requested,captured=struct.unpack_from('<iII',b,28)
            statuses[(chr(ev),ep,status)]+=1
            if ev in (ord('C'),ord('E')) and status:errors.append(dict(time=clock(t),endpoint=ep,status=status))
            if size!=original:trunc.append(n)
            if transfer!=3 or b[15]!=0:continue
            if captured!=size-64:raise ValueError('USB captured length mismatch')
            payload=b[64:]
            if ep==0x81 and ev==ord('C'):
                parts=[]
                for offset in range(0,len(payload),64):
                    part=payload[offset:offset+64]
                    if len(part)<2:raise ValueError('Incomplete FTDI status')
                    line_status[part[1]]+=1
                    parts.append(part[2:])
                payload=b''.join(parts);direction='in'
            elif ep==2 and ev==ord('S'):direction='out'
            else:continue
            if payload:
                chunks[direction].append(payload);totals[direction]+=len(payload)
                ends[direction].append(totals[direction]);times[direction].append(t)
    streams={d:frames(b''.join(v),ends[d],times[d],matches) for d,v in chunks.items()}
    return dict(path=str(path),start=clock(first),end=clock(last),records=n,
                completion_errors=errors,truncated_records=trunc,
                urb_statuses={str(k):v for k,v in statuses.items()},
                ftdi_line_status={hex(k):v for k,v in line_status.items()},
                line_error_packets=sum(v for k,v in line_status.items() if k & 0x9e),
                serial_bytes=dict(totals),streams=streams)


if __name__=='__main__':
    bag_results,matches=bags()
    result=dict(bags=bag_results,usb=[usb(p,matches) for p in ROOT.rglob('*.pcap')])
    target=Path(__file__).with_name('evidence.json')
    target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(target)
