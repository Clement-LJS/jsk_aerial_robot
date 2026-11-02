"""Read-only, bounded provenance for a local ROS capture; no device opens."""
from collections import deque
from datetime import datetime
import os
from pathlib import Path
import platform
import subprocess
import xmlrpc.client


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2
        return connection


def usb_devices():
    result = []
    for path in sorted(Path('/sys/bus/usb/devices').glob('*')):
        try:
            if (path / 'idVendor').read_text().strip() != '0403':
                continue
            item = {'sysfs': str(path)}
            for field in ('idVendor', 'idProduct', 'serial', 'product', 'busnum', 'devnum'):
                file = path / field
                if file.exists():
                    item[field] = file.read_text().strip()
            result.append(item)
        except OSError:
            continue
    return result


def snapshot(topics):
    result = dict(time=datetime.now().astimezone().isoformat(), kernel=platform.release(),
                  usb=usb_devices(), parameters={}, errors=[])
    caller = '/rosserial_capture_metadata'
    master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                                      transport=TimeoutTransport())
    try:
        code, _, graph = master.getSystemState(caller)
        if code != 1:
            raise RuntimeError('ROS master did not return a graph')
        result['publishers'] = {t: nodes for t, nodes in graph[0] if t in topics}
        result['subscribers'] = {t: nodes for t, nodes in graph[1] if t in topics}
        code, _, types = master.getTopicTypes(caller)
        result['topic_types'] = {t: typ for t, typ in types if t in topics} if code == 1 else {}
        result['unadvertised_topics'] = sorted(set(topics) - set(result['publishers']))
        for param in ('/use_sim_time', '/gimbalrotor/rosserial_server',
                      '/gimbalrotor/motor_info', '/gimbalrotor/aerial_robot_control_name',
                      '/gimbalrotor/flight_navigation_plugin_name',
                      '/gimbalrotor/simulate_run/bench_profile'):
            code, _, value = master.getParam(caller, param)
            if code == 1:
                result['parameters'][param] = value
        result['nodes'] = {}
        for node in ('/gimbalrotor/rosserial_server', '/gimbalrotor/aerial_robot_base_node'):
            code, _, uri = master.lookupNode(caller, node)
            if code == 1:
                proxy = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport())
                code, _, pid = proxy.getPid(caller)
                result['nodes'][node] = {'uri': uri, 'pid': pid if code == 1 else None}
    except Exception as exc:
        result['errors'].append(str(exc))
    root = next(p for p in Path(__file__).resolve().parents if (p / 'package.xml').is_file())
    for key, command in (('git_head', ['git', 'rev-parse', 'HEAD']),
                         ('git_branch', ['git', 'branch', '--show-current']),
                         ('git_status', ['git', 'status', '--short'])):
        try:
            result[key] = subprocess.check_output(command, cwd=root, text=True,
                                                 stderr=subprocess.DEVNULL, timeout=3)[:16384].strip()
        except (OSError, subprocess.SubprocessError) as exc:
            result['errors'].append(str(exc))
    # Preserve relevant recent local startup/error lines; not the whole ROS log.
    log = Path(os.environ.get('ROS_LOG_DIR', str(Path.home()/'.ros/log'))) / 'latest/rosout.log'
    try:
        with log.open('rb') as stream:
            stream.seek(0, 2)
            truncated = stream.tell() > 2 * 1024 * 1024
            stream.seek(max(0, stream.tell() - 2 * 1024 * 1024))
            if truncated:
                stream.readline()
            lines = stream.read().decode('utf-8', errors='replace').splitlines()
        result['bridge_log_tail'] = list(deque((line[:4096] for line in lines
            if '/gimbalrotor/rosserial_server' in line and any(s in line.lower() for s in
                ('publisher name:', 'subscriber name:', 'checksum', 'sync', 'serial port', 'deserializ'))), maxlen=100))
        result['bridge_log_source'] = str(log.resolve())
        result['bridge_log_search_truncated'] = truncated
    except OSError as exc:
        result['errors'].append('ROS log: ' + str(exc))
    return result
