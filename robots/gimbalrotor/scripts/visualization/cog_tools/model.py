"""Expanded URDF reader and component-aware FK; no ROS imports at module scope."""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .math3d import vector, transform, motion, inertia_tensor, validate_transform


def numbers(text, default=(0, 0, 0)):
    return vector(str(text).split() if text is not None else default)


def origin(element):
    return transform() if element is None else transform(numbers(element.get('xyz')), numbers(element.get('rpy')))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def resource_path(uri, package_root):
    if uri.startswith('package://'):
        pkg, rel = uri[10:].split('/', 1)
        if pkg == 'gimbalrotor':
            return Path(package_root) / rel
        import rospkg
        return Path(rospkg.RosPack().get_path(pkg)) / rel
    if uri.startswith('file://'):
        return Path(uri[7:])
    p = Path(uri)
    return p if p.is_absolute() else Path(package_root) / p


@dataclass(frozen=True)
class Visual:
    name: str
    pose: np.ndarray
    kind: str
    size: np.ndarray
    uri: str = ''
    mesh_hash: str = ''

    @property
    def fingerprint(self):
        return digest((self.uri + repr(self.size.tolist()) + repr(self.pose.tolist())).encode())


@dataclass(frozen=True)
class Link:
    name: str
    mass: object
    inertial: np.ndarray
    tensor: object
    visuals: tuple
    diagnostics: tuple = ()


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    pose: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    mimic: object = None


class Model:
    def __init__(self, xml, package_root, variant='unregistered', source=''):
        doc = ET.fromstring(xml)
        if doc.tag != 'robot' or any('xacro' in e.tag or '${' in str(e.attrib) for e in doc.iter()):
            raise ValueError('Expected expanded URDF robot, not a Xacro definition')
        self.xml, self.variant, self.source = xml, variant, source
        self.fingerprint = digest(xml.encode())
        self.links, self.joints, self.parent, self.children = {}, {}, {}, {}
        self.diagnostics = []
        mesh_hashes = {}
        for el in doc.findall('link'):
            name = el.attrib['name']
            if name in self.links:
                raise ValueError('Duplicate link: ' + name)
            inert = el.find('inertial')
            mass, tensor, warnings = None, None, []
            if inert is not None:
                mass = float(inert.find('mass').attrib['value'])
                if not math.isfinite(mass) or mass <= 0:
                    warnings.append('Nonpositive/nonfinite declared mass')
                ie = inert.find('inertia')
                if ie is not None:
                    v = {k: float(ie.attrib[k]) for k in ('ixx','ixy','ixz','iyy','iyz','izz')}
                    tensor = np.array([[v['ixx'],v['ixy'],v['ixz']], [v['ixy'],v['iyy'],v['iyz']], [v['ixz'],v['iyz'],v['izz']]])
                    try:
                        inertia_tensor(tensor)
                    except ValueError as exc:
                        warnings.append(str(exc))
                        tensor = None
            visuals = []
            for idx, ve in enumerate(el.findall('visual')):
                geo = ve.find('geometry')
                if geo is None or len(geo) != 1:
                    raise ValueError('Invalid visual geometry for ' + name)
                shape = geo[0]
                uri, mh = '', ''
                if shape.tag == 'mesh':
                    uri = shape.attrib['filename']
                    size = numbers(shape.get('scale'), (1, 1, 1))
                    path = resource_path(uri, package_root)
                    if not path.is_file():
                        raise ValueError('Unresolved mesh resource: ' + uri)
                    if path not in mesh_hashes:
                        mesh_hashes[path] = digest(path.read_bytes())
                    mh = mesh_hashes[path]
                    if np.any(size <= 0):
                        warnings.append('Nonpositive mesh scale: calibration unavailable')
                elif shape.tag == 'box':
                    size = numbers(shape.attrib['size'])
                elif shape.tag == 'sphere':
                    size = np.full(3, 2*float(shape.attrib['radius']))
                elif shape.tag == 'cylinder':
                    size = np.array([2*float(shape.attrib['radius'])]*2 + [float(shape.attrib['length'])])
                else:
                    raise ValueError('Unsupported geometry: ' + shape.tag)
                visuals.append(Visual(ve.get('name', str(idx)), origin(ve.find('origin')), shape.tag, vector(size), uri, mh))
            self.links[name] = Link(name, mass, origin(None if inert is None else inert.find('origin')), tensor, tuple(visuals), tuple(warnings))
        if not self.links:
            raise ValueError('Model has no links')
        for el in doc.findall('joint'):
            name, kind = el.attrib['name'], el.attrib['type']
            p, c = el.find('parent').attrib['link'], el.find('child').attrib['link']
            if name in self.joints or c in self.parent or p not in self.links or c not in self.links:
                raise ValueError('Duplicate joint/multiple parents/missing link: ' + name)
            axis_el = el.find('axis')
            axis = numbers(None if axis_el is None else axis_el.get('xyz'), (1, 0, 0))
            if kind in ('revolute','continuous','prismatic'):
                motion(kind, axis, 0)
                axis = axis / np.linalg.norm(axis)
            elif kind != 'fixed':
                if kind not in ('planar', 'floating'):
                    raise ValueError('Unknown URDF joint type: '+kind)
                self.diagnostics.append('Unsupported joint %s (%s): subtree pose unavailable' % (name, kind))
            lo, hi = -math.pi, math.pi
            if kind in ('prismatic','revolute'):
                lim = el.find('limit')
                if lim is None:
                    raise ValueError('Missing joint limits: ' + name)
                lo, hi = float(lim.attrib['lower']), float(lim.attrib['upper'])
                if not np.isfinite([lo, hi]).all() or lo > hi:
                    raise ValueError('Invalid joint limits: ' + name)
            mi = el.find('mimic')
            mimic = None if mi is None else (mi.attrib['joint'], float(mi.get('multiplier', 1)), float(mi.get('offset', 0)))
            if mimic and not np.isfinite(mimic[1:]).all():
                raise ValueError('Nonfinite mimic coefficients')
            self.joints[name] = Joint(name, kind, p, c, origin(el.find('origin')), axis, lo, hi, mimic)
            self.parent[c] = name
            self.children.setdefault(p, []).append(name)
        self.roots = sorted(set(self.links) - set(self.parent))
        for link in self.links:
            self.root_of(link)  # cycle detection, including components without roots
        for name in self.joints:
            self.joint_value(name, {})  # mimic cycle/missing-source validation
        if len(self.roots) > 1:
            self.diagnostics.append('Disconnected component roots: ' + ', '.join(self.roots))

    def root_of(self, link):
        seen = set()
        while link in self.parent:
            if link in seen:
                raise ValueError('Kinematic cycle: ' + link)
            seen.add(link)
            link = self.joints[self.parent[link]].parent
        return link

    def joint_value(self, name, values, seen=None):
        seen = set() if seen is None else set(seen)
        if name in seen or name not in self.joints:
            raise ValueError('Mimic cycle/missing joint: ' + name)
        seen.add(name)
        j = self.joints[name]
        q = self.joint_value(j.mimic[0], values, seen)*j.mimic[1]+j.mimic[2] if j.mimic else float(values.get(name, min(j.upper, max(j.lower, 0))))
        if not math.isfinite(q):
            raise ValueError('Nonfinite joint position: ' + name)
        if j.kind in ('revolute', 'prismatic') and not j.lower-1e-10 <= q <= j.upper+1e-10:
            raise ValueError('Joint outside limits: ' + name)
        return q

    def fk(self, root, values=None, origins=None):
        if root not in self.roots:
            raise ValueError('Preview anchor must be a component root')
        values, origins = values or {}, origins or {}
        poses, pending = {root: transform()}, [root]
        while pending:
            p = pending.pop()
            for name in self.children.get(p, []):
                j = self.joints[name]
                if j.kind not in ('fixed','revolute','continuous','prismatic'):
                    continue
                poses[j.child] = poses[p] @ validate_transform(origins.get(name, j.pose)) @ motion(j.kind, j.axis, self.joint_value(name, values))
                pending.append(j.child)
        return poses

    def chain(self, start, end):
        pending, seen = [(start, [start])], set()
        while pending:
            link, path = pending.pop(0)
            if link == end:
                return path
            if link in seen:
                continue
            seen.add(link)
            adjacent = [self.joints[n].child for n in self.children.get(link, [])]
            if link in self.parent:
                adjacent.append(self.joints[self.parent[link]].parent)
            pending.extend((n, path+[n]) for n in adjacent if n not in seen)
        raise ValueError('No actual URDF joint chain between %s and %s' % (start, end))


def load_model(source, variant, package_root, mappings=None):
    if source.startswith('param:'):
        import rospy
        xml = rospy.get_param(source[6:])
    elif source.endswith('.xacro'):
        import xacro
        xml = xacro.process_file(source, mappings=mappings or {}).toxml()
    else:
        xml = Path(source).read_text()
    return Model(xml, package_root, variant, source)
