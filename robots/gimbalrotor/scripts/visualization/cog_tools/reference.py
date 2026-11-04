"""Independent, versioned mass records. Never derive a baseline on model reload."""
import copy
import math
from pathlib import Path
import numpy as np
import yaml
from .math3d import transform, point, vector, import_properties


def placement(data):
    return transform(data.get('xyz_m', [0, 0, 0]), data.get('rpy_rad', [0, 0, 0]))


class Registry:
    def __init__(self, data=None):
        self.data = copy.deepcopy(data or {'schema_version': 1, 'records': {}})
        if self.data.get('schema_version') != 1 or not isinstance(self.data.get('records'), dict):
            raise ValueError('Unsupported reference registry schema')
        if self.data.get('units', {'length':'m','mass':'kg','angle':'rad','inertia':'kg_m2'}) != {'length':'m','mass':'kg','angle':'rad','inertia':'kg_m2'}:
            raise ValueError('Registry fields must be normalized to SI; convert raw imports once')
        identities = set()
        instances = set()
        for key, r in self.data['records'].items():
            ident = (r['variant'], r['link'])
            if ident in identities:
                raise ValueError('Overlapping records for same variant/link: ' + key)
            identities.add(ident)
            instance = (r['variant'], r.get('instance_id', r['link']))
            if instance in instances:
                raise ValueError('Duplicate physical instance: '+str(instance))
            instances.add(instance)
            s = r['source']
            import_properties(s['mass_kg'], s['cog_in_cad_m'], s.get('inertia_at_cog_kg_m2'))
            if s.get('inertia_at_cog_kg_m2') is not None and s.get('off_diagonal_convention') != 'tensor_entries':
                raise ValueError('Specify tensor_entries sign convention before importing inertia')
            vector(s.get('inertia_axes_rpy_in_cad', [0, 0, 0]))
            if r.get('cad_to_mesh') is not None:
                placement(r['cad_to_mesh'])
            placement(r['calibrated_cad_to_link'])
            scale = vector(r['visual']['mesh_scale'])
            if np.any(scale <= 0):
                raise ValueError('Registered unit scale must be positive')

    @classmethod
    def read(cls, path):
        return cls(yaml.safe_load(Path(path).read_text()))

    def record(self, variant, link):
        found = [(key, r) for key, r in self.data['records'].items() if r['variant'] == variant and r['link'] == link]
        return found[0] if found else (None, None)

    def resolve(self, model, link_name, visual_poses=None):
        key, r = self.record(model.variant, link_name)
        if r is None:
            return {'diagnostics': ['Missing CAD reference for variant/link']}
        out = {'id': key, 'record': copy.deepcopy(r), 'diagnostics': []}
        link, vreg = model.links[link_name], r['visual']
        idx = int(vreg.get('index', 0))
        if 'name' in vreg:
            matches = [i for i, v in enumerate(link.visuals) if v.name == vreg['name']]
            idx = matches[0] if len(matches) == 1 else -1
        if not 0 <= idx < len(link.visuals):
            out['diagnostics'].append('Registered visual no longer resolves')
            return out
        v = link.visuals[idx]
        out['visual_index'] = idx
        if v.uri != vreg['mesh_uri'] or not np.allclose(v.size, vreg['mesh_scale'], rtol=0, atol=1e-12) or (vreg.get('mesh_sha256') and v.mesh_hash != vreg['mesh_sha256']):
            out['diagnostics'].append('Material mesh/scale/hash mismatch: reference mapping unavailable')
            return out
        if vreg.get('origin_fingerprint') and v.fingerprint != vreg['origin_fingerprint']:
            out['diagnostics'].append('Visual origin/fingerprint differs from registration; saved calibration retained; review index binding')
        tcal = placement(r['calibrated_cad_to_link'])
        tma = None if r.get('cad_to_mesh') is None else placement(r['cad_to_mesh'])
        current = (visual_poses or {}).get((link_name, idx), v.pose)
        out['cad_pose'] = tcal
        if tma is not None:
            out['ghost_pose'] = tcal @ np.linalg.inv(tma)
            delta = np.linalg.inv(out['ghost_pose']) @ current
            out['mesh_placement_delta_m'] = float(np.linalg.norm(delta[:3, 3]))
            out['mesh_placement_delta_rad'] = float(math.acos(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1)))
            if not np.allclose(current, out['ghost_pose'], atol=1e-9):
                out['diagnostics'].append('Current mesh differs from saved physical placement')
        s = r['source']
        mass, cog, tensor = import_properties(s['mass_kg'], s['cog_in_cad_m'], s.get('inertia_at_cog_kg_m2'))
        # A direct, independently calibrated CAD→link placement does not require
        # knowing how CAD coordinates relate to the visual's STL export frame.
        verified = all(d.get('status') == 'verified' for d in (s, r['calibrated_cad_to_link']))
        common = {'mass_kg': mass, 'record_id': key, 'provenance': copy.deepcopy(s), 'verified': verified,
                  'composition_status': r.get('composition_status', 'unverified'),
                  'constituents': r.get('constituents', []), 'overlaps_with': r.get('overlaps_with', [])}
        out['reference'] = dict(common, cog_local_m=point(tcal, cog).tolist(), label='CAD reference — verified source/calibration' if verified else 'Reference — assumptions remain')
        if tensor is not None:
            rot = tcal[:3, :3] @ transform(rpy=s.get('inertia_axes_rpy_in_cad', [0,0,0]))[:3, :3]
            out['reference']['tensor_at_cog_link_kg_m2'] = (rot @ tensor @ rot.T).tolist()
        if tma is None:
            out['diagnostics'].append('CAD/mesh frame relationship missing: no candidate or reference ghost')
        elif not r.get('allow_follow_current_visual_candidate'):
            out['diagnostics'].append('Mesh-follow candidate disabled in record')
        elif len(link.visuals) != 1 or not vreg.get('covers_entire_mass_record'):
            out['diagnostics'].append('Ambiguous partial/multiple visuals: no whole-body candidate')
        else:
            out['candidate'] = dict(common, verified=False, cog_local_m=point(current @ tma, cog).tolist(), label='Current mesh candidate — physical placement assumed')
        return out
