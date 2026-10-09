"""Parse the official, firmware-matched Piper URDF for the local web viewer."""
from pathlib import Path
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET

from app_paths import APP_ROOT, LEGACY_ROOT

# ROOT remains the shared-data base for existing mapping and CAD tooling.
ROOT = LEGACY_ROOT
DESCRIPTION = ROOT / 'piper_model_source/src/piper_description'
SOURCE_COMMIT = 'ac41fcbcdda598f01b51cf6175ed9a24d0dacadc'

def vector(text, default):
    return [float(x) for x in text.split()] if text else list(default)

def origin(node):
    return {'xyz': vector(node.get('xyz'), [0, 0, 0]),
            'rpy': vector(node.get('rpy'), [0, 0, 0])} if node is not None else {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}

def flipped_bottom_origin(original, flip):
    """Post-compose T(part) with a half-turn about its upright mounting axis.

    The STEP part origin is offset from the foot. Rotate about the actual foot
    centre, not the world origin, so the attachment stays in place.
    """
    if not flip or not flip['enabled']:
        return original
    if flip['axis_in_part_frame'] != '+X' or flip['angle_degrees'] != 180:
        raise ValueError('Only the user-approved lower-mount half-turn is supported')
    pivot=flip['pivot_in_part_frame_m']
    if len(pivot)!=3 or any(not math.isfinite(v) for v in pivot):
        raise ValueError('Invalid lower-mount pivot')
    roll,pitch,yaw=original['rpy']
    # p - Rx(pi)*p = [0, 2*py, 2*pz], then map through the existing part rotation.
    dy,dz=2*pivot[1],2*pivot[2]
    y=math.cos(roll)*dy-math.sin(roll)*dz
    z=math.sin(roll)*dy+math.cos(roll)*dz
    x=math.sin(pitch)*z
    z=math.cos(pitch)*z
    delta=[math.cos(yaw)*x-math.sin(yaw)*y,
           math.sin(yaw)*x+math.cos(yaw)*y,z]
    return {'xyz':[v+d for v,d in zip(original['xyz'],delta)],
            'rpy':[math.remainder(roll+math.pi,2*math.pi),pitch,yaw]}

def add_wrist_camera(links, joints, assets):
    directory = APP_ROOT / 'assets/wrist_cad'
    if not (directory/'attachment.json').is_file():
        return None
    config = json.loads((directory/'attachment.json').read_text())
    if not config['enabled']:
        return None
    if config['parent_link'] not in {link['name'] for link in links}:
        raise ValueError('Invalid CAD parent link')
    if len(config['xyz_m']) != 3 or len(config['rpy_rad']) != 3 or any(not math.isfinite(x) for x in config['xyz_m'] + config['rpy_rad']):
        raise ValueError('Invalid CAD attachment transform')
    manifest = json.loads((directory/'manifest.json').read_text())
    flip_file=directory/'bottom_mount_flip.json'
    flip=json.loads(flip_file.read_text()) if flip_file.is_file() else None
    if config['source_sha256'] != manifest['source_sha256']:
        raise ValueError('CAD source changed; attachment registration must be repeated')
    fit_file = directory/'fitted_mount.json'
    fit = json.loads(fit_file.read_text()) if fit_file.is_file() else None
    fit = fit if fit and fit.get('enabled') else None
    anchor_origin = {'xyz':config['xyz_m'],'rpy':config['rpy_rad']}
    if fit:
        if fit.get('viewer_only') is not True or fit['source_sha256'] != manifest['source_sha256']:
            raise ValueError('Invalid viewer-only mount adaptation')
        for transform in [fit['anchor_origin'], fit['bottom_origin']]:
            if any(len(transform[k]) != 3 for k in ('xyz','rpy')) or any(
                    not math.isfinite(x) for x in transform['xyz']+transform['rpy']):
                raise ValueError('Invalid fitted mount transform')
        anchor_origin = fit['anchor_origin']
    anchor = 'd405_mount_frame'
    links.append({'name': anchor, 'visuals': [], 'accessory': True})
    joints.append({'name':'gripper_to_d405_mount','type':'fixed',
        'parent':config['parent_link'],'child':anchor,
        'origin':anchor_origin,'axis':[0,0,1],'limit':None})
    names = {'realsense_d405_mounting_bot':'d405_mount_bottom',
             'realsense_d405_mounting_top':'d405_mount_top','RealSense_D405':'d405_camera_body'}
    for part in manifest['parts']:
        is_bottom = part['name']=='realsense_d405_mounting_bot'
        filename = fit['bottom_file'] if fit and is_bottom else part['file']
        path = (directory/filename).resolve()
        if not path.is_relative_to(directory.resolve()) or not path.is_file():
            raise ValueError('Missing or invalid CAD mesh')
        if fit:
            expected = fit['derived_mesh_sha256'] if is_bottom else fit['original_mesh_sha256'][part['file']]
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError('Fitted assembly asset changed; regenerate and validate fit')
        url = '/mesh/cad_' + path.name
        assets[url] = path
        name = names[part['name']]
        part_origin=flipped_bottom_origin(part['origin'],flip) if part['name']=='realsense_d405_mounting_bot' else part['origin']
        if fit and is_bottom:
            part_origin = fit['bottom_origin']
        color = [.08,.09,.10,1] if 'mount' in name else [.88,.89,.92,1]
        links.append({'name':name,'accessory':True,'visuals':[{'origin':{'xyz':[0,0,0],'rpy':[0,0,0]},
            'url':url,'scale':[1,1,1],'color':color}]})
        joints.append({'name':name+'_fixed','type':'fixed','parent':anchor,'child':name,
            'origin':part_origin,'axis':[0,0,1],'limit':None})
    return {**config, 'source':manifest['source'], 'source_sha256':manifest['source_sha256'],
            'bottom_mount_flip':flip,
            'viewer_fit':fit, 'active_anchor_origin':anchor_origin,
            'frame_note':'CAD body frames only. No calibrated optical frame is defined.'}

def load_model(firmware=None, include_camera=True):
    version = re.search(r'S-V(\d+)\.(\d+)-(\d+)', firmware or '')
    known = version is not None
    old = not known or tuple(map(int, version.groups())) < (1, 6, 3)
    filename = 'piper_description_old.urdf' if old else 'piper_description.urdf'
    root = ET.parse(DESCRIPTION / 'urdf' / filename).getroot()
    links, joints, assets = [], [], {}
    for link in root.findall('link'):
        visuals = []
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Unsupported geometry in official description')
            uri = mesh.get('filename', '')
            prefix = 'package://piper_description/'
            if not uri.startswith(prefix):
                raise ValueError('Unsupported mesh URI')
            path = (DESCRIPTION / uri[len(prefix):]).resolve()
            if not path.is_relative_to(DESCRIPTION.resolve()) or not path.is_file():
                raise ValueError('Mesh outside description or missing')
            assets['/mesh/' + path.name] = path
            color = visual.find('material/color')
            visuals.append({'origin': origin(visual.find('origin')),
                'url': '/mesh/' + path.name,
                'scale': vector(mesh.get('scale'), [1, 1, 1]),
                'color': vector(color.get('rgba') if color is not None else None, [.79, .82, .93, 1])})
        links.append({'name': link.get('name'), 'visuals': visuals})
    for joint in root.findall('joint'):
        axis, limit = joint.find('axis'), joint.find('limit')
        joints.append({'name': joint.get('name'), 'type': joint.get('type'),
            'parent': joint.find('parent').get('link'), 'child': joint.find('child').get('link'),
            'origin': origin(joint.find('origin')),
            'axis': vector(axis.get('xyz') if axis is not None else None, [0, 0, 1]),
            'limit': {k: float(limit.get(k)) for k in ('lower', 'upper')} if limit is not None else None})
    accessory = add_wrist_camera(links, joints, assets) if include_camera else None
    children = {j['child'] for j in joints}
    roots = [link['name'] for link in links if link['name'] not in children]
    if len(roots) != 1:
        raise ValueError('Description is not a single tree')
    return {'name': 'AgileX Piper', 'root': roots[0], 'links': links, 'joints': joints,
            'urdf': filename, 'firmware_match_known': known, 'source_commit': SOURCE_COMMIT,
            'wrist_camera': accessory,
            'source_url': 'https://github.com/agilexrobotics/piper_ros'}, assets
