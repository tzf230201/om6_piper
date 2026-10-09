"""Read-only URDF FK and explicit world-frame point-cloud offset."""
import numpy as np
from scipy.spatial.transform import Rotation

def transform(origin):
    result = np.eye(4)
    result[:3,:3] = Rotation.from_euler('xyz', origin['rpy']).as_matrix()
    result[:3,3] = origin['xyz']
    if not np.isfinite(result).all():
        raise ValueError('Nonfinite camera transform')
    return result

def camera_world_pose(model, angles, config):
    angles = np.asarray(angles, dtype=float)
    if angles.shape != (6,) or not np.isfinite(angles).all():
        raise ValueError('Six measured joint angles required')
    if config['world_frame'] != model['root']:
        raise ValueError('Only the URDF root is available as world frame')
    values = dict(zip(['joint'+str(i) for i in range(1,7)], angles))
    poses = {model['root']: np.eye(4)}
    for joint in model['joints']:
        motion = np.eye(4)
        if joint['type'] == 'revolute':
            axis = np.asarray(joint['axis'], float)
            motion[:3,:3] = Rotation.from_rotvec(axis/np.linalg.norm(axis)*values[joint['name']]).as_matrix()
        poses[joint['child']] = poses[joint['parent']] @ transform(joint['origin']) @ motion
    body = poses['d405_camera_body']
    optical = body @ transform(config['body_from_depth_optical'])
    offset = np.asarray(config['world_offset_m'], float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('Invalid world offset')
    # Left multiplication by Trans_world(offset), not a camera-local offset.
    optical[:3,3] += offset
    return optical

def usable_pose(packet, max_age=.15):
    data = packet.get('data') or {}
    ages = data.get('rx_age_s', {})
    age = packet.get('sample_age_s', float('inf')) + max(
        ages.get(k, float('inf')) for k in ('0x2A5','0x2A6','0x2A7'))
    q = np.asarray(data.get('joint_angles_rad', []), float)
    return bool(packet.get('live') and data.get('all_joint_feedback_fresh') and
                age < max_age and q.shape == (6,) and np.isfinite(q).all())
