import * as THREE from 'three';

// User-requested presentation lift. Never used for FK/telemetry or F-GNG input.
export const ROBOT_DISPLAY_OFFSET = Object.freeze([0, 0, .020]);

export function createRobotDisplay(tree, feedbackMarker) {
  const display = new THREE.Group();
  display.name = 'robot_visual_offset';
  display.position.fromArray(ROBOT_DISPLAY_OFFSET);
  display.add(tree.root, feedbackMarker);
  display.updateMatrixWorld(true);
  return display;
}

export function displayPositionToBase(position) {
  return position.clone().sub(new THREE.Vector3(...ROBOT_DISPLAY_OFFSET));
}

// URDF fixed-axis RPY: Rz(yaw) * Ry(pitch) * Rx(roll).
export function urdfQuaternion(rpy) {
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(...rpy, 'ZYX'));
}

export function buildTree(model) {
  const links = Object.fromEntries(model.links.map(l => [l.name, new THREE.Group()]));
  const moving = [];
  for (const j of model.joints) {
    const frame = new THREE.Group();
    frame.position.fromArray(j.origin.xyz);
    frame.quaternion.copy(urdfQuaternion(j.origin.rpy));
    links[j.parent].add(frame);
    frame.add(links[j.child]);
    if (j.type !== 'fixed') moving.push({...j, node: links[j.child], axisVector: new THREE.Vector3(...j.axis).normalize()});
  }
  for (const [name, group] of Object.entries(links)) group.name = name;
  return {links, moving, root: links[model.root]};
}

export function feedbackValues(data) {
  const values = {};
  if (data?.all_joint_feedback_fresh && data.joint_angles_rad?.length === 6 && data.joint_angles_rad.every(Number.isFinite)) {
    data.joint_angles_rad.forEach((q, i) => values['joint'+(i+1)] = q);
  }
  // SDK width is the full jaw opening, in metres. URDF joints describe each jaw.
  if (data?.gripper?.mode === 'width' && Number.isFinite(data.gripper.value)) {
    values.joint7 = data.gripper.value / 2;
    values.joint8 = -data.gripper.value / 2;
  }
  return values;
}

export function applyFeedback(tree, values) {
  for (const j of tree.moving) {
    const value = values[j.name];
    if (!Number.isFinite(value)) continue;
    // Never clamp measured angles to nominal URDF limits: that would falsify pose.
    if (j.type === 'prismatic') j.node.position.copy(j.axisVector).multiplyScalar(value);
    else j.node.quaternion.setFromAxisAngle(j.axisVector, value);
  }
  tree.root.updateMatrixWorld(true);
}

export function feedbackUsable(packet) {
  return !!(packet?.data?.all_joint_feedback_fresh && (packet.live || packet.snapshot)
    && packet.data.joint_angles_rad?.length === 6 && packet.data.joint_angles_rad.every(Number.isFinite));
}
