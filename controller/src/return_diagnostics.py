"""CPU measurements shared by isolated return diagnostics and the live guard."""
import numpy as np
from collision_world import pose_matrix
from astra_planner import so3_log


def insertion_measurement(model, arm, desired_p, desired_r, desired_q, actual_ee,
                          state, target, relative, goal, contacts, controls=None):
    actual = pose_matrix(actual_ee)
    robot = state['robots'][arm]
    fk = model.evaluate(arm, robot['q'][:7])
    payload = pose_matrix(state['objects'][str(target)]['pose'])
    measured_relative = np.linalg.inv(actual) @ payload
    desired = np.eye(4); desired[:3, 3] = desired_p; desired[:3, :3] = desired_r
    def delta(a, b):
        return dict(position_error_m=float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                    rotation_error_rad=float(np.linalg.norm(so3_log(a[:3, :3]@b[:3, :3].T))))
    fk_pose = np.eye(4); fk_pose[:3, 3] = fk['ee_position_m']; fk_pose[:3, :3] = fk['ee_rotation_world']
    return dict(**delta(actual, desired), desired_ee=desired.tolist(), actual_ee=actual.tolist(),
        desired_q=list(desired_q), measured_q=robot['q'], measured_qdot=robot['qdot'],
        joint_error_rad=(np.asarray(robot['q'][:7])-desired_q).tolist(), controls=controls,
        fk_vs_measured=delta(fk_pose, actual), payload_pose=state['objects'][str(target)]['pose'],
        payload_goal_pose=goal, payload_vs_goal=delta(payload, pose_matrix(goal)),
        payload_vs_waypoint=delta(payload, desired@relative),
        planned_ee_to_payload=np.asarray(relative).tolist(), measured_ee_to_payload=measured_relative.tolist(),
        grasp_transform_drift=delta(measured_relative, np.asarray(relative)), contacts=contacts)


def tracking_ok(measure):
    return measure['position_error_m'] <= .005 and measure['rotation_error_rad'] <= .04
