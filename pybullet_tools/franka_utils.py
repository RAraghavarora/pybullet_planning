import numpy as np
from itertools import product

from .utils import create_box, set_base_values, set_point, set_pose, get_pose, \
    get_bodies, z_rotation, load_model, load_pybullet, HideOutput, create_body, assign_link_colors, \
    get_box_geometry, get_cylinder_geometry, create_shape_array, unit_pose, Pose, \
    Point, LockRenderer, FLOOR_URDF, TABLE_URDF, add_data_path, TAN, set_color, BASE_LINK, remove_body,\
    add_data_path, connect, dump_body, disconnect, wait_for_user, get_movable_joints, get_sample_fn, \
    set_joint_positions, get_joint_name, LockRenderer, link_from_name, get_link_pose, \
    multiply, Pose, Point, interpolate_poses, HideOutput, draw_pose, set_camera_pose, load_pybullet, \
    assign_link_colors, add_line, point_from_pose, remove_handles, BLUE, INF

from .pr2_utils import DRAKE_PR2_URDF

from .ikfast.utils import IKFastInfo
from .ikfast.ikfast import * # For legacy purposes

FE_GRIPPER_URDF = "models/franka_description/robots/hand.urdf"
#FRANKA_URDF = "models/franka_description/robots/panda_arm.urdf"
FRANKA_URDF = "models/franka_description/robots/panda_arm_hand.urdf"

PANDA_INFO = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0',
                        ee_link='panda_link8', free_joints=['panda_joint7'])

def create_franka():
    with LockRenderer():
        with HideOutput(True):
            robot = load_model(FRANKA_URDF, fixed_base=True)
            assign_link_colors(robot, max_colors=3, s=0.5, v=1.)
            # set_all_color(robot, GREEN)
    return robot

def create_fe_gripper():
    with LockRenderer():
        with HideOutput(True):
            robot = load_model(FE_GRIPPER_URDF, fixed_base=False)
            # assign_link_colors(robot, max_colors=3, s=0.5, v=1.)
            # set_all_color(robot, GREEN)
    return robot

#####################################################################

def test_retraction(robot, info, tool_link, distance=0.1, **kwargs):
    ik_joints = get_ik_joints(robot, info, tool_link)
    start_pose = get_link_pose(robot, tool_link)
    end_pose = multiply(start_pose, Pose(Point(z=-distance)))
    handles = [add_line(point_from_pose(start_pose), point_from_pose(end_pose), color=BLUE)]
    #handles.extend(draw_pose(start_pose))
    #hand®les.extend(draw_pose(end_pose))
    path = []
    pose_path = list(interpolate_poses(start_pose, end_pose, pos_step_size=0.01))
    for i, pose in enumerate(pose_path):
        print('Waypoint: {}/{}'.format(i+1, len(pose_path)))
        handles.extend(draw_pose(pose))
        conf = next(either_inverse_kinematics(robot, info, tool_link, pose, **kwargs), None)
        if conf is None:
            print('Failure!')
            path = None
            wait_for_user()
            break
        set_joint_positions(robot, ik_joints, conf)
        path.append(conf)
        wait_for_user()
        # for conf in islice(ikfast_inverse_kinematics(robot, info, tool_link, pose, max_attempts=INF, max_distance=0.5), 1):
        #    set_joint_positions(robot, joints[:len(conf)], conf)
        #    wait_for_user()
    remove_handles(handles)
    return path

def test_ik(robot, info, tool_link, tool_pose):
    draw_pose(tool_pose)
    # TODO: sort by one joint angle
    # TODO: prune based on proximity
    ik_joints = get_ik_joints(robot, info, tool_link)
    for conf in either_inverse_kinematics(robot, info, tool_link, tool_pose, use_pybullet=False,
                                          max_distance=INF, max_time=10, max_candidates=INF):
        # TODO: profile
        set_joint_positions(robot, ik_joints, conf)
        wait_for_user()

def sample_ik_tests(robot):
    joints = get_movable_joints(robot)
    tool_link = link_from_name(robot, 'panda_hand')

    info = PANDA_INFO
    check_ik_solver(info)

    sample_fn = get_sample_fn(robot, joints)
    for i in range(10):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(robot, joints, conf)
        wait_for_user()
        # test_ik(robot, info, tool_link, get_link_pose(robot, tool_link))
        test_retraction(robot, info, tool_link, use_pybullet=False,
                        max_distance=0.1, max_time=0.05, max_candidates=100)

######################################################

from pybullet_tools.utils import plan_joint_motion, create_flying_body, SE3, euler_from_quat, BodySaver, \
    intrinsic_euler_from_quat, quat_from_euler, wait_for_duration
from pybullet_tools.pr2_primitives import Trajectory, Commands, State

def pose_to_se3(p):
    # return list(p[0]) + list(euler_from_quat(p[1]))
    return np.concatenate([np.asarray(p[0]), np.asarray(euler_from_quat(p[1]))])

def se3_to_pose(conf):
    return (conf[:3], quat_from_euler(conf[3:]))

def plan_se3_motion(robot, pose1, pose2, obstacles=[], custom_limits={}):
    sub_robot = create_flying_body(SE3, robot, robot)
    joints = get_movable_joints(sub_robot)
    initial_conf = pose_to_se3(pose1)
    final_conf = pose_to_se3(pose2)
    set_joint_positions(sub_robot, joints, initial_conf)
    path = plan_joint_motion(sub_robot, joints, final_conf, obstacles=obstacles,
                             self_collisions=False, custom_limits=custom_limits)
    return path

def get_free_motion_gen(problem, custom_limits={}, collisions=True, visualize=False, teleport=False):
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    def fn(p1, p2, fluents=[]):
        from pybullet_tools.pr2_primitives import Pose
        saver.restore()
        p1.assign()
        raw_path = plan_se3_motion(robot, p1.value, p2.value, obstacles=obstacles, custom_limits=custom_limits)
        path = [Pose(robot, se3_to_pose(conf)) for conf in raw_path]
        if visualize:
            for p in path:
                p.assign()
                wait_for_duration(0.01)
            wait_for_user('finished')
            p1.assign()
        t = Trajectory(path)
        cmd = Commands(State(), savers=[BodySaver(robot)], commands=[t])
        return (cmd,)
    return fn