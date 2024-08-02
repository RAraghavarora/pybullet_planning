import numpy as np
from itertools import product

from .pr2_utils import set_arm_conf, REST_LEFT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf, PR2_URDF, DRAKE_PR2_URDF, create_gripper
from .utils import create_box, set_base_values, set_point, set_pose, get_pose, \
    get_bodies, z_rotation, load_model, load_pybullet, HideOutput, create_body, \
    get_box_geometry, get_cylinder_geometry, create_shape_array, unit_pose, Pose, \
    Point, LockRenderer, FLOOR_URDF, TABLE_URDF, add_data_path, TAN, set_color, BASE_LINK, remove_body

from world_builder.entities import Object, Location, Floor, Supporter, Surface, Movable, \
    Space, Steerable, Door


LIGHT_GREY = (0.7, 0.7, 0.7, 1.)

class Problem(object):
    def __init__(self, robot, arms=tuple(), movable=tuple(), grasp_types=tuple(),
                 surfaces=tuple(), sinks=tuple(), stoves=tuple(), buttons=tuple(),
                 goal_conf=None, goal_holding=tuple(), goal_on=tuple(),
                 goal_cleaned=tuple(), goal_cooked=tuple(), costs=False,
                 body_names={}, body_types=[], base_limits=None):
        self.robot = robot
        self.arms = arms
        self.movable = movable
        self.grasp_types = grasp_types
        self.surfaces = surfaces
        self.sinks = sinks
        self.stoves = stoves
        self.buttons = buttons
        self.goal_conf = goal_conf
        self.goal_holding = goal_holding
        self.goal_on = goal_on
        self.goal_cleaned = goal_cleaned
        self.goal_cooked = goal_cooked
        self.costs = costs
        self.body_names = body_names
        self.body_types = body_types
        self.base_limits = base_limits
        all_movable = [self.robot] + list(self.movable)
        self.fixed = list(filter(lambda b: b not in all_movable, get_bodies()))
        self.gripper = None
    def get_gripper(self, arm='left', visual=True):
        # upper = get_max_limit(problem.robot, get_gripper_joints(problem.robot, 'left')[0])
        # set_configuration(gripper, [0]*4)
        # dump_body(gripper)
        if self.gripper is None:
            self.gripper = create_gripper(self.robot, arm=arm, visual=visual)
        return self.gripper
    def remove_gripper(self):
        if self.gripper is not None:
            remove_body(self.gripper)
            self.gripper = None
    def __repr__(self):
        return repr(self.__dict__)

#######################################################

def get_fixed_bodies(problem): # TODO: move to problem?
    return problem.fixed

def create_pr2(use_drake=True, fixed_base=True, torso=0.2):
    pr2_path = DRAKE_PR2_URDF if use_drake else PR2_URDF
    with LockRenderer():
        with HideOutput():
            pr2 = load_model(pr2_path, fixed_base=fixed_base)
        set_group_conf(pr2, 'torso', [torso])
    return pr2

def create_floor(**kwargs):
    add_data_path()
    return load_pybullet(FLOOR_URDF, **kwargs)

def create_table(width=0.6, length=1.2, height=0.73, thickness=0.03, radius=0.015,
                 top_color=LIGHT_GREY, leg_color=TAN, cylinder=True, **kwargs):
    # TODO: table URDF
    surface = get_box_geometry(width, length, thickness)
    surface_pose = Pose(Point(z=height - thickness/2.))

    leg_height = height-thickness
    if cylinder:
        leg_geometry = get_cylinder_geometry(radius, leg_height)
    else:
        leg_geometry = get_box_geometry(width=2*radius, length=2*radius, height=leg_height)
    legs = [leg_geometry for _ in range(4)]
    leg_center = np.array([width, length])/2. - radius*np.ones(2)
    leg_xys = [np.multiply(leg_center, np.array(signs))
               for signs in product([-1, +1], repeat=len(leg_center))]
    leg_poses = [Pose(point=[x, y, leg_height/2.]) for x, y in leg_xys]

    geoms = [surface] + legs
    poses = [surface_pose] + leg_poses
    colors = [top_color] + len(legs)*[leg_color]

    collision_id, visual_id = create_shape_array(geoms, poses, colors)
    body = create_body(collision_id, visual_id, **kwargs)

    # TODO: unable to use several colors
    #for idx, color in enumerate(geoms):
    #    set_color(body, shape_index=idx, color=color)
    return body

def create_door():
    return load_pybullet("data/door.urdf")

#######################################################

# https://github.com/bulletphysics/bullet3/search?l=XML&q=.urdf&type=&utf8=%E2%9C%93

TABLE_MAX_Z = 0.6265 # TODO: the table legs don't seem to be included for collisions?

def holding_problem(arm='left', grasp_type='side'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_base_values(pr2, (0, -2, 0))
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    plane = create_floor()
    table = load_pybullet(TABLE_URDF)
    #table = load_pybullet("table_square/table_square.urdf")
    box = create_box(.07, .05, .15)
    set_point(box, (0, 0, TABLE_MAX_Z + .15/2))

    return Problem(robot=pr2, movable=[box], arms=[arm], grasp_types=[grasp_type], surfaces=[table],
                   goal_conf=get_pose(pr2), goal_holding=[(arm, box)])

def stacking_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_base_values(pr2, (0, -2, 0))
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    plane = create_floor()
    table1 = load_pybullet(TABLE_URDF)
    #table = load_pybullet("table_square/table_square.urdf")

    block = create_box(.07, .05, .15)
    set_point(block, (0, 0, TABLE_MAX_Z + .15/2))

    table2 = load_pybullet(TABLE_URDF)
    set_base_values(table2, (2, 0, 0))

    return Problem(robot=pr2, movable=[block], arms=[arm],
                   grasp_types=[grasp_type], surfaces=[table1, table2],
                   #goal_on=[(block, table1)])
                   goal_on=[(block, table2)])

#######################################################

from world_builder.world import World
def create_world(args):
    return World(time_step=args.time_step, segment=args.segment, use_rel_pose=args.use_rel_pose)

import numpy as np
from robot_builder.robots import PR2Robot
from pybullet_planning.robot_builder.robot_builders import set_pr2_ready
from pybullet_tools.pr2_utils import set_group_conf
from pybullet_tools.bullet_utils import BASE_LINK, BASE_RESOLUTIONS, draw_base_limits as draw_base_limits_bb, CAMERA_FRAME, CAMERA_MATRIX, EYE_FRAME, BASE_LIMITS
import numpy as np
from pybullet_tools.pr2_primitives import get_base_custom_limits
from world_builder.entities import Camera
def create_pr2_robot(robot=None, world=None, base_q=(0, 0, 0), dual_arm=False, resolutions=BASE_RESOLUTIONS, use_torso=True, custom_limits=BASE_LIMITS):
    # copied from pybullet_planning
    # robot = None
    # if robot is None:
    #     robot = create_pr2(use_drake=USE_DRAKE_PR2)
    #     set_pr2_ready(robot, arm=PR2Robot.arms[0], dual_arm=dual_arm)
    #     if len(base_q) == 3:
    #         set_group_conf(robot, 'base', base_q)
    #     elif len(base_q) == 4:
    #         set_group_conf(robot, 'base-torso', base_q)
    #     set_pose(robot, robot_pose)
    #     set_configuration(robot, robot_conf)
    with np.errstate(divide='ignore'):
        weights = np.reciprocal(resolutions)

    if isinstance(custom_limits, dict):
        custom_limits = np.asarray(list(custom_limits.values())).T.tolist()

    # if draw_base_limits:
    #     draw_base_limits_bb(custom_limits)

    robot = PR2Robot(robot, base_link=BASE_LINK,
                     dual_arm=dual_arm, use_torso=use_torso,
                     custom_limits=get_base_custom_limits(robot, custom_limits),
                     resolutions=resolutions, weights=weights)
    if world is not None:
        world.add_robot(robot)
    # print('initial base conf', get_group_conf(robot, 'base'))
    # set_camera_target_robot(robot, FRONT=True)

    camera = Camera(robot, camera_frame=CAMERA_FRAME, camera_matrix=CAMERA_MATRIX, max_depth=2.5, draw_frame=EYE_FRAME)
    robot.cameras.append(camera)

    ## don't show depth and segmentation data yet
    # if args.camera: robot.cameras[-1].get_image(segment=args.segment)

    return robot

def create_kitchen(w=.5, h=.7, robot=None):
    use_world = True
    robot_builder_args = {
        'robot_name': 'pr2', 
        'draw_base_limits': True, 
        'dual_arm': False, 
        'self_collisions': False, 
        'create_robot_fn': None, 
        'custom_limits': ((0.7, -2, 0), (3, 10, 3)), 
        'initial_xy': (2, 4)
        }
    from robot_builder.robot_builders import build_robot_from_args
    args = {
        'time_step': 0.05,
        'segment': False,
        'use_rel_pose': False
    }
    from argparse import Namespace
    args = Namespace(**args)
    world = create_world(args)
    # robot2 = build_robot_from_args(world, **robot_builder_args)
    custom_limits = ((0.7, -2, 0), (3, 10, 3))
    robot_instance = create_pr2_robot(robot=robot, world=world, custom_limits=custom_limits)
    from .rag_loaders_partnet_kitchen import sample_full_kitchen, load_counter_movables
    # sample_full_kitchen(world)
    floor = create_floor()
    floor_instance = world.add_object(
        Floor(floor),
        )
    table = create_box(w, w, h, color=(.75, .75, .75, 1))
    table_instance = world.add_object(Object(table), Pose(point = Point(2,0,h/2)))
    
    # set_point(table, (2, 0, h/2))

    mass = 1
    #mass = 0.01
    #mass = 1e-6
    cabbage = create_box(.07, .07, .1, mass=mass, color=(0, 1, 0, 1))
    set_point(cabbage, (2, 0, h + .1/2))
    cabbage_instance = world.add_object(Movable(cabbage), Pose(point=Point(2,0,h+.1/2)))
    from .rag_utils import load_asset
    # cabbage, _, _ = load_asset('food', floor=floor, random_instance=True) #R Adding real cabbage leads to AssertionError?
    # cabbage_instance = world.add_object(Movable(cabbage), Pose(point=Point(2, 0.1, h+.1/2)))
    # cabbage = load_model(BLOCK_URDF, fixed_base=False)

    # sink = create_box(w, w, h, color=(.25, .25, .75, 1))
    # set_point(sink, (0, 2, h/2))

    stove = create_box(w, w, h, color=(.75, .25, .25, 1))
    set_point(stove, (0, -2, h/2))
    stove_instance = world.add_object(Object(stove), Pose(point=Point(0,-2,h/2)))
    # Added by Raghav
    import math
    from pybullet_planning.pybullet_tools.utils import wait_for_user
    from examples.pybullet.utils.pybullet_tools.utils import Euler, get_aabb, get_aabb_extent

    if use_world:
        from .rag_loaders_partnet_kitchen import load_all_furniture, load_storage_spaces
        ordering = ['SinkBase', 'Minifridge']
        base = table_instance
        on_base=['Minifridge']
        # under_counter=['MiniFridge']
        under_counter=[]
        full_body=[]
        tall_body=[]
        start=0
        op = load_all_furniture(world, ordering, floor_instance, base, start, under_counter, on_base, full_body, tall_body)
        load_storage_spaces(world, epsilon=0, make_doors_transparent=False)
        minifridge_instance = world.name_to_object('minifridge')
        minifridge = minifridge_instance.body
    else:
        minifridge, mf_file, scale = load_asset('minifridge', x=2, y=0, yaw=1.5*math.pi, floor=floor,
                    random_instance=False, verbose=True)
    pose = Pose(point=Point(x=0, y=2, z=h/2), euler=Euler(yaw=math.pi))
    set_pose(minifridge, pose)

    minifridge_storage = world.name_to_object("minifridge::storage")
    # minifridge_storage.place_obj(cabbage_instance)
    counters = {
        'food': [table_instance, minifridge_instance, stove_instance]
    }
    load_counter_movables(world, counters, d_x_min=0.3, obstacles=[], reachability_check=False)

    ################## Add joints and storage space #########################
    from pybullet_planning.world_builder.world_utils import get_partnet_semantics, get_partnet_spaces, get_partnet_doors
    space = None
    ## --- ADD EACH DOOR JOINT
    # doors = get_partnet_doors(mf_file, minifridge)
    # for b, j in doors:
        # world.add_joint_object(b, j, 'door')
        # obj.doors.append((b, j))
        
    ## --- ADD ONE SPACE TO BE PUT INTO
    # spaces = get_partnet_spaces(mf_file, minifridge)
    # for b, _, l in spaces:
        # space = world.add_object(Space(b, l, name=f'storage'))  ## f'{obj.category}::storage'
        # break
    # return doors, space



    # ly = get_aabb_extent(get_aabb(minifridge))[1]
    # minifridge_base = load_asset('MiniFridgeBase', l=ly, yaw=math.pi, floor=floor,
    #                    random_instance=True, verbose=False)
    # minifridge_region = create_box(.07, .07, .07, mass=mass, color=(0, 1, 0, 1))
    # set_point(minifridge_region, (-1, 6, h/2 + .1/2))
    sink = minifridge
    if use_world:
        return table, cabbage, sink, stove, world
    else:
        return table, cabbage, sink, stove

#######################################################

def cleaning_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    #door = create_door()
    #set_point(door, (2, 0, 0))

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   goal_cleaned=[cabbage])

def cooking_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   goal_cooked=[cabbage])

def cleaning_button_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    d = 0.1
    sink_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(sink_button, ((0, 2-(.5+d)/2, .7-d/2), z_rotation(np.pi/2)))

    stove_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(stove_button, ((0, -2+(.5+d)/2, .7-d/2), z_rotation(-np.pi/2)))

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   buttons=[(sink_button, sink), (stove_button, stove)],
                   goal_conf=get_pose(pr2), goal_holding=[(arm, cabbage)], goal_cleaned=[cabbage])

def cooking_button_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    d = 0.1
    sink_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(sink_button, ((0, 2-(.5+d)/2, .7-d/2), z_rotation(np.pi/2)))

    stove_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(stove_button, ((0, -2+(.5+d)/2, .7-d/2), z_rotation(-np.pi/2)))

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   buttons=[(sink_button, sink), (stove_button, stove)],
                   goal_conf=get_pose(pr2), goal_holding=[(arm, cabbage)], goal_cooked=[cabbage])

PROBLEMS = [
    holding_problem,
    stacking_problem,
    cleaning_problem,
    cooking_problem,
    cleaning_button_problem,
    cooking_button_problem,
]
