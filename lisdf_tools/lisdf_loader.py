import os
import sys
from os.path import join, abspath, dirname, isdir, isfile
sys.path.append('lisdf')
from lisdf.parsing.sdf_j import load_sdf
from lisdf.components.model import URDFInclude

import warnings
warnings.filterwarnings('ignore')

from pybullet_planning.pybullet_tools.pr2_problems import create_floor
from pybullet_planning.pybullet_tools.pr2_utils import set_group_conf, get_group_joints
from pybullet_planning.pybullet_tools.utils import load_pybullet, connect, wait_if_gui, HideOutput, \
    disconnect, set_pose, set_joint_position, joint_from_name, quat_from_euler, draw_pose, unit_pose, \
    set_camera_pose, set_camera_pose2, get_pose, get_joint_position, get_link_pose, get_link_name, \
    set_joint_positions
from pybullet_planning.pybullet_tools.bullet_utils import nice

ASSET_PATH = join(dirname(__file__), '..', '..', 'assets')

class World():
    def __init__(self, lisdf):
        self.lisdf = lisdf
        self.body_to_name = {}
        self.name_to_body = {}

    @property
    def robot(self):
        return self.name_to_body['pr2']

    def summarize_all_objects(self):
        from pybullet_tools.logging import myprint as print

        print('----------------')
        print(f'PART I: world objects |')
        for body, name in self.body_to_name.items():

            line = f'{body}\t  |  {name}'
            if isinstance(body, tuple) and len(body) == 2:
                body, joint = body
                pose = get_joint_position(body, joint)
            elif isinstance(body, tuple) and len(body) == 3:
                body, _, link = body
                pose = get_link_pose(body, link)
            else:
                pose = get_pose(body)
            print(f"{line}\t|  Pose: {nice(pose)}")
        print('----------------')

def load_lisdf_pybullet(lisdf_path, verbose=True):
    # scenes_path = dirname(os.path.abspath(lisdf_path))
    tmp_path = join(ASSET_PATH, 'tmp')

    connect(use_gui=True, shadows=False, width=1980, height=1238)
    draw_pose(unit_pose(), length=1.)
    create_floor()

    # with HideOutput():
        # load_pybullet(join('models', 'Basin', '102379', 'mobility.urdf'))
    # load_pybullet(sdf_path)  ## failed
    # load_pybullet(join(tmp_path, 'table#1_1.sdf'))

    world = load_sdf(lisdf_path).worlds[0]
    bullet_world = World(world)

    # if world.name in HACK_CAMERA_POSES:
    #     cp, tp = HACK_CAMERA_POSES[world.name]
    #     set_camera_pose(camera_point=cp, target_point=tp)

    if world.gui != None:
        camera_pose = world.gui.camera.pose
        set_camera_pose2((camera_pose.pos, camera_pose.quat_xyzw))

    ## may be changes in joint positions
    model_states = {}
    if len(world.states) > 0:
        model_states = world.states[0].model_states
        model_states = {s.name: s for s in model_states}

    for model in world.models:
        scale = 1
        if isinstance(model, URDFInclude):
            uri = join(ASSET_PATH, 'scenes', model.uri)
            scale = model.scale_1d
        else:
            uri = join(tmp_path, f'{model.name}.sdf')
            with open(uri, 'w') as f:
                f.write(make_sdf_world(model.to_sdf()))

        with HideOutput():
            if verbose: print(f'..... loading {model.name} from {uri}', end="\r")
            body = load_pybullet(uri, scale=scale)
            if isinstance(body, tuple): body = body[0]
            bullet_world.body_to_name[body] = model.name
            bullet_world.name_to_body[model.name] = body

        ## set pose of body using PyBullet tools' data structure
        if model.name == 'pr2':
            pose = model.pose.pos
            set_group_conf(body, 'base', pose)
        else:
            pose = (model.pose.pos, quat_from_euler(model.pose.rpy))
            set_pose(body, pose)

        if model.name in model_states:
            for js in model_states[model.name].joint_states:
                position = js.axis_states[0].value
                set_joint_position(body, joint_from_name(body, js.name), position)

        if not isinstance(model, URDFInclude):
            os.remove(uri)

        # wait_if_gui('load next model?')
    return bullet_world

## will be replaced later will <world><gui><camera> tag
HACK_CAMERA_POSES = { ## scene_name : (camera_point, target_point)
    'kitchen_counter': ([3, 8, 3], [0, 8, 1]),
    'kitchen_basics': ([3, 6, 3], [0, 6, 1])
}

def make_sdf_world(sdf_model):
    return f"""<?xml version="1.0" ?>
<!-- tmp sdf file generated from LISDF -->
<sdf version="1.9">
  <world name="tmp_world">

{sdf_model}

  </world>
</sdf>"""


if __name__ == "__main__":

    for lisdf_test in ['kitchen_lunch']: ## 'm0m_joint_test', 'kitchen_basics', 'kitchen_counter'
        lisdf_path = join(ASSET_PATH, 'scenes', f'{lisdf_test}.lisdf')
        world = load_lisdf_pybullet(lisdf_path, verbose=True)
        wait_if_gui('load next test scene?')
        disconnect()
