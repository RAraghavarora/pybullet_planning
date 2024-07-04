from pybullet_planning.world_builder.world_utils import get_file_by_category, get_scale_by_category, get_model_scale
from os.path import join, isdir, isfile, dirname, abspath, basename, split
from examples.pybullet.utils.pybullet_tools.utils import set_pose, get_pose, connect, clone_world, \
    disconnect, set_client, add_data_path, WorldSaver, wait_for_user, get_joint_positions, get_configuration, \
    set_configuration, ClientSaver, HideOutput, is_center_stable, add_body_name, draw_base_limits, VideoSaver, load_model, \
    create_box, BROWN, get_aabb, stable_z, Pose, Point, Euler, set_color, WHITE, RGBA

DARK_GREEN = RGBA(35/255, 66/255, 0, 1)

def load_asset(category, x=0, y=0, yaw=0, floor=None, z=None, w=None, l=None, h=None,
               scale=1, verbose=False, random_instance=False, sampling=False, random_scale=1.0):

    """ ============= load body by category ============= """
    file = get_file_by_category(category, random_instance=random_instance, sampling=sampling)
    if verbose and file is not None:
        print(f"Loading ...... {abspath(file)}", end='\r')

    scale = get_scale_by_category(file=file, category=category, scale=scale)
    if file is not None:
        scale = get_model_scale(file, l, w, h, scale, category)

        scale = random_scale * scale

        with HideOutput():
            body = load_model(file, scale=scale, fixed_base=True)
    else:
        body = create_box(w=w, l=l, h=1, color=BROWN, collision=True)

    """ ============= place object z on a surface or floor ============= """
    if z is None:
        if category.lower() in ['oven']:
            aabb = get_aabb(body)
            height = aabb.upper[2] - aabb.lower[2]
            z = height / 2
        elif isinstance(floor, tuple):
            z = stable_z(body, floor[0], floor[1])
        elif isinstance(floor, int):
            z = stable_z(body, floor)
        elif hasattr(floor, 'body') and isinstance(floor.body, int):
            z = stable_z(body, floor.body)
        else:
            z = 0
    pose = Pose(point=Point(x=x, y=y, z=z), euler=Euler(yaw=yaw))
    set_pose(body, pose)

    """ ============= category-specific modification ============= """
    if category.lower() == 'veggieleaf':
        set_color(body, DARK_GREEN, 0)
    elif category.lower() == 'veggiestem':
        set_color(body, WHITE, 0)
    elif category.lower() == 'facetbase':
        from pybullet_tools.bullet_utils import open_doors_drawers
        open_doors_drawers(body)

    """ ============= create an Object ============= """
    # if movable:
    #     object = Movable(body, category=category)
    # elif category.lower() == 'food':
    #     # index = file.replace('/mobility.urdf', '')
    #     # index = index[index.index('models/')+7:]
    #     # index = index[index.index('/')+1:]
    #     index = partnet_id_from_path(file)
    #     return body, file, scale, index
    # else:
    #     object = Object(body, category=category)

    # if category.lower() == 'food':
    #     # index = file.replace('/mobility.urdf', '')
    #     # index = index[index.index('models/')+7:]
    #     # index = index[index.index('/')+1:]
    #     index = partnet_id_from_path(file)
    #     return body, file, scale, index
    return body, file, scale
