from pybullet_planning.world_builder.world_utils import get_file_by_category, get_parent_category
from os.path import join, isdir, isfile, dirname, abspath, basename, split
from examples.pybullet.utils.pybullet_tools.utils import set_pose, get_pose, connect, clone_world, \
    disconnect, set_client, add_data_path, WorldSaver, wait_for_user, get_joint_positions, get_configuration, \
    set_configuration, ClientSaver, HideOutput, is_center_stable, add_body_name, draw_base_limits, VideoSaver, load_model, \
    create_box, BROWN, get_aabb, stable_z, Pose, Point, Euler, set_color, WHITE, RGBA
from pybullet_tools.utils import get_aabb_extent, remove_body


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

def get_scale_by_category(file=None, category=None, scale=1):
    from world_builder.asset_constants import MODEL_HEIGHTS, MODEL_SCALES, OBJ_SCALES
    cat = category.lower()

    ## general category-level
    if category is not None:
        if cat in OBJ_SCALES:
            scale = OBJ_SCALES[cat]

    ## specific instance-level
    if file is not None:
        if category in MODEL_HEIGHTS:
            height = MODEL_HEIGHTS[category]['height']
            # print('bullet_utils.get_scale_by_category', file)
            scale = get_model_scale(file, h=height)
        elif category in MODEL_SCALES:
            f = file.lower()
            f = f[f.index(cat) + len(cat) + 1:]
            id = f[:f.index('/')]
            if id in MODEL_SCALES[category]:
                scale = MODEL_SCALES[category][id]
        else:
            parent = get_parent_category(category)
            if parent is None:
                print('\tcant find model scale', category, 'using default 1')
            elif parent in MODEL_SCALES:
                scale = MODEL_SCALES[parent][category]

    return scale


def get_model_scale(file, l=None, w=None, h=None, scale=1, category=None):
    scale_db = {}
    # if isfile(SCALE_DB):
    #     with open(SCALE_DB, "r") as read_file:
    #         scale_db = json.loads(read_file.read())
        # if file in scale_db:
        #     return scale_db[file]

    # if 'oven' in file.lower() or (category != None and category.lower() == 'oven'):
    #     print(file, l, w)

    ## ------- Case 1: no restrictions or directly given scale
    if w is None and l is None and h is None:
        return scale

    ## --- load and adjust
    with HideOutput():
        if isdir(file):
            file = join(file, 'mobility.urdf')
        body = load_model(file, scale=scale, fixed_base=True)
    aabb = get_aabb(body)
    extent = get_aabb_extent(aabb)

    ## ------- Case 2: given width and length of object
    if w is not None or l is not None:
        width, length = extent[:2]
        if w is None:
            scale *= l / length
        elif l is None:
            scale *= w / width
        else:
            scale *= min(l / length, w / width) ## unable to reconstruct

    ## ------ Case 3: given height of object
    elif h is not None:
        height = extent[2]
        scale *= h / height

    ## ------ Case N: exceptions
    if category is not None:
        if 'door' == category.lower():
            set_joint_position(body, get_joints(body)[1], -0.8)
        if 'dishwasher' == category.lower():
            set_joint_position(body, get_joints(body)[3], -0.66)
        if 'door' == category.lower():
            scale = (l / length + w / width) / 2

    # scale_db[file] = scale
    # with open(SCALE_DB, 'w') as f:
    #     json.dump(scale_db, f)
    remove_body(body)

    return scale