#!/usr/bin/env python

import rospy
import rospkg
from gazebo_ros.gazebo_interface import DeleteModelRequest, SpawnModelRequest, SpawnModel, DeleteModel, SetModelState, SetModelStateRequest

from ipa_navigation_utils import log

import sys
import yaml
import numpy as np
import subprocess

ros_packages = rospkg.RosPack()
object_path = ros_packages.get_path('cob_gazebo_objects') + '/objects/'

### PARAMETERS
CONFIG_FILE = ros_packages.get_path('cob_bringup_sim') + '/config/benchmark_config.yaml'
RADIUS = 0.71

### GLOBAL VARIABLES
objects_ = {}
object_ids_ = {}
moving_objects_ = []

log.level = log.DEBUG


class CONFIG:
    """
    Contains the configuration parameters for the benchmark-tool node.
    """
    inflation_radius = None
    room_number = None
    max_num_objects = None
    spawn_object_rate = None
    num_changed_objects = None
    spawn_moving_objects = None

    models = None
    room_boundaries = None
    moving_objects = None
    paths = None

    @classmethod
    def load(cls, file_path):
        """
        Loads all config parameters from the specified yaml-file.
        :param file_path: The yaml-file containing the config.
        :type file_path: str
        :return:
        :rtype: None
        """
        with open(file_path) as f:
            data = yaml.safe_load(f)

        cls.inflation_radius = data['inflation_radius']
        cls.room_number = data['room_number']
        cls.max_num_objects = data['max_num_objects']
        cls.spawn_object_rate = data['spawn_object_rate']
        cls.num_changed_objects = data['num_changed_objects']
        cls.spawn_moving_objects = data['spawn_moving_objects']

        cls.models = data['models']
        cls.room_boundaries = data['boundaries']
        cls.moving_objects = data['moving_objects']
        cls.paths = data['paths']

        for model in cls.models.keys():
            cls.models[model]['path'] = ros_packages.get_path(cls.models[model]['package']) + '/' + \
                                        cls.models[model]['path']

        for room in cls.paths:
            for path in cls.paths[room]:
                path['start_x'] -= cls.inflation_radius
                path['start_y'] -= cls.inflation_radius
                path['end_x'] += cls.inflation_radius
                path['end_y'] += cls.inflation_radius

    @classmethod
    def set_inflation_radius(cls, inflation_radius):
        """
        Sets the inflation radius to the new value and updates all paths.
        :param inflation_radius: The new inflation radius value.
        :type inflation_radius: float
        :return:
        :rtype: None
        """
        diff = inflation_radius - cls.inflation_radius

        for room in cls.paths:
            for path in room[room.keys()[0]]:
                path['start_x'] -= diff
                path['start_y'] -= diff
                path['end_x'] += diff
                path['end_y'] += diff

        cls.inflation_radius = inflation_radius


class Object:
    def __init__(self, x, y, room_id, radius=0.5):
        self.pos = np.array([x, y])
        self.room_id = room_id
        self.radius = radius


def spawn_moving_objects():
    global moving_objects_

    log.debug('Spawning moving objects')

    moving_obj_id = 0
    for moving_obj in CONFIG.moving_objects:
        if moving_obj['room_id'] not in CONFIG.spawn_moving_objects:
            continue

        # Spawn model in Gazebo
        model_name = 'moving_' + moving_obj['type'] + '_' + str(moving_obj_id)
        obj_pos = np.array(moving_obj['path'][0])
        spawn_model(model_name, CONFIG.models[moving_obj['type']]['path'], obj_pos)

        # Run driver to move object around
        new_obj_proc = subprocess.Popen(['rosrun', 'cob_bringup_sim', 'move_object.py',
                                         '--mode='+moving_obj['path_type'],
                                         '--name='+model_name,
                                         '--velocity='+str(moving_obj['velocity']),
                                         '--polygon='+str(moving_obj['path'])])
        moving_objects_.append([model_name, new_obj_proc])

        moving_obj_id += 1

    log.info('Successfully spawned moving objects')


def spawn_object_loop(radius, type=None):
    """
    Loops until node terminates and spawns objects dynamically at the given rate.
    :param radius: The radius of the objects to spawn.
    :type radius: float
    :param type: The type of the objects to spawn.
    :type type: str
    :return:
    :rtype: None
    """
    while not rospy.is_shutdown():
        for i in range(CONFIG.num_changed_objects):
            log.debug('Changing another spawned object')
            # Select randomly a spawned object to delete
            spawned_object_ids = np.where(object_ids_[type]>0)[0]
            if spawned_object_ids.shape[0] <= 0:
                log.error('No objects spawned yet, shutting down now')
                return
            obj_id = np.random.choice(spawned_object_ids, 1)[0]
            model_name = type + '_' + str(obj_id)

            # # Delete selected model
            # delete_model(del_model_name)
            #
            # # Spawn a new object
            # spawn_object(type, np.random.randint(CONFIG.room_number), radius)

            room_id = np.random.randint(CONFIG.room_number)
            obj_pos = get_new_coordinates(room_id, radius)
            if len(obj_pos) <= 0:
                continue

            move_model(model_name, obj_pos)

            objects_[type][obj_id] = np.copy(obj_pos)

        rospy.sleep(CONFIG.spawn_object_rate)


def get_new_coordinates(room_id, radius, threshold=1000):
    coord_check = False
    threshold = 100
    count = 0

    while not coord_check:
        obj_pos = generate_coordinates(room_id, radius)

        coord_check = check_coordinates(obj_pos, room_id, radius)

        count += 1
        if count >= threshold:
            log.warn('Could not find a valid position for model with type \'{}\' in room {}'.format(type, room_id))
            log.warn('Aborting now')
            return []

    return obj_pos


def spawn_object(type, room_id, radius, name=None):
    """
    Spawns an object with the given type in the specified room.
    :param type: The type of the the object to spawn.
    :type type: str
    :param room_id: The room-id for the room the object should be spawned in.
    :type room_id: int
    :param radius: The radius of the object to spawn.
    :type radius: float
    :return:
    :rtype: None
    """
    global objects_

    obj_pos = get_new_coordinates(room_id, radius)
    if len(obj_pos) <= 0:
        return

    obj_id = np.argmax(object_ids_[type]==0)
    model_name = type + '_' + str(obj_id)

    # Set used object id to true (1)
    object_ids_[type][obj_id] = 1

    # Set coordinates for object
    objects_[type][obj_id] = obj_pos

    spawn_model(model_name, CONFIG.models[type]['path'], obj_pos)


def check_coordinates(new_obj_pos, room_id, radius):
    """
    Performs a check on new object coordinates. The function verifies, that the position lies not within the boundaries
    of another object or a robot path.
    :param new_obj_pos: The position (x, y) of the new object.
    :type new_obj_pos: ndarray
    :param room_id: The id of the room, where the object is supposed to be spawned.
    :type room_id: int
    :param radius: The maximum radius of the object.
    :type radius: float
    :return: True if the object is collision-free, False otherwise.
    :rtype: bool
    """
    # Check all objects within the room
    for type in objects_:
        for obj_pos in objects_[type]:
            dist = np.linalg.norm(new_obj_pos - obj_pos)
            if dist <= radius:
                log.info('New Object didn\'t pass the object location check [room={}, pos=({}, {}), radius={}]'.format(
                    room_id, new_obj_pos[0], new_obj_pos[1], radius))
                return False

    # Check for all paths in the room
    for path in CONFIG.paths['room_'+str(room_id)]:
        in_x = path['start_x']-radius <= new_obj_pos[0] <= path['end_x']+radius
        in_y = path['start_y']-radius <= new_obj_pos[1] <= path['end_y']+radius

        if in_x and in_y:
            log.debug('New Object didn\'t pass the robot path check [room={}, pos=({}, {}), radius={}]'.format(
                room_id, new_obj_pos[0], new_obj_pos[1], radius))
            return False

    # Check for moving obj paths
    for moving_obj in CONFIG.moving_objects:
        if moving_obj['room_id'] not in CONFIG.spawn_moving_objects:
            continue

        for i, start in enumerate(moving_obj['path'][:-1]):
            end = moving_obj['path'][i+1]
            if start[0] <= end[0]:
                start_x = start[0]
                end_x = end[0]
            else:
                start_x = end[0]
                end_x = start[0]
            if start[1] <= end[1]:
                start_y = start[1]
                end_y = end[1]
            else:
                start_y = end[1]
                end_y = start[1]

            in_x = start_x - radius*2 <= new_obj_pos[0] <= end_x + radius*2
            in_y = start_y - radius*2 <= new_obj_pos[1] <= end_y + radius*2

            if in_x and in_y:
                log.debug('New Object didn\'t pass the moving object path check [room={}, pos=({}, {}), radius={}]'.format(
                    room_id, new_obj_pos[0], new_obj_pos[1], radius))
                return False

    log.debug('New Object passed the check [room={}, pos=({}, {}), radius={}]'.format(
        room_id, new_obj_pos[0], new_obj_pos[1], radius))

    return True


def generate_coordinates(room_id, radius):
    """
    Randomly generates a set of coordinates (x, y) that lie within the given rooms boundaries.
    :param room_id: The id of the room where the coordinates are supposed to be in.
    :type room_id: int
    :param radius: The radius of the object, the coordinates are generated for.
    :type radius: float
    :return: The new coordinates (x, y).
    :rtype: ndarray
    """
    x = np.random.uniform(CONFIG.room_boundaries[room_id]['start_x']+radius,
                          CONFIG.room_boundaries[room_id]['end_x']-radius)
    y = np.random.uniform(CONFIG.room_boundaries[room_id]['start_y']+radius,
                          CONFIG.room_boundaries[room_id]['end_y']-radius)

    return np.array([x, y])


def spawn_init_objects(type, radius):
    """
    Spawns initially the number of objects specified in the config.
    :param type: The type of the object to be spawned.
    :type type: str
    :param radius: The maximum radius of the object to be spawned.
    :type radius: float
    :return:
    :rtype: None
    """
    log.debug('Spawning {} initial objects'.format(CONFIG.max_num_objects))
    for i in range(CONFIG.max_num_objects):
        spawn_object(type, np.random.randint(CONFIG.room_number), radius)

    log.info('Spawned {} initial objects'.format(CONFIG.max_num_objects))


def delete_all_objects():
    """
    Deletes all objects that were created from this node.
    :return:
    :rtype: None
    """
    global moving_objects_

    log.debug('Deleting all dynamic objects')
    for dyn_obj in moving_objects_:
        delete_model(dyn_obj[0])

    log.info('Successfully deleted all dynamic objects')

    log.debug('Deleting all static objects')
    for obj_type in object_ids_:
        for i in range(len(object_ids_[obj_type])):
            if object_ids_[obj_type][i] > 0:
                delete_model(obj_type + '_' + str(i))

    log.info('Successfully deleted all static objects')


def move_model(model_name, new_pos):
    log.debug('Waiting for service: /gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
    try:
        log.debug('Trying to move model \'{}\' to position {}'.format(model_name, new_pos))
        req = SetModelStateRequest()
        req.model_state.model_name = model_name
        req.model_state.pose.position.x = new_pos[0]
        req.model_state.pose.position.y = new_pos[1]
        srv_mov_scanner = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        ret = srv_mov_scanner(req)
        if not ret.success:
            log.warn('Could not move model \'{}\' because an error occured during service call'.format(model_name))
            return
    except rospy.ServiceException, e:
        log.warn("Service call failed: %s" % e)
        return

    log.info('Moved model \'{}\' to new position at [{:.2f}, {:.2f}]'.format(model_name, new_pos[0], new_pos[1]))

    return


def delete_model(model_name):
    """
    Deletes the given model from the current gazebo simulation.
    :param model_name: The name of the model to be removed.
    :type model_name: str
    :return:
    :rtype: None
    """
    log.debug('Waiting for service: /gazebo/delete_model')
    rospy.wait_for_service('/gazebo/delete_model', timeout=2.0)
    try:
        log.debug('Trying to delete model: {}'.format(model_name))
        req = DeleteModelRequest()
        req.model_name = str(model_name)
        srv_del_scanner = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        ret = srv_del_scanner(req)
        if not ret.success:
            log.warn('Could not delete model \'{}\' because an error occured during service call'.format(model_name))
            return
    except rospy.ServiceException, e:
        log.warn("Service call failed: %s" % e)
        return

    log.info('Deleted existing model \'{}\''.format(model_name))

    # Reset object id to 0
    obj_id = int(model_name.split('_')[-1])
    obj_name = model_name.split('_')[0]

    if not 'moving' in model_name:
        object_ids_[obj_name][obj_id] = 0

    return


def spawn_model(model_name, urdf_path, position=None):
    """
    Spawns the specified model in the current gazebo simulation.
    :param model_name: The name of the model to be spawned.
    :type model_name: str
    :param urdf_path: The path of the model to be spawned.
    :type urdf_path: str
    :param position: The position (x, y) of the model to be spawned.
    :type position: ndarray
    :return:
    :rtype: None
    """
    log.info('Spawning new model \'{}\' at position [{}, {}]'.format(model_name, position[0], position[1]))
    rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=2.0)
    try:
        req = SpawnModelRequest()
        req.model_name = model_name

        # Load Model XML
        f = open(urdf_path, 'r')
        req.model_xml = f.read()
        if req.model_xml == "":
            rospy.logerr("Error: file is empty %s", urdf_path)
            sys.exit(0)

        if position is not None:
            req.initial_pose.position.x = position[0]
            req.initial_pose.position.y = position[1]

        srv_del_scanner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        ret = srv_del_scanner(req)
        if ret.success:
            log.info('Successfully spawned model \'{}\'.'.format(model_name))
        else:
            log.warn('Could not spawn model \'{}\'.'.format(model_name))
    except rospy.ServiceException, e:
        log.warn("Service call failed: %s" % e)


def shutdown_moving_objects():
    global moving_objects_

    log.debug('Shutting down drivers for moving objects')

    for moving_obj in moving_objects_:
        moving_obj[1].terminate()
        moving_obj[1].wait()
        log.info('Terminated driver for moving object: '+moving_obj[0])

    log.info('Successfully shut down all drivers for moving objects')


def on_shutdown():
    log.info('Shutting down now')

    shutdown_moving_objects()

    delete_all_objects()

    log.info('Shutdown complete')


def init_ros():
    """
    Initializes ros with subscribers, publishers, etc.
    :return:
    :rtype: None
    """
    rospy.init_node('benchmark_tool')

    rospy.on_shutdown(on_shutdown)


def initialize():
    """
    Initializes the node including ros and the config.
    :return:
    :rtype: None
    """
    global objects_, object_ids_

    init_ros()

    CONFIG.load(CONFIG_FILE)

    # objects_ = [[] for i in range(CONFIG.room_number)
    for obj_name in CONFIG.models.keys():
        objects_[obj_name] = np.zeros((CONFIG.max_num_objects, 2))
        object_ids_[obj_name] = np.zeros(CONFIG.max_num_objects)

    log.info('Done initializing')


def main():
    """
    The main function of the node. Contains the core functionality.
    :return:
    :rtype: None
    """
    initialize()

    spawn_init_objects('box', RADIUS)

    if len(CONFIG.spawn_moving_objects) > 0:
        spawn_moving_objects()

    spawn_object_loop(RADIUS, 'box')

    delete_all_objects()


if __name__ == '__main__':
    main()
