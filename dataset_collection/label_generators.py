try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og

from src.dataset_collection.common import check_config
from src.env.space import initialize_space
from src.forward_kinematics.fkClient import FKClient
from src.state_validity_check.state_validity_checker import MoveitStateValidityChecker
from src.utils.utils import get_joint_names, convertWaypointToJointState
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

SELF_COLLISION_KEY = 'self_collision_free'
COLLISION_KEY = 'collision_free'

class LabelGenerator(object):
    """
    A label generator is responsible for taking in a joint configuration, and then produce the desired labels
    """
    def __init__(self, env_name):
        self.env_name = env_name
        self.label = ""
        self.headers = []

    def fill_label(self, waypoint):
        raise NotImplementedError

class SelfCollisionLabelGenerator(LabelGenerator):
    """
    A generator that produces environment collision label.
    """
    def __init__(self, env_name):
        super(SelfCollisionLabelGenerator, self).__init__(env_name)
        self.headers = [SELF_COLLISION_KEY]
        self.label = "selfCollision"
        space = initialize_space()
        ss = og.SimpleSetup(space)
        self.state_validity_checker = MoveitStateValidityChecker(ss.getSpaceInformation())

    def fill_label(self, waypoint):
        self_collision_free = check_config(waypoint, self.state_validity_checker, 'right')
        waypoint[self.headers[0]] = int(self_collision_free)

class EnvironmentCollisionLabelGenerator(LabelGenerator):
    """
    A generator that produces environment collision label.
    """
    def __init__(self, env_name):
        super(EnvironmentCollisionLabelGenerator, self).__init__(env_name)
        self.headers = [get_collision_label_name(self.env_name)]
        self.label = "envCollision"
        space = initialize_space()
        ss = og.SimpleSetup(space)
        self.state_validity_checker = MoveitStateValidityChecker(ss.getSpaceInformation())

    def fill_label(self, waypoint):
        label = self.headers[0]
        self_collision_free = waypoint.get(SELF_COLLISION_KEY)
        if self_collision_free == 0:
            # if in self_collision
            waypoint[label] = 0
        else:
            env_collision_free = check_config(waypoint, self.state_validity_checker, 'right')
            waypoint[label] = int(env_collision_free)

class EndEffectorPositionGenerator(LabelGenerator):
    """
    Label Generator that produces an end-effector position given the joint position;
    """
    def __init__(self, env_name):
        super(EndEffectorPositionGenerator, self).__init__(env_name)
        self.label = "pos"
        self.fkclient = FKClient()
        self.headers = ['x', 'y', 'z']

    def fill_label(self, waypoint):
        joint_state = convertWaypointToJointState(waypoint)
        resp = self.fkclient.get_fk(joint_state, ["right_hand"], "world")
        position = resp.pose_stamped[0].pose.position
        waypoint['x'] = position.x
        waypoint['y'] = position.y
        waypoint['z'] = position.z

def get_collision_label_name(env_name):
    return env_name + '_' + COLLISION_KEY