import tf
import numpy as np

def combine_transform(p1, p2):
    """
    Utility method for combining two transforms, adds translational component, and multiplies quaternions
    :param t1: transform 1
    :param t2: transform 2
    :return: resulting transform
    """
    r1 = tf.transformations.euler_from_quaternion(p1[3:7])
    r2 = tf.transformations.euler_from_quaternion(p2[3:7])
    m1 = tf.transformations.compose_matrix(translate=p1[0:3], angles=r1)
    m2 = tf.transformations.compose_matrix(translate=p2[0:3], angles=r2)
    res = m1.dot(m2)
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(res)
    rot_quat = tf.transformations.quaternion_from_euler(*angles)
    return trans.tolist() + rot_quat.tolist()


def compute_transform(p1, p2):
    """
    Utility method for computing transform between two poses, subtracts translation, and divides quaternions
    :param p1: pose 1
    :param p2: pose 2
    :return: resulting transform
    """
    r1 = tf.transformations.euler_from_quaternion(p1[3:7])
    r2 = tf.transformations.euler_from_quaternion(p2[3:7])
    m1 = tf.transformations.compose_matrix(translate=p1[0:3], angles=r1)
    m2 = tf.transformations.compose_matrix(translate=p2[0:3], angles=r2)
    res = np.linalg.inv(m1).dot(m2)
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(res)
    rot_quat = tf.transformations.quaternion_from_euler(*angles)
    return trans.tolist()+rot_quat.tolist()


def find_max_count(data):
    """
    Utility function for finding element in dictionary with most items
    :param data: dict
    """
    max_count, max_id = -1, -1
    for key, value in data.iteritems():
        if len(value) > max_count:
            max_id = key
            max_count = len(value)
    return max_id


def standardize_pose(pose):
    """
    Convert geometry msgs pose to numpy array and standardize pose format
    :param pose: geom msgs pose
    :return: numpy array
    """
    trans = pose.position
    orient = pose.orientation
    return np.array([trans.x, trans.y, trans.z, orient.x, orient.y, orient.z, orient.w])
