#!/usr/bin/python
import math
import tf
import tf2_ros
import rospy

import geometry_msgs
from geometry_msgs.msg import TransformStamped

# helper functions for geometry problems

def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

# rotate vector v1 by quaternion q1 
def quaternion_vector_mult(q1, v1):
    """ Multiplies a numpy quaternion with a numpy vector (array)"""
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

def convertMsgQuatToNumpyQuat(msg_quat):
    """ Converts a geometry_msgs Quaternion with x,y,z,w into an array with indices 0,1,2,3"""
    return [msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w]

def convertNumpyQuatToMsgQuat(numpy_quat):
    """ Converts an array with indices 0,1,2,3 into an geometry_msgs Quaternion with x,y,z,w"""
    msg_quaternion = geometry_msgs.msg.Quaternion()
    msg_quaternion.x = numpy_quat[0]
    msg_quaternion.y = numpy_quat[1]
    msg_quaternion.z = numpy_quat[2]
    msg_quaternion.w = numpy_quat[3]
    return msg_quaternion

def publishPoseToTf2(poseStamped, child_frame_id):
        # broadcast transform to tf2
        broadcaster = tf2_ros.TransformBroadcaster()
        transform = TransformStamped()

        transform.header.stamp = poseStamped.header.stamp
        transform.header.frame_id = poseStamped.header.frame_id
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = poseStamped.pose.position.x
        transform.transform.translation.y = poseStamped.pose.position.y
        transform.transform.translation.z = poseStamped.pose.position.z
        transform.transform.rotation.x = poseStamped.pose.orientation.x
        transform.transform.rotation.y = poseStamped.pose.orientation.y
        transform.transform.rotation.z = poseStamped.pose.orientation.z
        transform.transform.rotation.w = poseStamped.pose.orientation.w

        broadcaster.sendTransform(transform)

# rotate quaternion q1 by quaternion q2
# def quaternion_quaternion_mult(q1, q2):
#     return tf.transformations.quaternion_multiply(q2, q1)

if __name__ == '__main__':
    vec = [0,2,0]
    quat = tf.transformations.quaternion_from_euler(math.pi/2.0, 0, 0)
    quat2 = tf.transformations.quaternion_from_euler(math.pi/4, 0, 0)
    new_vec = quaternion_vector_mult(quat, vec)

    # new_quat = quaternion_quaternion_mult(quat2, quat)
    new_quat = tf.transformations.quaternion_multiply(quat, quat2)
    print (new_vec)
    print (tf.transformations.euler_from_quaternion(new_quat))