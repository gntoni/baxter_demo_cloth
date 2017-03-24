#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_interface import Gripper, Limb, RobotEnable
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from sensor_msgs.msg import Image
import copy
from baxter_perception.srv import graspPoint, graspPointRequest, graspPointResponse
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import tf

import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


#################################################
# Callbacks & Service Proxies & Publishers/Subscribers
#


def imageDepth_cb(msg):
    global imsgd
    imsgd = msg


def imageColor_cb(msg):
    global imsgc
    imsgc = msg


def armMove_cb(msg, group):
    print "============ New grasp goal received"
    group.set_pose_target(msg)

    plan1 = group.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(2)

    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)

    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(2)

    group.go(wait=True)


ikLeft_ns = '/ExternalTools/left/PositionKinematicsNode/IKService'
ikLeft_svc = rospy.ServiceProxy(ikLeft_ns, SolvePositionIK)
ikRight_ns = '/ExternalTools/right/PositionKinematicsNode/IKService'
ikRight_svc = rospy.ServiceProxy(ikRight_ns, SolvePositionIK)
getGraspPt_ns = '/get_grasp_point'
getGraspPt_svc = rospy.ServiceProxy(getGraspPt_ns, graspPoint)

sub = rospy.Subscriber(rospy.get_param("camera_depth_topic"), Image, imageDepth_cb)
sub = rospy.Subscriber(rospy.get_param("camera_color_topic"), Image, imageColor_cb)

#################################################
# Initialization
#
rospy.init_node("ClothManipulationDemo")
baxter = RobotEnable()
baxter.enable()

bridge = CvBridge()
listener = tf.TransformListener()

gripperR = Gripper('right')
gripperL = Gripper('left')
gripperR.calibrate()
gripperL.calibrate()
limbR = Limb('right')
limbL = Limb('left')
limbR.set_joint_position_speed(0.1)
limbL.set_joint_position_speed(0.1)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
groupLeftArm = moveit_commander.MoveGroupCommander("left_arm")
groupRightArm = moveit_commander.MoveGroupCommander("right_arm")
display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory)


#################################################
# Righ arm init
#
limbR.move_to_neutral()

# pt = Point(x=0.7, y=0.0, z=0.4)
# ori = Quaternion(x=0.382683432, y=-0.923879532, z=-0.0, w=0.0)
pt = rospy.get_param("hanging_arm_position")
hangArmPosition = Point(x=pt[0], y=pt[1], z=pt[2])

ori = rospy.get_param("hanging_arm_orientation")
hangArmOrientation = Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3])

hdr = Header(stamp=rospy.Time.now(), frame_id='base')

pos = Pose(position=hangArmPosition, orientation=hangArmOrientation)
psst = PoseStamped(header=hdr, pose=pos)

ikrqt = SolvePositionIKRequest()
ikrqt.pose_stamp = [psst]
rospy.wait_for_service(ikRight_ns)
res = ikRight_svc(ikrqt)
goal_angles = dict(zip(res.joints[0].name, res.joints[0].position))


#################################################
# Left arm init
#
gripperL.open()
limbL.move_to_neutral()

#################################################
# Get image
#
img_depth = copy.deepcopy(imsgd)
graspPtReq_msg = graspPointRequest()
graspPtReq_msg.depth = img_depth

rospy.wait_for_service(getGraspPt_ns)
graspPtResp = getGraspPt_svc(graspPtReq_msg)

x_d = graspPtResp.graspPoint.x
y_d = graspPtResp.graspPoint.y

# Transform point to 3D coordinates in camera frame
img = bridge.imgmsg_to_cv2(img_depth)
patch_depth = img[int(y_d)-2:int(y_d)+2, int(x_d)-2:int(x_d)+2].copy()
patch_depth[patch_depth == 0] = patch_depth.max()
depth_x_y = np.min(patch_depth)/1000.0

# TODO Parameters
cx_d = 480.9395901553567
cy_d = 268.35982687586403
fx_d = 527.0985907291398
fy_d = 526.5497893423392

p_cam = PointStamped()
p_cam.point.x = x
p_cam.point.y = y
p_cam.point.z = z
p_cam.header.frame_id='kinect2_link'
p_base = listener.transformPoint('/base',p_cam)

# TODO parameters (multiples de 90)
GraspPt = geometry_msgs.msg.PoseStamped()
GraspPt.pose.position.x = p_base.point.x
GraspPt.pose.position.y = p_base.point.y
GraspPt.pose.position.z = p_base.point.z
GraspPt.pose.orientation.x = 0.505720017498
GraspPt.pose.orientation.y = -0.461847671509
GraspPt.pose.orientation.z = 0.53446382832
GraspPt.pose.orientation.w = 0.495270035881
GraspPt.header.frame_id = 'base'

# TODO parameters
lgoal_angles = {
 'left_e0': -0.004311786537481661,
 'left_e1': 1.7260304016167203,
 'left_s0': -0.2057305781367268,
 'left_s1': -0.41681713345880916,
 'left_w0': -1.219336093642705,
 'left_w1': 1.7110492700878688,
 'left_w2': -0.19780227454625238
}

# Go graspReady
limbL.move_to_joint_positions(lgoal_angles)

# Go preGrasp
armMove_cb(GraspPt)

# Grasp
rospy.sleep(3)
GraspPt.pose.position.y -= 0.1
armMove_cb(GraspPt)
gripperL.close()

# Extend
GraspPt.pose.position.y += 0.3
GraspPt.pose.position.z += 0.2
armMove_cb(GraspPt)

# Show reverse
GraspPt.pose.position.x -= 0.15
armMove_cb(GraspPt)

GraspPt.pose.position.y -= 0.50
armMove_cb(GraspPt)
