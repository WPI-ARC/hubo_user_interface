#!/usr/bin/python

#############################################################
#                                                           #
#   Calder Phillips-Grafflin -- ARC Lab                     #
#                                                           #
#   Displays interactive markers of valve-like objects and  #
#   allows the user to align the objects against the world  #
#   represented by the robot's sensors.                     #
#                                                           #
#############################################################

import rospy
import math
import time
import threading
import subprocess
from copy import deepcopy

from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from shape_msgs.msg import *
from transformation_helper import *
from rosgraph_msgs.msg import *
from teleop_msgs.msg import *
from teleop_msgs.srv import *

class MarkerStatus:



    def __init__(self):
        self.default_radius = 0.1
        self.default_thickness = 0.02
        self.radius = self.default_radius
        self.default_pose_stamped = PoseStamped()
        self.default_pose_stamped.header.frame_id = "/Body_TSY"
        self.default_pose_stamped.pose.position.x = 0.0
        self.default_pose_stamped.pose.position.y = 0.0
        self.default_pose_stamped.pose.position.z = .75
        self.default_pose_stamped.pose.orientation.x = 0.0
        self.default_pose_stamped.pose.orientation.y = 0.0
        self.default_pose_stamped.pose.orientation.z = 0.0
        self.default_pose_stamped.pose.orientation.w = 1.0
        self.session_pose_stamped = deepcopy(self.default_pose_stamped)
        self.pose_stamped = deepcopy(self.default_pose_stamped)

        self.rotate = 0



class MenuStatus:

    def __init__(self):
        self.flash = 0
        self.warningFlash = 0
        self.warningState = False
        self.numWarnings = 0
        self.numErrors = 0
        self.numFatal = 0
        self.alignmentToolChecked = False
        self.gripperTeleopChecked = False
        self.robotPlacementChecked = False




class Control_Marker:

    def __init__(self, pointcloud_request_service, laser_request_service, head_control_action):
        self.current_tilt = 0.0
        self.current_pan = 0.0
        self.valve_localization_pub = rospy.Publisher("valve_localization/show", Bool)
        self.robot_placement_pub = rospy.Publisher("robot_placement/show", Bool)
        self.pointcloud_request_client = None
        if (pointcloud_request_service != None):
            self.pointcloud_request_client = rospy.ServiceProxy(pointcloud_request_service, RateControl)
        else:
            rospy.logwarn("RGBD Pointcloud request is not enabled!")
        self.laser_request_client = None
        if (laser_request_service != None):
            self.laser_request_client = rospy.ServiceProxy(laser_request_service, RateControl)
        else:
            rospy.logwarn("LIDAR Pointcloud request is not enabled!")
        self.head_client = None
        if (head_control_action != None):
            self.head_client = actionlib.SimpleActionClient(head_control_action, hrms.PointHeadAction)
        else:
            rospy.logwarn("Head pointing disabled")

        rospy.Subscriber("/rosout_agg", Log, self.monitorMsgsCB)

        #Setup The Valve and Valve Marker
        self.status = MarkerStatus()
        self.menu = MenuStatus()
        self.populate_menu()

        #Setup the interactive marker server
        self.server = InteractiveMarkerServer("control_crystal")

        # Make sure pointcloud is request-only
        req = RateControlRequest()
        req.Rate = 0.0
        try:
            res = self.pointcloud_request_client.call(req)
            print "Switched RGBD pointcloud transport to request-only mode"
        except:
            print "Service call failed - is RGBD pointcloud transport running?"

        # Make sure pointcloud is request-only
        req = RateControlRequest()
        req.Rate = 0.0
        try:
            res = self.laser_request_client.call(req)
            print "Switched LIDAR pointcloud transport to request-only mode"
        except:
            print "Service call failed - is LIDAR pointcloud transport running?"

        # Make sure the head is pointing forwards
        head_goal = hrms.PointHeadGoal()
        head_goal.target_type = head_goal.ANGLETARGET
        head_goal.pan_angle = self.current_pan
        head_goal.tilt_angle = self.current_tilt
        head_goal.min_duration = rospy.Duration(5.0)
        try:
            self.head_client.send_goal(head_goal)
            self.head_client.wait_for_result(rospy.Duration(5.0))
            print "Ensured head is at the zero position"
        except:
            print "PointHeadAction call failed - is the head control running?"

        # Setup the Refresh Rate of the Marker
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()



    def update(self):
        # This bugfix should have made it into Groovy
        #for marker_name in self.server.marker_contexts.keys():
        #    self.server.erase(marker_name)
        self.server.clear()


        if self.menu.numWarnings > 0:
            if self.menu.warningState == False:
                self.menu.warningState = True

            if self.menu.warningState == True:
                self.menu.flash += 1

            if self.menu.flash >= 60:
                self.menu.flash = 0
                self.menu.warningState = False
                self.menu.numWarnings = 0
                self.populate_menu()

        self.status.rotate += 1
        if self.status.rotate > 60:
            self.status.rotate = 1

        alignment_marker = self.make_alignment_imarker(self.status.pose_stamped)
        self.server.insert(alignment_marker, self.alignment_feedback_cb)
        self.menu_handler.apply(self.server, "control_marker")

        self.server.applyChanges()






##################################
#   Valve Menu Stuff             #
##################################


    def populate_menu(self):
        global tools_entry, alignment_marker_show_check, warning_entry, \
               errors_entry, fatal_entry, fatal_t, warning_t, error_t

        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Request RGBD Cloud", callback = self.pointCloudRGBDCB)
        self.menu_handler.insert("Request LIDAR Cloud", callback = self.pointCloudLIDARCB)

        self.menu_handler.insert("Tilt head UP 10 DEGREES", callback = self.tiltHeadUpCB)
        self.menu_handler.insert("Tilt head DOWN 10 DEGREES", callback = self.tiltHeadDownCB)
        self.menu_handler.insert("Pan head LEFT 10 DEGREES", callback = self.panHeadLeftCB)
        self.menu_handler.insert("Pan head RIGHT 10 DEGREES", callback = self.panHeadRightCB)

        tools_entry = self.menu_handler.insert("Tools")

        # Make the Alignment Marker
        alignment_marker_tool = self.menu_handler.insert("Alignment Marker", \
                                parent = tools_entry, \
                                callback = self.alignmentToolCB)
        if self.menu.alignmentToolChecked == True:
            self.menu_handler.setCheckState(alignment_marker_tool, MenuHandler.CHECKED)
        elif self.menu.alignmentToolChecked == False:
            self.menu_handler.setCheckState(alignment_marker_tool, MenuHandler.UNCHECKED)

        # Make the Placement Tool
        robot_placement_tool = self.menu_handler.insert("Robot Placement", \
                                parent = tools_entry, \
                                callback = self.robotPlacementCB)
        if self.menu.robotPlacementChecked == True:
            self.menu_handler.setCheckState(robot_placement_tool, MenuHandler.CHECKED)
        elif self.menu.robotPlacementChecked == False:
            self.menu_handler.setCheckState(robot_placement_tool, MenuHandler.UNCHECKED)

        # Make the Gripper Teleop Tool
        gripper_teleop_tool = self.menu_handler.insert("Gripper Teleop", \
                                parent = tools_entry, \
                                callback = self.gripperTeleopToolCB)
        if self.menu.gripperTeleopChecked == True:
            self.menu_handler.setCheckState(gripper_teleop_tool, MenuHandler.CHECKED)
        elif self.menu.gripperTeleopChecked == False:
            self.menu_handler.setCheckState(gripper_teleop_tool, MenuHandler.UNCHECKED)

        #Make the Menu Entries for the Warnings, Errors, and Fatals
        warning_entry = self.menu_handler.insert("Warnings ("+ str(self.menu.numWarnings) + ")", callback = self.warningClear)
        errors_entry = self.menu_handler.insert("Errors ("+ str(self.menu.numErrors) + ")", callback = self.errorClear)
        fatal_entry = self.menu_handler.insert("Fatal ("+ str(self.menu.numFatal) + ")", callback = self.fatalClear)
        self.menu_handler.insert("Clear All Problems", callback = self.menuReset)



    def menuReset(self, feedback):
        self.menu.numWarnings = 0
        self.menu.numErrors = 0
        self.menu.numFatal = 0
        self.populate_menu()


    def warningClear(self, feedback):
        self.menu.numWarnings = 0
        self.populate_menu()


    def errorClear(self, feedback):
        self.menu.numErrors = 0
        self.populate_menu()


    def fatalClear(self, feedback):
        self.menu.numFatal = 0
        self.populate_menu()



    def pointCloudRGBDCB(self, feedback):
        # Make sure pointcloud is request-only
        req = RateControlRequest()
        req.Rate = -1.0
        try:
            res = self.pointcloud_request_client.call(req)
            print "Requested a single RGBD pointcloud"
        except:
            print "Request pointcloud call failed - is RGBD pointcloud transport running?"

    def pointCloudLIDARCB(self, feedback):
        # Make sure pointcloud is request-only
        req = RateControlRequest()
        req.Rate = -1.0
        try:
            res = self.laser_request_client.call(req)
            print "Requested a single LIDAR pointcloud"
        except:
            print "Request pointcloud call failed - is LIDAR pointcloud transport running?"



    def tiltHeadUpCB(self, feedback):
        current_tilt = self.current_tilt
        new_tilt = current_tilt - (math.pi / 18.0)
        rospy.loginfo("Tilting the head right from " + str(current_tilt) + " to " + str(new_tilt))
        self.controlHead(self.current_pan, new_tilt)
        rospy.loginfo("Head tilting complete")
        pass

    def tiltHeadDownCB(self, feedback):
        current_tilt = self.current_tilt
        new_tilt = current_tilt + (math.pi / 18.0)
        rospy.loginfo("Tilting the head right from " + str(current_tilt) + " to " + str(new_tilt))
        self.controlHead(self.current_pan, new_tilt)
        rospy.loginfo("Head tilting complete")
        pass

    def panHeadLeftCB(self, feedback):
        current_pan = self.current_pan
        new_pan = current_pan + (math.pi / 18.0)
        rospy.loginfo("Panning the head right from " + str(current_pan) + " to " + str(new_pan))
        self.controlHead(new_pan, self.current_tilt)
        rospy.loginfo("Head panning complete")
        pass

    def panHeadRightCB(self, feedback):
        current_pan = self.current_pan
        new_pan = current_pan - (math.pi / 18.0)
        rospy.loginfo("Panning the head right from " + str(current_pan) + " to " + str(new_pan))
        self.controlHead(new_pan, self.current_tilt)
        rospy.loginfo("Head panning complete")
        pass

    def controlHead(self, pan, tilt):
        # Make sure the head is pointing forwards
        head_goal = hrms.PointHeadGoal()
        head_goal.target_type = head_goal.ANGLETARGET
        head_goal.pan_angle = pan
        head_goal.tilt_angle = tilt
        head_goal.min_duration = rospy.Duration(1.0)
        try:
            self.head_client.send_goal(head_goal)
            self.head_client.wait_for_result(rospy.Duration(1.0))
            print "Head pointing completed"
        except:
            print "PointHeadAction call failed - is the head control running?"


    def alignmentToolCB(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if state == MenuHandler.CHECKED:
            print "Turning Off Alignment Tool"
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self.valve_localization_pub.publish(False)
            self.menu.alignmentToolChecked = False
        else:
            print "Turning On Alignment Tool"
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self.valve_localization_pub.publish(True)
            self.menu.alignmentToolChecked = True

        self.menu_handler.reApply( self.server )
        self.server.applyChanges()



    def gripperTeleopToolCB(self, feedback):
        rospy.logwarn("This doesn't work yet!")



    def robotPlacementCB(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if state == MenuHandler.CHECKED:
            print "Turning Off Placement Tool"
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self.robot_placement_pub.publish(False)
            self.menu.robotPlacementChecked = False
        else:
            print "Turning On Placement Tool"
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self.robot_placement_pub.publish(True)
            self.menu.robotPlacementChecked = True

        self.menu_handler.reApply( self.server )
        self.server.applyChanges()



    def alignment_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            pass
        elif (event_type == feedback.MENU_SELECT):
            pass
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))




    def monitorMsgsCB(self, data):

        if data.level == 4:
            self.menu.numWarnings += 1

        elif data.level == 8:
            self.menu.numErrors += 1

        elif data.level == 16:
            self.menu.numFatal += 1

        self.populate_menu()


################################
#   Stuff For Making a Valve   #
################################



    def make_alignment_imarker(self, marker_pose):
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = marker_pose.pose
        new_marker.scale = 1.0
        new_marker.name = 'control_marker'

        # Make the default control for the marker itself
        display_marker = self.make_valve_marker(marker_pose)

        # Make the menu control
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.markers.append(display_marker)
        new_marker.controls.append(new_control)
        return new_marker




    def make_valve_marker(self, marker_pose):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = "control_marker"
        marker.id = 1

        #Give the marker a type
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://gui_robot_control/meshes/SimsIcon.dae"

        pose_offset = Pose()
        pose_offset.position.z = 0
        q1 = quaternion_about_axis((2*math.pi / 60) * self.status.rotate, (0,0,1))
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]

        rotated_pose = ComposePoses(marker_pose.pose, pose_offset)
        marker.pose = rotated_pose

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2

        #Set the color -- be sure to set alpha to something non-zero!


        if self.menu.flash > 30:
            diff = 60 - self.menu.flash
            red = 0.7 * (diff / 30.0)
        else:
            red = 0.7 * (self.menu.flash / 30.0)

        marker.color.r = 0.0
        marker.color.b = 0.0
        marker.color.g = 0.7

        if self.menu.numWarnings != 0:
            marker.color.r = red
            marker.color.b = 0.0
            marker.color.g = .7
        if self.menu.numErrors != 0:
            marker.color.r = 1.0
            marker.color.b = 0.0
            marker.color.g = 0.5
        if self.menu.numFatal != 0:
            marker.color.r = .70
            marker.color.b = 0.0
            marker.color.g = 0.0

        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1)
        return marker





if __name__ == '__main__':
    rospy.init_node("control_crystal")
    pointcloud_request = rospy.get_param("~rgbd_pointcloud_request_service", "")
    print "Pointcloud request service: " + pointcloud_request
    if (pointcloud_request == ""):
        pointcloud_request = None

    laser_request = rospy.get_param("~lidar_pointcloud_request_service", "")
    print "Pointcloud request service: " + laser_request
    if (laser_request == ""):
        laser_request = None

    head_control_action = rospy.get_param("~head_pointing_action", "")
    print "Head pointing action: " + head_control_action
    if (head_control_action == ""):
        head_control_action = None

    Control_Marker(pointcloud_request, laser_request, head_control_action)
