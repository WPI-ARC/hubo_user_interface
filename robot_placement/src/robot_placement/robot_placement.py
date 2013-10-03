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



class MarkerStatus:



    def __init__(self):
        self.default_radius = 0.1
        self.default_thickness = 0.02
        self.radius = self.default_radius
        self.default_pose_stamped = PoseStamped()
        self.default_pose_stamped.header.frame_id = "/Body_TSY"
        self.default_pose_stamped.pose.position.x = 1.0
        self.default_pose_stamped.pose.position.y = 0.0
        self.default_pose_stamped.pose.position.z = 0.0
        self.default_pose_stamped.pose.orientation.x = 0.0
        self.default_pose_stamped.pose.orientation.y = 0.0
        self.default_pose_stamped.pose.orientation.z = 0.0
        self.default_pose_stamped.pose.orientation.w = 1.0
        self.session_pose_stamped = deepcopy(self.default_pose_stamped)
        self.pose_stamped = deepcopy(self.default_pose_stamped)
        self.visible = True





class Placement_Marker:

    def __init__(self):

        rospy.Subscriber("robot_placement/show", Bool, self.show_tool)

        #Setup The Valve and Valve Marker
        self.status = MarkerStatus()
        self.menu_handler = MenuHandler()
        self.populate_menu()

        #Setup the interactive marker server
        self.server = InteractiveMarkerServer("placement_marker")


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

        if self.status.visible == True:
            alignment_marker = self.make_alignment_imarker(self.status.pose_stamped)
            self.server.insert(alignment_marker, self.alignment_feedback_cb)
            self.menu_handler.apply(self.server, "placement_marker")

        self.server.applyChanges()




    def show_tool(self, data):
        self.status.visible = data.data




##################################
#   Valve Menu Stuff             #
##################################


    def populate_menu(self):
        self.menu_handler.insert("Send Robot Here")



    def alignment_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            self.status.pose_stamped.pose = feedback.pose
        elif (event_type == feedback.MENU_SELECT):
            rospy.loginfo("Menu feedback selection: " + self.options[feedback.menu_entry_id - 1])
            #self.process_menu_select(feedback.menu_entry_id)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))



################################
#   Stuff For Making a Valve   #
################################



    def make_alignment_imarker(self, marker_pose):
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = marker_pose.pose
        new_marker.scale = 1.0
        new_marker.name = 'placement_marker'


        display_marker_left = self.make_valve_marker(marker_pose, -.2)
        display_marker_right = self.make_valve_marker(marker_pose, +.2)

        # Make the menu control
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.markers.append(display_marker_left)
        new_control.markers.append(display_marker_right)
        new_marker.controls.append(new_control)

        # Make the x-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_x"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 1.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)

        # Make the x-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_z"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        new_marker.controls.append(new_control)

        # Make the z-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "rotate_y"
        new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 1.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)

        return new_marker




    def make_valve_marker(self, marker_pose, y_offset):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = "placement_marker"
        marker.id = 1

        #Give the marker a type
        marker.type = Marker.CUBE

        pose_offset = Pose()
        pose_offset.position.y = y_offset
        rotated_pose = ComposePoses(marker_pose.pose, pose_offset)
        marker.pose = rotated_pose

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = .25
        marker.scale.y = .20
        marker.scale.z = .05

        #Set the color -- be sure to set alpha to something non-zero!

        marker.color.r = 0.0
        marker.color.b = 0.7
        marker.color.g = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1)
        return marker





if __name__ == '__main__':
    rospy.init_node("robot_placement")
    Placement_Marker()
