#Copyright (c) 2013,  Nicholas Alunni - Worcester Polytechnic Institute Darpa 
#Robotics Challenge (DRC) Team
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, 
#are permitted provided that the following conditions are met:
#
#- Redistributions of source code must retain the above copyright notice, this 
#  list of conditions and the following disclaimer.
#
#- Redistributions in binary form must reproduce the above copyright notice, this 
#  list of conditions and the following disclaimer in the documentation and/or 
#  other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
#SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# localization_marker_server/set message.
# Author: Nicholas Alunni
#
# The set mesasge is used for setting all of the relevent parameters for the localization marker server.
# All of the parameters that exist on the ROS parameter server are represented here and can be set and
# changed at runtime.

Header header

############# Marker #############
# These options all control the look of the marker that the user moves on the screen

# The name of the marker
string name

# Short ( < 40 char ) description
string description

# The type of marker to visualize
# The options are:
#   DISK
#   QUAD
#   MESH
string marker_type

# If you are using a mesh, the location of the mesh to sue
# Example:
# package://wpi_drc_sim/meshes/valve_models/logitech_driving_force_pro_wheel.dae
string mesh_resource

# The default pose of the marker to display
geometry_msgs/Pose defaultPose

# The size of the marker, there are different fields depending on the type of marker specified
# in the "string marker_type" field above.
# DISK:
float64 radius

# QUAD:
float64 width
float64 height

# MESH:
float64 scale

#The color and alpha of the marker
std_msgs/ColorRGBA marker_color


############# Bounding Volume #############
# These options control the bounding volume that is shown surrounding the marker that the user can move in space.
# The bounding volume can be turned off, display as a sphere, or display as a rectangle.  Only parts of the target
# cloud that are within the bounding volume will be included when running Iterative Closest Point.

# Whether or not to show the bounding box in the scene
bool show_bounding_volume

# The type of bounding box to display
# The options are:
#   SPHERE
#   RECTANGLE
string bounding_volume_type

# The size of the bounding volume
# The are multiple options depending on whether or not the bounding volume
# SPHERE:
float64 sphere_radius

# RECTANGLE:
geometry_msgs/Vector3 rectangle_scale

# The color and alpha of the bounding volume
std_msgs/ColorRGBA bounding_volume_color

############# Marker Control #############
# These options turn on and off the abilit to control the interactive marker for translation and rotation along different axis.
# Translation and Rotation can be turned on and off for the X, Y, and Z axis and although undesirable, the ability to turn on
# and off the message can be controlled.  This may be useful if there are times at which you wish to programatically control
# whether or not the user can perform certain actions.

# Translation Control
bool translate_X
bool translate_Y
bool translate_Z

# Rotation Control
bool rotate_X
bool rotate_Y
bool rotate_Z

# Menu Control
bool menu_control

############# Target Cloud #############
# The target cloud is the cloud that is being align to.  It is typically the point cloud that is generated from the robot
string target_cloud_topic

# Whether or not to voxel the incoming cloud that will be aligned to
bool target_cloud_voxel

############# Input Cloud #############
# The input cloud is the cloud that represents the object that is trying to be located in the world.
# If a MESH is being used for the display type, a custom pointcloud must be specified in the resource section.
# If a PLANE or CIRCLE is being used then a level of discretization must be specified for the cloud
string input_cloud_resource
int32 input_cloud_discretization

############# Iterative Closest Point (ICP) #############
# ICP is the algorithm that is used to align the two clouds
# A max coorespondance distace (how far away it will look) and a max number of iterations that will be run must be specified.
# The larger the distance and iterations to run the longer the algorithm will take, but the better the solution
float64 icp_coor_distance
int32 icp_iterations

############# Debug Stuff #############
# There are a few debug parameters that can make error detection easier.
# What currently exists is the ability to display the input cloud before and after ICP has been run in order to ensure that the cloud
# is generated at the correct location originally and that the maker aligns to the correct position and orientation after ICP has
# been run.  For each of these a topic must be specified and whether or not to display them must be given
bool pre_ICP_show
bool post_ICP_show
string pre_ICP_topic
string post_ICP_topic
