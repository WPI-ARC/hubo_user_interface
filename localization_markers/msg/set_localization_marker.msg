std_msgs/Header header

string name
string description

# The type of marker to visualize
uint8 MESH=0
uint8 DISK=1
uint8 QUAD=2
uint8 marker_type

# If you are using a mesh, the location of the mesh to use
string mesh_resource

# The default pose of the marker to display
geometry_msgs/PoseStamped defaultPose

# For marker_type == DISK:
float64 radius

# For marker_type == QUAD:
float64 width
float64 height

# For marker_type == MESH:
float64 scale

# Display color of the marker
std_msgs/ColorRGBA marker_color

# Whether or not to show the bounding box in the scene
bool show_bounding_volume

# The type of bounding box to display
uint8 SPHERE=0
uint8 BOX=1
uint8 bounding_volume_type

# The size of the bounding volume
# For bounding_volume_type == SPHERE:
float64 sphere_radius

# For bounding_volume_type == RECTANGLE:
geometry_msgs/Vector3 bounding_volume_dimensions

# The color and alpha of the bounding volume
std_msgs/ColorRGBA bounding_volume_color

############# Marker Control #############
# Translation Control
bool enable_translate_X
bool enable_translate_Y
bool enable_translate_Z

# Rotation Control
bool enable_rotate_X
bool enable_rotate_Y
bool enable_rotate_Z

# Menu Control
bool enable_menu_control

############# Target Cloud #############
# The target cloud is the cloud that is being align to.  It is typically the point cloud that is generated from the robot
string target_cloud_topic

# Whether or not to voxel the incoming cloud that will be aligned to
bool voxelize_target_cloud

############# Input Cloud #############
# The input cloud is the cloud that represents the object that is trying to be located in the world.
# For marker_type == MESH:
string input_cloud_resource

# For marker_type == DISK or marker_type == QUAD:
int32 input_cloud_discretization

############# Iterative Closest Point (ICP) #############
# ICP is the algorithm that is used to align the two clouds
float64 max_icp_coorespondence_distance
int32 max_icp_iterations

############# Debug Stuff #############
# There are a few debug parameters that can make error detection easier.
# What currently exists is the ability to display the input cloud before and after ICP has been run in order to ensure that the cloud
# is generated at the correct location originally and that the marker aligns to the correct position and orientation after ICP has
# been run.  For each of these a topic must be specified and whether or not to display them must be given
bool pre_ICP_show
bool post_ICP_show
string pre_ICP_topic
string post_ICP_topic
