/*
Copyright (c) 2013,  Nicholas Alunni - Worcester Polytechnic Institute Darpa 
Robotics Challenge (DRC) Team
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this 
  list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice, this 
  list of conditions and the following disclaimer in the documentation and/or 
  other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/** TODO LIST:

  [ ] - Remove paramters that have been removed form the launch file
  [ ] - Make sure all parameters have a default value and check if the paramter
        exists before attemtping to call it. This is for safety.
  [ ] - Remove the hasChanged paramter and instead re-write it to use a good old
        fashioned ROS Topic of Service. Doing it with paramters is the wrong way
        unless I want to transition to using the dynamic parameter service,
        and aint nobody got time for that. ~NA
  [ ] - Add a paramter to change the base frame that the operations are
        performed in.
  [ ] - Create an RVIZ Panel plugin to control the functionality of this
        localization server.
  [ ] - Make GenerateTargetCloud use a default shape if none is otherwise specified

  --------------- Desirables: ---------------

  [ ] - Figure out if the parameters I've created are local are global, and if
        they are global change them to be local so that cannot be seen
        elsewhere or tampered with.
  [ ] - Create functions for different parts of the code so that it is easier
        to extend in the future, such as a different function for each type or
        marker.
  [ ] - Re-write into a C++ class, lets be honest it would make everything
        cleaner and safer.


 **/



#include <ros/ros.h>
#include <math.h>
#include <string>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "localization_markers/set_localization_marker.h"

// PCL specific includes
#include <iostream>
#include <fstream>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

/** DEFAULT PARAMTER VALUES **/
#define DEFAULT_ICP_CORR .25
#define DEFAULT_ICP_ITERATIONS 500
#define DEFAULT_POST_ICP_DEBUG false
#define DEFAULT_BOUNDING_BOX_TYPE "CUBE"
#define DEFAULT_BOUNDING_BOX_SIZE .3
#define DEFAULT_BOUNDING_BOX_COLOR "BLUE"
#define DEFAULT_MARKER_TYPE "VALVE"
#define DEFAULT_VALVE_RESOURCE "package://localization_markers/resources/torus.dae"
#define DEFAULT_VALVE_SIZE .25
#define DEFAULT_VALVE_COLOR "RED"
#define DEFAULT_CONTROL_TYPE "6DOF"
#define DEFAULT_VOXEL true

using namespace visualization_msgs;
using namespace interactive_markers;

void makeInteractiveMarker(geometry_msgs::Pose newPose);
void generateInputCloud(void);
void generateTargetCloud(void);
void runICP(void);
void set_cb(localization_markers::set_localization_marker input);

interactive_markers::MenuHandler menu_handler;
interactive_markers::InteractiveMarkerServer * server = NULL;

geometry_msgs::Pose defaultPose;
geometry_msgs::Pose currentPose;
sensor_msgs::PointCloud2 recentCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 ros_input_cloud;
sensor_msgs::PointCloud2 ros_target_cloud;
Eigen::Matrix4f homogeneousMatrix;
std::string frame_id = "/base_link";
std::string marker_name;
geometry_msgs::PoseStamped objectPose;

ros::Publisher object_pose;
ros::Publisher debug_input_cloud_pre_ICP_pub;
ros::Publisher debug_input_cloud_post_ICP_pub;

enum shape_type {MESH, QUAD, DISK};
shape_type currentMarkerType = QUAD;

bool hasChanged = false;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

    currentPose = feedback->pose;

	switch (feedback->event_type){
		
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:

            //Set the position to where the valve current is

            switch(feedback->menu_entry_id){

            //RUN ICP
            case 1:

                pcl_input_cloud->clear();
                pcl_target_cloud->clear();

                generateTargetCloud();
                generateInputCloud();
                runICP();
                server->erase(marker_name);
                makeInteractiveMarker(currentPose);
                break;

            //Publish the position of the object
            case 2:

                objectPose.header.frame_id = frame_id;
                objectPose.header.stamp = ros::Time::now();
                objectPose.pose = currentPose;

                object_pose.publish(objectPose);
                break;

            //Refresh the server
            case 3:
                server->erase(marker_name);
                makeInteractiveMarker(feedback->pose);
                break;

            //Reset the server
            case 4:
                server->erase(marker_name);
                makeInteractiveMarker(defaultPose);
                break;

      } //END switch(feedback->menu_entry_id){
			
			break;
	} //END switch (feedback->event_type){

  server->applyChanges();
}

/** RUN Iterative Closest Point (ICP) on the input and target clouds

  This function will attempt to RUN ICP on the two clouds that have been "specified" by the user.
  The TARGET cloud is the one that is seen by the robot
  The INPUT cloud is the one that is generated by the marker's position

**/
void runICP(void){

    //The cloud that will be used to align the two other clouds coming from RVIZ
    pcl::PointCloud<pcl::PointXYZ> Final;

    /** Setup the max coorespondance distance for the ICP. This is the max distance that it
        will look for a match within the two clouds **/
    double corr;
    if (ros::param::has("/localization_marker_server/icp/maxCoorespondanceDistance")) {
        ros::param::get("/localization_marker_server/icp/maxCoorespondanceDistance", corr);
    } else corr = DEFAULT_ICP_CORR;

    /** Setup the number of iterations that ICP will run. The higher this number the larger
        distance it can find a match, but the slower it will perform. **/
    int iterations;
    if (ros::param::has("/localization_marker_server/icp/maxInterations")) {
        ros::param::get("/localization_marker_server/icp/maxInterations", iterations);
    } else iterations = DEFAULT_ICP_ITERATIONS;

    //Create a ICP object, set parameters, give it input and target
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance (corr);
    icp.setMaximumIterations (iterations);
    icp.setInputCloud(pcl_input_cloud);
    icp.setInputTarget(pcl_target_cloud);

    //Align the two clouds
    icp.align(Final);

    //Save the matrix from the transformation
    Eigen::Matrix4f myTransform = icp.getFinalTransformation();
    Eigen::Matrix4f finalPosition = myTransform*homogeneousMatrix;

    tf::Matrix3x3 myTfMatrix(myTransform(0,0), myTransform(0,1), myTransform(0,2),
                             myTransform(1,0), myTransform(1,1), myTransform(1,2),
                             myTransform(2,0), myTransform(2,1), myTransform(2,2));

    //Create the update to the valve
    currentPose.position.x = finalPosition(0, 3);
    currentPose.position.y = finalPosition(1, 3);
    currentPose.position.z = finalPosition(2, 3);

    //Turn the rotation matrix into a Quaternion
    tfScalar yaw, pitch, roll;
    myTfMatrix.getEulerYPR(yaw, pitch, roll);

    //Multiply the Quaternions to get my new Orientation
    tf::Quaternion currentQuaternion(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w);
    tf::Quaternion myTfQuaternion(pitch, roll, yaw);
    currentQuaternion*=myTfQuaternion;

    tf::quaternionTFToMsg(currentQuaternion, currentPose.orientation);

    /** Determined whether or not to show a point cloud of after it has been aligned. This
        can be useful for determining errors between different frames **/
    bool debug;
    if (ros::param::has("/localization_marker_server/debug/input_cloud/post_ICP/show")) {
        ros::param::get("/localization_marker_server/debug/input_cloud/post_ICP/show", debug);
    } else debug = DEFAULT_POST_ICP_DEBUG;

    //If the debug was set to true, then publish the post cloud to RVIZ
    if (debug) {

        //Do something to copy over output cloud here
        pcl::transformPointCloud(*pcl_input_cloud, *pcl_input_cloud, icp.getFinalTransformation());

        pcl::toROSMsg(*pcl_input_cloud, ros_input_cloud);
        ros_input_cloud.header.frame_id = frame_id;
        debug_input_cloud_post_ICP_pub.publish(ros_input_cloud);

    }

}

/** Generate the Target Cloud that will be used by ICP

  This function generates one of the point clouds that will be used by ICP.
  The Target cloud is a subset of the point cloud that is seen by the robot in the world

**/
void generateTargetCloud(void){

    pcl::PointCloud<pcl::PointXYZ> pcl_recent_cloud;
    pcl::PointXYZ point;

    //Convert the recent cloud into a pcl cloud that can be used
    pcl::fromROSMsg(recentCloud, pcl_recent_cloud);

    /** Check what type of bounding box the user wants to use to get a subset of the cloud **/
    std::string box_type;
    if (ros::param::has("/localization_marker_server/bounding_box/type")) {
        ros::param::get("/localization_marker_server/bounding_box/type", box_type);
    } else box_type = DEFAULT_BOUNDING_BOX_TYPE;

    if (box_type.compare("SPHERE") == 0) {

        /** Check that the user has specified a size for the bounding box **/
        double radius;
        if (ros::param::has("/localization_marker_server/bounding_box/size")) {
            ros::param::get("/localization_marker_server/bounding_box/size", radius);
        } radius = DEFAULT_BOUNDING_BOX_SIZE;

        for (unsigned int i = 0; i < pcl_recent_cloud.size(); i++){

            if(sqrt(pow(currentPose.position.x - pcl_recent_cloud.points[i].x, 2) +
                    pow(currentPose.position.y - pcl_recent_cloud.points[i].y, 2) +
                    pow(currentPose.position.z - pcl_recent_cloud.points[i].z, 2)) <= radius ){

                point.x = pcl_recent_cloud.points[i].x;
                point.y = pcl_recent_cloud.points[i].y;
                point.z = pcl_recent_cloud.points[i].z;

                pcl_target_cloud->push_back(point);
            }

        }

    } else if (box_type.compare("CUBE") == 0) {

        /** Check that the user has specified a size for the bounding box **/
        double size;
        if (ros::param::has("/localization_marker_server/bounding_box/size")) {
            ros::param::get("/localization_marker_server/bounding_box/size", size);
        } size = DEFAULT_BOUNDING_BOX_SIZE;

        for (unsigned int i = 0; i < pcl_recent_cloud.size(); i++){

            if (pcl_recent_cloud.points[i].x > currentPose.position.x - size/2 &&
                pcl_recent_cloud.points[i].x < currentPose.position.x + size/2 &&
                pcl_recent_cloud.points[i].y > currentPose.position.y - size/2 &&
                pcl_recent_cloud.points[i].y < currentPose.position.y + size/2 &&
                pcl_recent_cloud.points[i].z > currentPose.position.z - size/2 &&
                pcl_recent_cloud.points[i].z < currentPose.position.z + size/2 ){

                point.x = pcl_recent_cloud.points[i].x;
                point.y = pcl_recent_cloud.points[i].y;
                point.z = pcl_recent_cloud.points[i].z;

                pcl_target_cloud->push_back(point);

            } else {
            }

        }

    } else {
        ROS_WARN("Localization Marker Server: Bounding Box Type not Recognized!");
        ROS_WARN("Eventually this will choose a default shape, but not now!");
    }
}

/** Generate the Input Cloud that will be used by ICP

  This function generates one of the point clouds that will be used by ICP.
  The Input cloud is the cloud that is generated around the marker

**/
void generateInputCloud(void){

    pcl::PointXYZ point;
    bool debug;
    int discrete;
    ros::param::get("/localization_marker_server/input_cloud/discretization", discrete);
    std::string input_cloud_resource;

    switch(currentMarkerType){

        case MESH:

            ros::param::get("/localization_marker_server/input_cloud/resource", input_cloud_resource);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_cloud_resource.c_str(), *pcl_input_cloud) == -1)
            {
              ROS_ERROR ("Couldn't read file test_pcd.pcd \n");
            }

            break;
        case QUAD:


        double w, h;
        ros::param::get("/localization_marker_server/marker/width", w);
        ros::param::get("/localization_marker_server/marker/height", h);

        for (int width = (w * discrete) * -.5; width <= (w * discrete) * .5; width++){
                for (int height = (h * discrete) * -.5; height <= (h * discrete) * .5; height++){
                    point.x = (float)width / discrete;
                    point.y = (float)height / discrete;
                    point.z = 0;

                    pcl_input_cloud->push_back(point);
                }
            }

            break;

        case DISK:

            double r;
            ros::param::get("/localization_marker_server/marker/radius", r);

            for (int angle = 0; angle <= 2*3.14159*discrete; angle++){
                for (int radius = 0; radius <= (r * discrete) / 2; radius++){
                    point.x = ((float)radius / discrete) * cos((float)angle / discrete);
                    point.y = ((float)radius / discrete) * sin((float)angle / discrete);
                    point.z = 0;

                    pcl_input_cloud->push_back(point);
                }
            }

            break;

    }

    //Transform the cloud to where the interactive marker is located
    //==============================================================

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(currentPose.orientation, orientation);

    //This is so fucking assinine
    tf::Matrix3x3 rotationMatrix(orientation);

    homogeneousMatrix(0,0) = rotationMatrix[0][0];
    homogeneousMatrix(0,1) = rotationMatrix[0][1];
    homogeneousMatrix(0,2) = rotationMatrix[0][2];
    homogeneousMatrix(0,3) = currentPose.position.x;

    homogeneousMatrix(1,0) = rotationMatrix[1][0];
    homogeneousMatrix(1,1) = rotationMatrix[1][1];
    homogeneousMatrix(1,2) = rotationMatrix[1][2];
    homogeneousMatrix(1,3) = currentPose.position.y;

    homogeneousMatrix(2,0) = rotationMatrix[2][0];
    homogeneousMatrix(2,1) = rotationMatrix[2][1];
    homogeneousMatrix(2,2) = rotationMatrix[2][2];
    homogeneousMatrix(2,3) = currentPose.position.z;

    homogeneousMatrix(3,0) = 0;
    homogeneousMatrix(3,1) = 0;
    homogeneousMatrix(3,2) = 0;
    homogeneousMatrix(3,3) = 1;

    pcl::transformPointCloud(*pcl_input_cloud, *pcl_input_cloud, homogeneousMatrix);

    //==============================================================

    ros::param::get("/localization_marker_server/debug/input_cloud/pre_ICP/show", debug);
    if (debug) {
        pcl::toROSMsg(*pcl_input_cloud, ros_input_cloud);
        ros_input_cloud.header.frame_id = frame_id;
        debug_input_cloud_pre_ICP_pub.publish(ros_input_cloud);
    }


}

////////////////////////////////////////////////////////////////////////////////////

/** Make the interactive marker that is seen in RVIZ
**/
void makeInteractiveMarker(geometry_msgs::Pose newPose){

// Variables needed for creating the marker

    Marker marker;
    InteractiveMarker int_marker;
    InteractiveMarkerControl control_box, control_menu, control_pose;

    bool use_control;
    double c;


// ********* Marker ********* //

    /** Get what type of marker should be used from the paramter server **/
    std::string type;
    if (ros::param::has("/localization_marker_server/marker/type")) {
        ros::param::get("/localization_marker_server/marker/type", type);
    } else type = DEFAULT_MARKER_TYPE;

    if (type.compare("VALVE") == 0){

        currentMarkerType = MESH;
        marker.type = Marker::MESH_RESOURCE;

        if (ros::param::has("/localization_marker_server/marker/mesh_resource")) {
            ros::param::get("/localization_marker_server/marker/mesh_resource", marker.mesh_resource);
        } else marker.mesh_resource = DEFAULT_VALVE_RESOURCE;

        if (ros::param::has("/localization_marker_server/marker/size_meters")) {
            ros::param::get("/localization_marker_server/marker/size_meters", marker.scale.x);
        } else marker.scale.x = DEFAULT_VALVE_SIZE;

        marker.scale.y = marker.scale.z = marker.scale.x;

    } else {
        ROS_WARN("Localization Marker Server: Marker Type Not Recognized");
        ROS_WARN("Please define an accepted type of marker!");

    }

    std::string color;
    if (ros::param::has("/localization_marker_server/marker/color")) {
        ros::param::get("/localization_marker_server/marker/color", color);
    } else color = DEFAULT_VALVE_COLOR;

    if (color.compare("RED") == 0) {
        marker.color.r = marker.color.a = 1;
        marker.color.g = marker.color.b = 0;
    }


  // ****** Create the Interactive Marker ****** //
    int_marker.name = "Localization Marker";
    int_marker.description = "Align Me!";
    int_marker.header.frame_id = frame_id;
    marker_name = int_marker.name;

    int_marker.pose.position.x = newPose.position.x;
    int_marker.pose.position.y = newPose.position.y;
    int_marker.pose.position.z = newPose.position.z;
    int_marker.pose.orientation.x = newPose.orientation.x;
    int_marker.pose.orientation.y = newPose.orientation.y;
    int_marker.pose.orientation.z = newPose.orientation.z;
    int_marker.pose.orientation.w = newPose.orientation.w;


    // Create the bounding box seen around the marker

    Marker boundingBox;

    /** Get the type of the box off of the paramter server **/
    std::string box_type;
    if (ros::param::has("/localization_marker_server/bounding_box/type")) {
        ros::param::get("/localization_marker_server/bounding_box/type", box_type);
    } else box_type = DEFAULT_BOUNDING_BOX_TYPE;

    if (box_type.compare("SPHERE") == 0) {
        boundingBox.type = Marker::SPHERE;

        if(ros::param::has("/localization_marker_server/bounding_box/size")) {
            ros::param::get("/localization_marker_server/bounding_box/size", boundingBox.scale.x);
        } else boundingBox.scale.x = DEFAULT_BOUNDING_BOX_SIZE;

        boundingBox.scale.y = boundingBox.scale.z = boundingBox.scale.x;

        boundingBox.scale.x *= 2;
        boundingBox.scale.y *= 2;
        boundingBox.scale.z *= 2;

    } else if (box_type.compare("CUBE") == 0) {
        boundingBox.type = Marker::CUBE;

        if(ros::param::has("/localization_marker_server/bounding_box/size")) {
            ros::param::get("/localization_marker_server/bounding_box/size", boundingBox.scale.x);
        } else boundingBox.scale.x = DEFAULT_BOUNDING_BOX_SIZE;

        boundingBox.scale.y = boundingBox.scale.z = boundingBox.scale.x;


    } else {
        ROS_WARN("Localization Marker Server: Bounding Box Type not Recognized!");
        ROS_WARN("Please define a shape!"); }

    if (ros::param::has("/localization_marker_server/marker/color")) {
        ros::param::get("/localization_marker_server/marker/color", color);
    } else color = DEFAULT_BOUNDING_BOX_COLOR;

    if (color.compare("BLUE") == 0) {
        boundingBox.color.b = 1;
        boundingBox.color.g = boundingBox.color.r = 0;
        boundingBox.color.a = .3;
    }

    InteractiveMarkerControl boundingBoxControl;
    boundingBoxControl.always_visible = true;
    boundingBoxControl.orientation_mode = InteractiveMarkerControl::FIXED;
    boundingBoxControl.markers.push_back(boundingBox);
    int_marker.controls.push_back(boundingBoxControl);

// ********* The Model to Control ********* //  
    control_box.always_visible = true;
    control_box.markers.push_back( marker );
    int_marker.controls.push_back( control_box );
    int_marker.controls.back();



// ********* The Menu Controller ********* //  

    control_menu.interaction_mode = InteractiveMarkerControl::MENU;
    int_marker.controls.push_back(control_menu);


// ********* The Pose Controls ********* // 
    control_pose.orientation_mode = InteractiveMarkerControl::INHERIT;

    /** Get the type of desired control from the user **/
    std::string control;
    if (ros::param::has("/localization_marker_server/control")) {
        ros::param::get("/localization_marker_server/control", control);
    } else control = DEFAULT_CONTROL_TYPE;

    if (control.compare("6DOF") || control.compare("TRANS")) {

        control_pose.name = "translate_x";
        control_pose.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 1;
        control_pose.orientation.y = 0;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);

        control_pose.name = "translate_y";
        control_pose.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 0;
        control_pose.orientation.y = 1;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);

        control_pose.name = "translate_z";
        control_pose.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 0;
        control_pose.orientation.y = 0;
        control_pose.orientation.z = 1;
        int_marker.controls.push_back(control_pose);
    }

    if (control.compare("6DOF") || control.compare("ROT")) {

        control_pose.name = "rotate_x";
        control_pose.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 1;
        control_pose.orientation.y = 0;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);

        control_pose.name = "rotate_y";
        control_pose.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 0;
        control_pose.orientation.y = 1;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);

        control_pose.name = "rotate_z";
        control_pose.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 0;
        control_pose.orientation.y = 0;
        control_pose.orientation.z = 1;
        int_marker.controls.push_back(control_pose);
    }


// ********* Update The Server ********* //  
	//"Publish" the marker to the server
  server->insert(int_marker, &processFeedback);
  menu_handler.apply( *server, int_marker.name );

}

/** Get the most recent cloud that has been seen by the robot and save it
**/
void pc_target_cb(sensor_msgs::PointCloud2::Ptr input){

    /** Determine whether or not to voxel the cloud **/
    bool use_voxel;
    if (ros::param::has("/localization_marker_server/target_cloud/voxel")) {
        ros::param::get("/localization_marker_server/target_cloud/voxel", use_voxel);
    } else use_voxel = DEFAULT_VOXEL;

    if (use_voxel){
        pcl::VoxelGrid<sensor_msgs::PointCloud2> filter;
        filter.setInputCloud (input);
        filter.setLeafSize (0.01f, 0.01f, 0.01f);
        filter.filter (recentCloud);
    } else {
        recentCloud = *input;
    }

    if (frame_id.compare(input->header.frame_id.c_str()) != 0){

        if (currentPose.position.x != defaultPose.position.x ||
            currentPose.position.y != defaultPose.position.y ||
            currentPose.position.z != defaultPose.position.z) {
            ROS_WARN("Localization_Marker_Server: Frame_ID has Changed! Reseting Marker Position"); }

        frame_id = input->header.frame_id;
        makeInteractiveMarker(currentPose);
        server->applyChanges();
    }

}

/** Callback to change the state of the localization_marker_server
  **/
void set_cb(localization_markers::set_localization_marker input){

    if (ros::param::has("/localization_marker_server/hasChanged")) {
        ros::param::set("/localization_marker_server/hasChanged", true);
    } else ROS_WARN("CRITICAL SYSTEM ERROR, hasChanged DOES NOT EXIST!!!");

}

/** A loop that runs to check if any updates have been received
**/
void check(void){

    if (ros::param::has("/localization_marker_server/hasChanged")) {
        ros::param::get("/localization_marker_server/hasChanged", hasChanged);
    } else hasChanged = true;

    if (hasChanged){
        server->erase(marker_name);
        makeInteractiveMarker(currentPose);
        server->applyChanges();
        if (ros::param::has("/localization_marker_server/hasChanged")) {
            ros::param::set("/localization_marker_server/hasChanged", false);
        } else hasChanged = false;
    }

}

// %Tag(main)%
int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_marker_server_drc");
    ros::NodeHandle n;

    std::string pc_topic;
    ros::param::get("/localization_marker_server/target_cloud/topic", pc_topic);
    ros::Subscriber pc_target_sub = n.subscribe(pc_topic.c_str(),1, pc_target_cb);
    ros::Subscriber localization_marker_set = n.subscribe("/localization_marker_server/set", 1, set_cb);
    object_pose = n.advertise<geometry_msgs::PoseStamped>("/localization_marker_server/object/pose", 1);

    bool debug_pre_ICP;
    if (ros::param::has("/localization_marker_server/debug/input_cloud/pre_ICP/show")){
        ros::param::get("/localization_marker_server/debug/input_cloud/pre_ICP/show", debug_pre_ICP);
    } else debug_pre_ICP = false;

    if (debug_pre_ICP){
        std::string input_topic;
        if (ros::param::has("/localization_marker_server/debug/input_cloud/pre_ICP/topic")) {
            ros::param::get("/localization_marker_server/debug/input_cloud/pre_ICP/topic", input_topic);
        } else input_topic = "/localization_marker_server/debug/input_cloud/pre_icp";
        debug_input_cloud_pre_ICP_pub = n.advertise<sensor_msgs::PointCloud2>(input_topic.c_str(), 1);
    }

    bool debug_post_ICP;
    if (ros::param::has("/localization_marker_server/debug/input_cloud/post_ICP/show")){
        ros::param::get("/localization_marker_server/debug/input_cloud/post_ICP/show", debug_post_ICP);
    } debug_post_ICP = false;

    if (debug_post_ICP){
        std::string input_topic;
        if (ros::param::has("/localization_marker_server/debug/input_cloud/pre_ICP/topic")) {
            ros::param::get("/localization_marker_server/debug/input_cloud/post_ICP/topic", input_topic);
        } input_topic = "/localization_marker_server/debug/input_cloud/post_icp";
        debug_input_cloud_post_ICP_pub = n.advertise<sensor_msgs::PointCloud2>(input_topic.c_str(), 1);
    }

    server = new interactive_markers::InteractiveMarkerServer("localization_marker_server","",false);
    ros::Duration(.1).sleep();

    //Setup the menu options, this may change location
    menu_handler.insert( "Align", &processFeedback );
    menu_handler.insert( "Publish", &processFeedback );
    menu_handler.insert( "Refresh", &processFeedback);
    menu_handler.insert( "Reset", &processFeedback);


    //Set the default pose of the marker
    defaultPose.position.x = 0;
    defaultPose.position.y = 0;
    defaultPose.position.z = 0;
    defaultPose.orientation.w = .5;
    defaultPose.orientation.x = .5;
    defaultPose.orientation.y = -.5;
    defaultPose.orientation.z = .5;
    currentPose = defaultPose;

    if (ros::param::has("/localization_marker_server/hasChanged")) {
        ros::param::set("/localization_marker_server/hasChanged", true);
    } else hasChanged = true;

    ros::Rate r(20);

    while(ros::ok()){

        check();

        ros::spinOnce();

        r.sleep();
    }

    ros::spin();
}
// %EndTag(main)%
