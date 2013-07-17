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

#include "localization_marker_msgs/set_localization_marker.h"

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

using namespace visualization_msgs;
using namespace interactive_markers;

void makeValve(geometry_msgs::Pose newPose);
void generateInputCloud(void);
void generateTargetCloud(void);
void runICP(void);
void set_cb(localization_marker_msgs::set_localization_marker input);

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
                makeValve(currentPose);
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
                makeValve(feedback->pose);
                break;

            //Reset the server
            case 4:
                server->erase(marker_name);
                makeValve(defaultPose);
                break;

      } //END switch(feedback->menu_entry_id){
			
			break;
	} //END switch (feedback->event_type){

  server->applyChanges();
}

void runICP(void){

    pcl::PointCloud<pcl::PointXYZ> Final;

    bool debug;
    double corr;
    int interations;

    ros::param::get("/localization_marker_server/icp/maxCoorespondanceDistance", corr);
    ros::param::get("/localization_marker_server/icp/maxInterations", interations);

    //Create a ICP object, set parameters, give it input and target
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance (corr);
    icp.setMaximumIterations (interations);
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
    tf::Quaternion newQuaternion;
    myTfMatrix.getRotation(newQuaternion);

    //Compose the Quaternions to get my new Orientation
    tf::Quaternion currentQuaternion(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w);

    currentQuaternion = currentQuaternion * newQuaternion;

    tf::quaternionTFToMsg(currentQuaternion, currentPose.orientation);

    ros::param::get("/localization_marker_server/debug/input_cloud/post_ICP/show", debug);
    if (debug) {

        //Do something to copy over output cloud here
        pcl::transformPointCloud(*pcl_input_cloud, *pcl_input_cloud, icp.getFinalTransformation());

        pcl::toROSMsg(*pcl_input_cloud, ros_input_cloud);
        ros_input_cloud.header.frame_id = frame_id;
        debug_input_cloud_post_ICP_pub.publish(ros_input_cloud);

    }

}

void generateTargetCloud(void){


    pcl::PointCloud<pcl::PointXYZ> pcl_recent_cloud;
    pcl::PointXYZ point;

    //Convert the recent cloud into a pcl cloud that can be used
    pcl::fromROSMsg(recentCloud, pcl_recent_cloud);

    std::string box_type;
    ros::param::get("/localization_marker_server/bounding_box/type", box_type);
    if (box_type.compare("SPHERE") == 0) {

        double radius;
        ros::param::get("/localization_marker_server/bounding_box/radius", radius);

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

    } else if (box_type.compare("RECTANGLE") == 0) {

        double x, y,z;
        ros::param::get("/localization_marker_server/bounding_box/x", x);
        ros::param::get("/localization_marker_server/bounding_box/y", y);
        ros::param::get("/localization_marker_server/bounding_box/z", z);

        for (unsigned int i = 0; i < pcl_recent_cloud.size(); i++){

            if (pcl_recent_cloud.points[i].x > currentPose.position.x - x/2 &&
                pcl_recent_cloud.points[i].x < currentPose.position.x + x/2 &&
                pcl_recent_cloud.points[i].y > currentPose.position.y - y/2 &&
                pcl_recent_cloud.points[i].y < currentPose.position.y + y/2 &&
                pcl_recent_cloud.points[i].z > currentPose.position.z - z/2 &&
                pcl_recent_cloud.points[i].z < currentPose.position.z + z/2 ){

                point.x = pcl_recent_cloud.points[i].x;
                point.y = pcl_recent_cloud.points[i].y;
                point.z = pcl_recent_cloud.points[i].z;

                pcl_target_cloud->push_back(point);

            } else {
            }

        }

    } else {
        ROS_WARN("Localization Marker Server: Bounding Box Type not Recognized!");
        ROS_WARN("Default Type = SPHERE");

        double radius;
        ros::param::get("/localization_marker_server/bounding_box/radius", radius);

        //Convert the recent cloud into a pcl cloud that can be used
        pcl::fromROSMsg(recentCloud, pcl_recent_cloud);

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
    }
}

void generateInputCloud(void){

    pcl::PointXYZ point;
    bool debug;
    int discrete;
    ros::param::get("/localization_marker_server/input_cloud/discretization", discrete);
    std::string input_cloud_resource;

    switch(currentMarkerType){

        case MESH:

            ros::param::get("/localization_marker_server/input_cloud/resource", input_cloud_resource);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_cloud_resource.c_str(), *pcl_input_cloud) == -1) //* load the file
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

void makeValve(geometry_msgs::Pose newPose){

// Variables needed for creating the valve

    Marker marker;
    InteractiveMarker int_marker;
    InteractiveMarkerControl control_box, control_menu, control_pose;

    bool use_control;
    double c;
    std::string type;


// ********* Marker ********* //

    ros::param::get("/localization_marker_server/marker/type", type);
    if (type.compare("MESH") == 0){

        currentMarkerType = MESH;
        marker.type = Marker::MESH_RESOURCE;
        ros::param::get("/localization_marker_server/marker/mesh_resource", marker.mesh_resource);

        ros::param::get("/localization_marker_server/marker/scale", marker.scale.x);
        ros::param::get("/localization_marker_server/marker/scale", marker.scale.y);
        ros::param::get("/localization_marker_server/marker/scale", marker.scale.z);


    } else if (type.compare("QUAD") == 0){

        currentMarkerType = QUAD;
        marker.type = Marker::CUBE;
        ros::param::get("/localization_marker_server/marker/width", marker.scale.x);
        ros::param::get("/localization_marker_server/marker/height", marker.scale.y);
        marker.scale.z = .001;


    } else if (type.compare("DISK") == 0){

        currentMarkerType = DISK;
        marker.type = Marker::SPHERE;
        ros::param::get("/localization_marker_server/marker/radius", marker.scale.x);
        ros::param::get("/localization_marker_server/marker/radius", marker.scale.y);
        marker.scale.z = .001;


    } else {
        ROS_WARN("Localization Marker Server: Marker Type Not Recognized");
        ROS_WARN("Default Shape is QUAD");

        currentMarkerType = QUAD;
        marker.type = Marker::CUBE;
        ros::param::get("/localization_marker_server/marker/width", marker.scale.x);
        ros::param::get("/localization_marker_server/marker/height", marker.scale.y);
        marker.scale.z = .001;

    }

    ros::param::get("/localization_marker_server/marker/color/r", c); marker.color.r = c;
    ros::param::get("/localization_marker_server/marker/color/g", c); marker.color.g = c;
    ros::param::get("/localization_marker_server/marker/color/b", c); marker.color.b = c;
    ros::param::get("/localization_marker_server/marker/color/a", c); marker.color.a = c;


  // ****** Create the Interactive Marker ****** //
    ros::param::get("/localization_marker_server/marker/name", int_marker.name);
    ros::param::get("/localization_marker_server/marker/description", int_marker.description);
    int_marker.header.frame_id = frame_id;
    marker_name = int_marker.name;

    int_marker.pose.position.x = newPose.position.x;
    int_marker.pose.position.y = newPose.position.y;
    int_marker.pose.position.z = newPose.position.z;
    int_marker.pose.orientation.x = newPose.orientation.x;
    int_marker.pose.orientation.y = newPose.orientation.y;
    int_marker.pose.orientation.z = newPose.orientation.z;
    int_marker.pose.orientation.w = newPose.orientation.w;



    ros::param::get("/localization_marker_server/bounding_box/show", use_control);
    if(use_control){
        Marker boundingBox;

        std::string box_type;
        ros::param::get("/localization_marker_server/bounding_box/type", box_type);
        if (box_type.compare("SPHERE") == 0) {
            boundingBox.type = Marker::SPHERE;
            ros::param::get("/localization_marker_server/bounding_box/radius", boundingBox.scale.x);
            ros::param::get("/localization_marker_server/bounding_box/radius", boundingBox.scale.y);
            ros::param::get("/localization_marker_server/bounding_box/radius", boundingBox.scale.z);
            boundingBox.scale.x *= 2;
            boundingBox.scale.y *= 2;
            boundingBox.scale.z *= 2;

        } else if (box_type.compare("RECTANGLE") == 0) {
            boundingBox.type = Marker::CUBE;
            ros::param::get("/localization_marker_server/bounding_box/x", boundingBox.scale.x);
            ros::param::get("/localization_marker_server/bounding_box/y", boundingBox.scale.y);
            ros::param::get("/localization_marker_server/bounding_box/z", boundingBox.scale.z);

        } else {
            ROS_WARN("Localization Marker Server: Bounding Box Type not Recognized!");
            ROS_WARN("Default Type = SPHERE");

            boundingBox.type = Marker::SPHERE;
            ros::param::get("/localization_marker_server/bounding_box/radius", boundingBox.scale.x);
            ros::param::get("/localization_marker_server/bounding_box/radius", boundingBox.scale.y);
            ros::param::get("/localization_marker_server/bounding_box/radius", boundingBox.scale.z);
        }

        ros::param::get("/localization_marker_server/bounding_box/color/r", c); boundingBox.color.r = c;
        ros::param::get("/localization_marker_server/bounding_box/color/g", c); boundingBox.color.g = c;
        ros::param::get("/localization_marker_server/bounding_box/color/b", c); boundingBox.color.b = c;
        ros::param::get("/localization_marker_server/bounding_box/color/a", c); boundingBox.color.a = c;

        InteractiveMarkerControl boundingBoxControl;
        boundingBoxControl.always_visible = true;
        boundingBoxControl.orientation_mode = InteractiveMarkerControl::FIXED;
//        boundingBoxControl.interaction_mode = InteractiveMarkerCOntrol::NONE;
        boundingBoxControl.markers.push_back(boundingBox);
        int_marker.controls.push_back(boundingBoxControl);
    }

// ********* The Model to Control ********* //  
    control_box.always_visible = true;
    control_box.markers.push_back( marker );
    int_marker.controls.push_back( control_box );
    int_marker.controls.back();



// ********* The Menu Controller ********* //  

    //Check for Menu Control
    ros::param::get("/localization_marker_server/control/menu", use_control);
    if (use_control){
        control_menu.interaction_mode = InteractiveMarkerControl::MENU;
        int_marker.controls.push_back(control_menu);
    }


// ********* The Pose Controls ********* // 
    control_pose.orientation_mode = InteractiveMarkerControl::INHERIT;

    //Check for Translation X Control
    ros::param::get("/localization_marker_server/control/translation/x", use_control);
    if (use_control){
        control_pose.name = "translate_x";
        control_pose.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 1;
        control_pose.orientation.y = 0;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);
    }

    //Check for Translation Y Control
    ros::param::get("/localization_marker_server/control/translation/y", use_control);
    if (use_control){
        control_pose.name = "translate_y";
        control_pose.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 0;
        control_pose.orientation.y = 1;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);
    }

    //Check for Translation Z Control
    ros::param::get("/localization_marker_server/control/translation/z", use_control);
    if (use_control){
        control_pose.name = "translate_z";
        control_pose.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 0;
        control_pose.orientation.y = 0;
        control_pose.orientation.z = 1;
        int_marker.controls.push_back(control_pose);
    }

    //Check for Rotation X Control
    ros::param::get("/localization_marker_server/control/rotation/x", use_control);
    if (use_control){
        control_pose.name = "translate_x";
        control_pose.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 1;
        control_pose.orientation.y = 0;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);
    }

    //Check for Rotation Y Control
    ros::param::get("/localization_marker_server/control/rotation/y", use_control);
    if (use_control){
        control_pose.name = "translate_y";
        control_pose.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        control_pose.orientation.w = 1;
        control_pose.orientation.x = 0;
        control_pose.orientation.y = 1;
        control_pose.orientation.z = 0;
        int_marker.controls.push_back(control_pose);
    }

    //Check for Rotation Z Control
    ros::param::get("/localization_marker_server/control/rotation/z", use_control);
    if (use_control){
        control_pose.name = "translate_z";
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

void pc_target_cb(sensor_msgs::PointCloud2::Ptr input){

    bool use_voxel;

    ros::param::get("/localization_marker_server/target_cloud/voxel", use_voxel);
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
        makeValve(currentPose);
        server->applyChanges();
    }

}

void set_cb(localization_marker_msgs::set_localization_marker input){

    ros::param::set("/localization_marker_server/marker/name", input.name);
    ros::param::set("/localization_marker_server/marker/description", input.description);
    ros::param::set("/localization_marker_server/marker/type", input.marker_type);
    ros::param::set("/localization_marker_server/marker/mesh_resource", input.mesh_resource);
    ros::param::set("/localization_marker_server/marker/defaultPose/position/x", input.defaultPose.pose.position.x);
    ros::param::set("/localization_marker_server/marker/defaultPose/position/y", input.defaultPose.pose.position.y);
    ros::param::set("/localization_marker_server/marker/defaultPose/position/z", input.defaultPose.pose.position.z);
    ros::param::set("/localization_marker_server/marker/defaultPose/orientation/w", input.defaultPose.pose.orientation.w);
    ros::param::set("/localization_marker_server/marker/defaultPose/orientation/x", input.defaultPose.pose.orientation.x);
    ros::param::set("/localization_marker_server/marker/defaultPose/orientation/y", input.defaultPose.pose.orientation.y);
    ros::param::set("/localization_marker_server/marker/defaultPose/orientation/z", input.defaultPose.pose.orientation.z);
    ros::param::set("/localization_marker_server/marker/radius", input.radius);
    ros::param::set("/localization_marker_server/marker/width", input.width);
    ros::param::set("/localization_marker_server/marker/height", input.height);
    ros::param::set("/localization_marker_server/marker/scale", input.scale);
    ros::param::set("/localization_marker_server/marker/color/r", input.marker_color.r);
    ros::param::set("/localization_marker_server/marker/color/g", input.marker_color.g);
    ros::param::set("/localization_marker_server/marker/color/b", input.marker_color.b);
    ros::param::set("/localization_marker_server/marker/color/a", input.marker_color.a);

    ros::param::set("/localization_marker_server/bounding_box/show", input.show_bounding_volume);
    ros::param::set("/localization_marker_server/bounding_box/type", input.bounding_volume_type);
    ros::param::set("/localization_marker_server/bounding_box/radius", input.sphere_radius);
    ros::param::set("/localization_marker_server/bounding_box/x", input.bounding_volume_dimensions.x);
    ros::param::set("/localization_marker_server/bounding_box/y", input.bounding_volume_dimensions.y);
    ros::param::set("/localization_marker_server/bounding_box/z", input.bounding_volume_dimensions.z);
    ros::param::set("/localization_marker_server/bounding_box/color/r", input.bounding_volume_color.r);
    ros::param::set("/localization_marker_server/bounding_box/color/g", input.bounding_volume_color.g);
    ros::param::set("/localization_marker_server/bounding_box/color/b", input.bounding_volume_color.b);
    ros::param::set("/localization_marker_server/bounding_box/color/a", input.bounding_volume_color.a);

    ros::param::set("/localization_marker_server/control/translation/x", input.enable_translate_X);
    ros::param::set("/localization_marker_server/control/translation/y", input.enable_translate_Y);
    ros::param::set("/localization_marker_server/control/translation/z", input.enable_translate_Z);
    ros::param::set("/localization_marker_server/control/rotation/x", input.enable_rotate_X);
    ros::param::set("/localization_marker_server/control/rotation/y", input.enable_rotate_Y);
    ros::param::set("/localization_marker_server/control/rotation/z", input.enable_rotate_Z);
    ros::param::set("/localization_marker_server/control/menu", input.enable_menu_control);

    ros::param::set("/localization_marker_server/target_cloud/topic", input.target_cloud_topic);
    ros::param::set("/localization_marker_server/target_cloud/voxel", input.voxelize_target_cloud);
    ros::param::set("/localization_marker_server/input_cloud/resource", input.input_cloud_resource);
    ros::param::set("/localization_marker_server/input_cloud/discretization", input.input_cloud_discretization);

    ros::param::set("/localization_marker_server/icp/maxCoorespondanceDistance", input.max_icp_coorespondence_distance);
    ros::param::set("/localization_marker_server/icp/maxInterations", input.max_icp_iterations);

    ros::param::set("/localization_marker_server/debug/input_cloud/pre_ICP/show", input.pre_ICP_show);
    ros::param::set("/localization_marker_server/debug/input_cloud/pre_ICP/topic", input.pre_ICP_topic);
    ros::param::set("/localization_marker_server/debug/input_cloud/post_ICP/show", input.post_ICP_show);
    ros::param::set("/localization_marker_server/debug/input_cloud/post_ICP/topic", input.post_ICP_topic);


    ros::param::set("/localization_marker_server/hasChanged", true);
}

void check(void){

    bool changed;
    ros::param::get("/localization_marker_server/hasChanged", changed);
    if (changed){
        server->erase(marker_name);
        makeValve(currentPose);
        server->applyChanges();
        ros::param::set("/localization_marker_server/hasChanged", false);
    }

}

// %Tag(main)%
int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_marker_server");
    ros::NodeHandle n;

    std::string pc_topic;
    ros::param::get("/localization_marker_server/target_cloud/topic", pc_topic);
    ros::Subscriber pc_target_sub = n.subscribe(pc_topic.c_str(),1, pc_target_cb);

    ros::Subscriber localization_marker_set = n.subscribe("/localization_marker_server/set", 1, set_cb);

    object_pose = n.advertise<geometry_msgs::PoseStamped>("/localization_marker_server/object/pose", 1);

    bool debug_pre_ICP;
    ros::param::get("/localization_marker_server/debug/input_cloud/pre_ICP/show", debug_pre_ICP);
    if (debug_pre_ICP){
        std::string input_topic;
        ros::param::get("/localization_marker_server/debug/input_cloud/pre_ICP/topic", input_topic);
        debug_input_cloud_pre_ICP_pub = n.advertise<sensor_msgs::PointCloud2>(input_topic.c_str(), 1);
    }

    bool debug_post_ICP;
    ros::param::get("/localization_marker_server/debug/input_cloud/post_ICP/show", debug_post_ICP);
    if (debug_post_ICP){
        std::string input_topic;
        ros::param::get("/localization_marker_server/debug/input_cloud/post_ICP/topic", input_topic);
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
    ros::param::get("/localization_marker_server/marker/defaultPose/position/x", defaultPose.position.x);
    ros::param::get("/localization_marker_server/marker/defaultPose/position/y", defaultPose.position.y);
    ros::param::get("/localization_marker_server/marker/defaultPose/position/z", defaultPose.position.z);
    ros::param::get("/localization_marker_server/marker/defaultPose/orientation/w", defaultPose.orientation.w);
    ros::param::get("/localization_marker_server/marker/defaultPose/orientation/x", defaultPose.orientation.x);
    ros::param::get("/localization_marker_server/marker/defaultPose/orientation/y", defaultPose.orientation.y);
    ros::param::get("/localization_marker_server/marker/defaultPose/orientation/z", defaultPose.orientation.z);
    currentPose = defaultPose;

    ros::param::set("/localization_marker_server/hasChanged", true);

    ros::Rate r(20);

    while(ros::ok()){

        check();

        ros::spinOnce();

        r.sleep();
    }

    ros::spin();
}
// %EndTag(main)%
