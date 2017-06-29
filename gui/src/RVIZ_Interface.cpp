#include "RVIZ_Interface.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <vector>

#include <QDebug>

rvizInterface::rvizInterface(ros::NodeHandle *nh){
	objectPub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10000);
	mapPub = nh->advertise<nav_msgs::OccupancyGrid>("/map", 1000);

	nh->getParam("start_longitude", mapOrigin.longitude);
	nh->getParam("start_latitude", mapOrigin.latitude);
}


void rvizInterface::set_object(string objectID, gpsPoint3DOF position, double crossSection){
	/*
	nav_msgs::OccupancyGrid myMap;
	myMap.header.stamp = ros::Time();
	myMap.header.frame_id = "/map";
	myMap.info.map_load_time = ros::Time::now();
	myMap.info.resolution = 10;
	myMap.info.width = 100;
	myMap.info.height = 50;
	myMap.info.origin.position.x = 0;
	myMap.info.origin.position.y = 0;
	myMap.info.origin.position.z = 0;
	myMap.info.origin.orientation.x = 0;
	myMap.info.origin.orientation.y = 0;
	myMap.info.origin.orientation.z = 0;
	myMap.info.origin.orientation.w = 0;
	vector<signed char> data;
	for (int i = 0; i < 5000; i++)
	{
		data.push_back(50);
	}
	myMap.data = data;

	mapPub.publish(myMap);
	ros::spinOnce();
	*/

	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    stringstream ss;
    ss.str(objectID);
    string element;
    vector<string> IDelements;
   	while( getline(ss, element, '_')){
   		IDelements.push_back(element);
   	}
   	if(IDelements.back().c_str()[0] < '0' || IDelements.back().c_str()[0] > '9' || IDelements.size() < 2){
   		qDebug() << "RVIZ object ID must contain a descriptive string and end with '_' and a number. Tried to set position of" << objectID.c_str();
   		return;
   	}
   	int rvizID = atoi(IDelements.back().c_str());
    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
 	IDelements.pop_back();
 	string nameSpace;
 	for(auto const& IDelement: IDelements){
 		nameSpace += IDelement + '_';
	}
	marker.ns = nameSpace.substr(0, nameSpace.size()-1);
    marker.id = rvizID;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    double y = -(position.longitude - mapOrigin.longitude)/longitude_degs_pr_meter(position.latitude);
    double x = (position.latitude - mapOrigin.latitude)/latitude_degs_pr_meter();


    if(marker.ns == "AIS_user"){
	    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	    marker.mesh_resource = "file:///home/uss_deplorables_small.stl";

	    double scale = sqrt(crossSection)*0.0013;
	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = x + 64*sqrt(2)*cos(deg2rad(position.heading + 36))*scale/0.1;
	    marker.pose.position.y = y - 64*sqrt(2)*cos(deg2rad(position.heading - 54))*scale/0.1;
	    marker.pose.position.z = 15*scale/0.1;
	    marker.pose.orientation.x = 0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = sin(deg2rad(-position.heading/2));//-deg2rad(position.heading);
	    marker.pose.orientation.w = cos(deg2rad(-position.heading/2));

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = scale;
	    marker.scale.y = scale;
	    marker.scale.z = scale;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 0.2;
	    marker.color.g = 0.2;
	    marker.color.b = 0.2;
	    marker.color.a = 1;
    }
    else if(marker.ns == "fixed_obstacle"){
    	marker.type = visualization_msgs::Marker::CYLINDER;
	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = x;
	    marker.pose.position.y = y;
	    marker.pose.position.z = 0;
	    marker.pose.orientation.x = 0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = sin(deg2rad(-position.heading/2));//-deg2rad(position.heading);
	    marker.pose.orientation.w = cos(deg2rad(-position.heading/2));

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = sqrt(crossSection);
	    marker.scale.y = sqrt(crossSection);
	    marker.scale.z = sqrt(crossSection)*2;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 1;
	    marker.color.g = 1;
	    marker.color.b = 1;
	    marker.color.a = 1;
    }

    marker.lifetime = ros::Duration();
    objectPub.publish(marker);
    // ros::spinOnce();
}

void rvizInterface::show_detected_target(	int targetID, 
											string objectDescriptor, 
											gpsPointStamped position,
											double SOG,
											double crossSection){
	visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    marker.ns = "detected_" + objectDescriptor;
    marker.id = targetID;

    marker.action = visualization_msgs::Marker::MODIFY;

    double y = -(position.longitude - mapOrigin.longitude)/longitude_degs_pr_meter(position.latitude);
    double x = (position.latitude - mapOrigin.latitude)/latitude_degs_pr_meter();

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sin(deg2rad(-position.heading/2));;
    marker.pose.orientation.w = cos(deg2rad(-position.heading/2));

    if(objectDescriptor == "fixed_obstacle"){
	marker.type = visualization_msgs::Marker::CYLINDER;
	    marker.scale.x = sqrt(crossSection);
	    marker.scale.y = sqrt(crossSection);
	    marker.scale.z = 1;
    }
    else if( objectDescriptor == "vessel"){
	marker.type = visualization_msgs::Marker::CYLINDER;
	    marker.scale.x = sqrt(crossSection);
	    marker.scale.y = sqrt(crossSection)/3;
	    marker.scale.z = 1;// sqrt(crossSection)/2;
    }

    marker.color.r = 0.9;
    marker.color.g = 0.5;
    marker.color.b = 0;
	marker.color.a = 0.6;

    marker.lifetime = ros::Duration(2);
    objectPub.publish(marker);
    ros::spinOnce();


    // Display info above target:
    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = "map";
    textMarker.header.stamp = ros::Time();

    textMarker.ns = "detected_" + objectDescriptor + "_info";
    textMarker.id = targetID;
    textMarker.text = 	objectDescriptor 
    					+ "\nID: " + to_string(targetID)
    					+ "\nCOG: " + to_string((int)position.heading) + " [deg]"
    					+ "\nSOG: " + to_string((int)SOG) + " [kts]";

    textMarker.action = visualization_msgs::Marker::ADD;
	textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    y = -(position.longitude - mapOrigin.longitude)/longitude_degs_pr_meter(position.latitude);
    x = (position.latitude - mapOrigin.latitude)/latitude_degs_pr_meter();

    textMarker.pose.position.x = x;
    textMarker.pose.position.y = y;

    textMarker.pose.position.z = sqrt(crossSection)/4 + 20;
	textMarker.scale.z = 5;

	textMarker.color.a = 1;

    textMarker.lifetime = ros::Duration(2);
    objectPub.publish(textMarker);
    ros::spinOnce();
}

