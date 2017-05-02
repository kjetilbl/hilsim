#include "RVIZ_Interface.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <vector>

#include <QDebug>

rvizInterface::rvizInterface(ros::NodeHandle nh){
	objectPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	nh.getParam("start_longitude", mapOrigin.longitude);
	nh.getParam("start_latitude", mapOrigin.latitude);
}


void rvizInterface::set_object(string objectID, gpsPoint3DOF position){
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

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

/*   	IDelements.pop_back();
   	marker.ns = IDelements.pop_back().c_str();
   	while(!IDelements.empty()){
   		marker.ns = IDelements.pop_back().c_str() + marker.ns;
   	}*/
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = objectID;
    marker.id = rvizID;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "file:///home/uss_deplorables_small.stl";

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    double y = -(position.longitude - mapOrigin.longitude)/longitude_degs_pr_meter(position.latitude);
    double x = (position.latitude - mapOrigin.latitude)/latitude_degs_pr_meter();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x + 64*sqrt(2)*cos(deg2rad(position.heading + 36));
    marker.pose.position.y = y - 64*sqrt(2)*cos(deg2rad(position.heading - 54));
    marker.pose.position.z = 15;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sin(deg2rad(-position.heading/2));//-deg2rad(position.heading);
    marker.pose.orientation.w = cos(deg2rad(-position.heading/2));

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.2;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker.color.a = 1;


    marker.lifetime = ros::Duration();
    objectPub.publish(marker);
}