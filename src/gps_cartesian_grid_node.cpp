
#include <ros/ros.h>
#include <gps_cartesian_grid/FixToPoint.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <GeographicLib/LocalCartesian.hpp>

class GpsCartesianGridNode
{
public:
	GpsCartesianGridNode();
private:
	geometry_msgs::Point fixToPoint(sensor_msgs::NavSatFix gps_fix);
	bool fixToPointServiceCall(gps_cartesian_grid::FixToPoint::Request &req, gps_cartesian_grid::FixToPoint::Response &point);
	void newNavSatFix(sensor_msgs::NavSatFix newFix){gps_fix_ = newFix; point_pub_.publish(fixToPoint(gps_fix_));}
	void newImuData(sensor_msgs::Imu newImu){imu_ = newImu;  publishTf();}
	void publishTf();
	ros::NodeHandle nh_, private_nh_;

	// for service
	ros::ServiceServer fixToPointService_;

	// for tf
	tf2_ros::TransformBroadcaster tf_broadcaster_;
	std::string global_frame_, robot_frame_;

	std::string gps_topic_, imu_topic_;
	ros::Subscriber gps_sub_, imu_sub_;
	ros::Publisher point_pub_;
	sensor_msgs::NavSatFix gps_fix_;
	sensor_msgs::Imu imu_;

	// thirdparty GeographicLib
	GeographicLib::LocalCartesian grid_;
};

GpsCartesianGridNode::GpsCartesianGridNode() : nh_(),
		private_nh_("~"),
		grid_(0,0,0) // need to init to SOMETHING since its statically allocated.  will reset later when actual values are read.
{
	// setup origin
	double lat, lon, alt;
	if(!private_nh_.getParam("origin_latitude", lat) || !private_nh_.getParam("origin_longitude", lon))
	{
		ROS_FATAL("GPS Cartesian Grid cannot function without origin.  Either latitude, longitude, or both are missing");
	}

	private_nh_.param("origin_altitude", alt, 0.0);
	private_nh_.param("global_frame", global_frame_, std::string("/world"));
	private_nh_.param("robot_frame", robot_frame_, std::string("base_link"));

	private_nh_.param("gps_fix_topic", gps_topic_, std::string("gps/fix"));
	private_nh_.param("imu_topic", imu_topic_, std::string("imu/data"));

	grid_.Reset(lat,lon,alt);

	fixToPointService_ = nh_.advertiseService("fix_to_point", &GpsCartesianGridNode::fixToPointServiceCall, this);
	gps_sub_ = nh_.subscribe(gps_topic_, 1, &GpsCartesianGridNode::newNavSatFix, this);
	imu_sub_ = nh_.subscribe(imu_topic_, 1, &GpsCartesianGridNode::newImuData, this);
	point_pub_ = nh_.advertise<geometry_msgs::Point>("position", 1);

}

geometry_msgs::Point GpsCartesianGridNode::fixToPoint(sensor_msgs::NavSatFix gps_fix)
{
	geometry_msgs::Point point;
	grid_.Forward(gps_fix.latitude, gps_fix.longitude, gps_fix.altitude, point.x,  point.y,  point.z);
	return point;
}

bool GpsCartesianGridNode::fixToPointServiceCall(gps_cartesian_grid::FixToPoint::Request &req,
		gps_cartesian_grid::FixToPoint::Response &point)
{
	point.cartesian = fixToPoint(req.gps_fix);
	return true;
}

void GpsCartesianGridNode::publishTf()
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = global_frame_;
	transformStamped.child_frame_id = robot_frame_;

	geometry_msgs::Point point = fixToPoint(gps_fix_);
	transformStamped.transform.translation.x = point.x;
	transformStamped.transform.translation.y = point.y;
	transformStamped.transform.translation.z = point.z;

	/*
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	*/
	transformStamped.transform.rotation.x = imu_.orientation.x;
	transformStamped.transform.rotation.y = imu_.orientation.y;
	transformStamped.transform.rotation.z = imu_.orientation.z;
	transformStamped.transform.rotation.w = imu_.orientation.w;

	tf_broadcaster_.sendTransform(transformStamped);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_cartesian_grid_node");
	GpsCartesianGridNode local_grid;
	ros::spin();
	return 0;
}
