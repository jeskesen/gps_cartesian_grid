
#include <ros/ros.h>
#include <gps_cartesian_grid/FixToPoint.h>
#include <GeographicLib/LocalCartesian.hpp>

class GpsCartesianGridNode
{
public:
	GpsCartesianGridNode();
private:

	bool fixToPointServiceCall(gps_cartesian_grid::FixToPoint::Request &req, gps_cartesian_grid::FixToPoint::Response &point);
	ros::NodeHandle nh_, private_nh_;
	ros::ServiceServer fixToPointService_;

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

	grid_.Reset(lat,lon,alt);

	fixToPointService_ = nh_.advertiseService("fix_to_point", &GpsCartesianGridNode::fixToPointServiceCall, this);
}

bool GpsCartesianGridNode::fixToPointServiceCall(gps_cartesian_grid::FixToPoint::Request &req,
		gps_cartesian_grid::FixToPoint::Response &point)
{
	grid_.Forward(req.gps_fix.latitude, req.gps_fix.longitude, req.gps_fix.altitude, point.cartesian.x,  point.cartesian.y,  point.cartesian.z);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_cartesian_grid_node");

	ros::spin();

	return 0;
}
