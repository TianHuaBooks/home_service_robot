#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class ObjPickupDrop {
public:
	ObjPickupDrop(ros::NodeHandle& nh, ros::Publisher& pub) 
	  : m_marker_pub(pub) 
	{
	    m_goal_sub = nh.subscribe("/mygoal", 1, &ObjPickupDrop::goalCallback, this);
            ROS_INFO("sub /mygoal");
	}
	~ObjPickupDrop() {}

	enum State { UNINIT, PICKING_UP, PICKED_UP, DROPPED };

	bool waitForSubscriber();
	void goalCallback(const geometry_msgs::Pose& msg);
	void addMarkerAt(const geometry_msgs::Pose& pose);
	void removeMarker();

private:
	const ros::Publisher& m_marker_pub;
        ros::Subscriber m_goal_sub;
	State m_state = { UNINIT };
};

bool ObjPickupDrop::waitForSubscriber()
{
	for (int i = 0; (i < 5) && (m_marker_pub.getNumSubscribers() < 1); i++)
    	{
      	   if (!ros::ok())
       	       return false;
      	   ROS_WARN_ONCE("Please create a subscriber to the marker");
      	   sleep(1);
    	}

        ROS_INFO("... continue ... ");
	return true;
}

void ObjPickupDrop::goalCallback(const geometry_msgs::Pose& msg)
{
	switch(m_state) {
	   case UNINIT:
		addMarkerAt(msg);
		m_state = PICKING_UP;
        	ROS_INFO("Picking up (add marker) ... ");
		break;
	   case PICKING_UP:
		removeMarker();
		m_state = PICKED_UP;
        	ROS_INFO("Picked up (removed marker) ... ");
		break;
	   case PICKED_UP:
		addMarkerAt(msg);
		m_state = DROPPED;
        	ROS_INFO("Dropped (add marker) ... ");
		break;
	   case DROPPED:
		break;
	}
}

void ObjPickupDrop::addMarkerAt(const geometry_msgs::Pose& pose)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose = pose;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    m_marker_pub.publish(marker);
}

void ObjPickupDrop::removeMarker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;

    marker.action = visualization_msgs::Marker::DELETE;
    m_marker_pub.publish(marker);
}

void do_simpleMarker(ros::Publisher& marker_pub);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //do_simpleMarker(marker_pub);

  // Create an object pickup and drop
  ObjPickupDrop obj(n, marker_pub);
  if (!obj.waitForSubscriber())
      ROS_WARN("ros failed!");

  ros::spin();

  return 0;
}
  
void do_simpleMarker(ros::Publisher& marker_pub) {

  ros::Rate r(1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
        return;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }
}
