#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"dynamic_tf_sub");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate rate(10);
    while (ros::ok())
    {

        try {
            geometry_msgs::TransformStamped t = tf_buffer_->lookupTransform( "world", "bottle_blue", ros::Time(0));
            ROS_INFO("(%.2f,%.2f,%.2f)",t.transform.translation.x,t.transform.translation.y,t.transform.translation.z);
        } 
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("Could not transform: %s",e.what());
        }

        r.sleep();  
        ros::spinOnce();
    }

    return 0;
}