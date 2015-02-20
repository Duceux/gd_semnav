#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sn_msgs/DetectionArray.h>
#include <sn_geometry/transform.h>

struct Transformer{

    tf::TransformListener tf_listener;
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    std::string target;
    double timeout;

    void scanCallback(sn_msgs::DetectionArrayPtr input)
    {
        tf::StampedTransform transform;
        try{
            tf_listener.waitForTransform(target, input->header.frame_id, input->header.stamp,
                                         ros::Duration(timeout));
            tf_listener.lookupTransform(target, input->header.frame_id, input->header.stamp, transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return;
        }

        input->header.frame_id = target;
        for(auto& lsd: input->detections){
            lsd.robot = transformPose2D(transform, lsd.robot);
            lsd.points = sn::transform(lsd.points, transform);
        }

        publisher.publish(input);
    }

    geometry_msgs::Pose2D transformPose2D(const tf::StampedTransform& transform, const geometry_msgs::Pose2D& pose)
    {
      double x, y, theta;
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
      theta = tf::getYaw(transform.getRotation());

  //oplus
      geometry_msgs::Pose2D res;
      res.x = x +  pose.x*std::cos(theta) - pose.y*std::sin(theta);
      res.y = y + pose.x*std::sin(theta) + pose.y*std::cos(theta);
      res.theta = theta + pose.theta;

      return res;
    }
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "detection_transform");
    ros::NodeHandle handle(std::string("~"));

    Transformer transformer;

    std::string laser_input;
    ros::param::param<std::string>("~input", laser_input, "/laser_segmentation/detections");
    transformer.subscriber = handle.subscribe(laser_input, 1, &Transformer::scanCallback, &transformer);

    std::string output;
    ros::param::param<std::string>("~ouput", output, "/transformed"+laser_input);
    transformer.publisher = handle.advertise<sn_msgs::DetectionArray>(output, 1);

    ros::param::param<std::string>("~target_frame", transformer.target, "odom");

    ros::param::param<double>("~timeout", transformer.timeout, 0.5);


    ros::spin();

    return 0;
}
