#include "localizernode.h"

using namespace sn;

LocalizerNode::LocalizerNode(const ros::NodeHandle& node):
    node_(node)
{
    mapSub_ = node_.subscribe("/map", 1, &LocalizerNode::mapCallback, this);
    scanSub_ = node_.subscribe("/scan", 1, &LocalizerNode::scanCallback, this);
    poseSub_ = node_.subscribe("/initialpose", 1, &LocalizerNode::initialpose, this);

    laserPub_ = node_.advertise<Laser>("/localized_scan", 1);
    scorePub_ = node_.advertise<std_msgs::Float64>("/score", 1);
    cloudPub_ = node_.advertise<geometry_msgs::PoseArray>("particles", 1);
}

void LocalizerNode::scanCallback(LaserPtr scan){
    Laser s = *scan;

    static double max_score = DMAX;
    static tf::Transform best_tranform;
    static tf::TransformBroadcaster br;

    static bool first_scan = true;
    if(first_scan){
      best_tranform.getRotation().normalize();
      br.sendTransform(tf::StampedTransform(best_tranform.inverse(),
                                            s.header.stamp,
                                            "/laser", "/map"));
      first_scan = false;
      return;
    }

    /*
    tf::StampedTransform transform;
    try{
      listener_.lookupTransform("map", "odom", scan->header.stamp, transform);
      pose_t odom;
      sn::tfToPose(transform, odom);
      localizer_.setPosition(odom);
    }
    catch (tf::TransformException ex){
    }
*/

//    if(max_score > 0.0)
    {
        vector_pts_t l;
        sn::rosToLaser(s, l);
        localizer_.process(l);
        geometry_msgs::PoseArray pose_array;
        for(auto it=localizer_.mParticles.begin(); it!=localizer_.mParticles.end(); ++it){
            geometry_msgs::PoseStamped p;
            sn::poseToRos(*it, p);
            p.pose.position.z = odomTrans_.getOrigin().getZ();
            pose_array.poses.push_back(p.pose);
        }
        pose_array.header = scan->header;
        pose_array.header.frame_id = "/map";
        cloudPub_.publish(pose_array);
    }


//    if(max_score > localizer_.getScore())
    {
        max_score = localizer_.getScore();
        sn::poseToTf(localizer_.getPosition(), best_tranform);
        std_msgs::Float64 score_msg;
        score_msg.data = max_score;
        scorePub_.publish(score_msg);
    }


    laserPub_.publish(s);

    best_tranform.getRotation().normalize();
    br.sendTransform(tf::StampedTransform(best_tranform.inverse(), s.header.stamp, "/laser", "/map"));
}

void LocalizerNode::mapCallback(OGridPtr map){
    sn::Map mymap;
    sn::rosToMap(map, mymap);
    mymap.computeDistSup(120);
    localizer_.init(mymap);
}

void LocalizerNode::initialpose(PoseCovStampPtr initial_pose){
    ROS_INFO("Initial pose received");

    sn::pose_t hint;
    sn::rosToPose(initial_pose, hint);
    localizer_.setPosition(hint);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle n(std::string("~"));

    LocalizerNode mynode(n);

    ros::spin();
    return 0;
}
