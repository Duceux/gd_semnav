#include "globallocalizer.h"

using namespace sn;

GlobalLocalizer::GlobalLocalizer(const ros::NodeHandle& node):
    node_(node)
{
    mapSub_ = node_.subscribe("/static_map", 1, &GlobalLocalizer::mapCallback, this);
    scanSub_ = node_.subscribe("/scan", 1, &GlobalLocalizer::scanCallback, this);
    poseSub_ = node_.subscribe("/initialpose", 1, &GlobalLocalizer::initialpose, this);

    laserPub_ = node_.advertise<Laser>("localized_scan", 1);
    scorePub_ = node_.advertise<std_msgs::Float64>("score", 1);
    cloudPub_ = node_.advertise<geometry_msgs::PoseArray>("particles", 1);
}

void GlobalLocalizer::scanCallback(LaserPtr scan){
    Laser s = *scan;

    static double max_score = DMAX;
    static tf::Transform best_tranform;
    static tf::TransformBroadcaster br;

    static bool first_scan = true;
    if(first_scan){
      best_tranform.getRotation().normalize();
      br.sendTransform(tf::StampedTransform(best_tranform.inverse(),
                                            s.header.stamp,
                                            "/laser", "/static_map"));
      first_scan = false;
      return;
    }

    {
        vector_pts_t l;
        sn::rosToLaser(s, l);
        localizer_.process(l);
        geometry_msgs::PoseArray pose_array;
        for(auto it=localizer_.getParticles().begin(); it!=localizer_.getParticles().end(); ++it){
            geometry_msgs::PoseStamped p;
            sn::poseToRos(*it, p);
            p.pose.position.z = odomTrans_.getOrigin().getZ();
            pose_array.poses.push_back(p.pose);
        }
        pose_array.header = scan->header;
        pose_array.header.frame_id = "/static_map";
        cloudPub_.publish(pose_array);
    }

    if(max_score > localizer_.getScore())
    {
        max_score = localizer_.getScore();
        sn::poseToTf(localizer_.getPosition(), best_tranform);
        std_msgs::Float64 score_msg;
        score_msg.data = max_score;
        scorePub_.publish(score_msg);
    }


    laserPub_.publish(s);

    best_tranform.getRotation().normalize();
    br.sendTransform(tf::StampedTransform(best_tranform.inverse(), s.header.stamp, "/laser", "/static_map"));
}

void GlobalLocalizer::mapCallback(OGridPtr map){
    sn::Map mymap;
    sn::rosToMap(map, mymap);
    mymap.computeDistSup(120);
    localizer_.init(mymap);
}

void GlobalLocalizer::initialpose(PoseCovStampPtr initial_pose){
    ROS_INFO("Initial pose received");

    sn::pose_t hint;
    sn::rosToPose(initial_pose, hint);
    localizer_.setPosition(hint);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "globallocalizer");
    ros::NodeHandle n(std::string("~"));

    GlobalLocalizer mynode(n);

    ros::spin();
    return 0;
}
