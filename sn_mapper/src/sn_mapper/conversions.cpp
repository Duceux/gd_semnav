#include <sn_mapper/conversions.h>

namespace sn{

void rosToMap(OGridPtr og, Map &map){
    OGrid grid = *og;
    rosToMap(grid, map);
}

void rosToMap(OGrid& og, Map &map){
    map.resolution = og.info.resolution;
    map.origin.x = og.info.origin.position.x;
    map.origin.y = og.info.origin.position.y;
    map.origin.theta = tf::getYaw(og.info.origin.orientation);
    map.width = og.info.width;
    map.height = og.info.height;

    map.grid = cv::Mat(map.height, map.width, CV_8UC1);

    // FIXME
    cv::Mat tmp = map.grid;
    for(uint i=0; i<og.data.size(); ++i){
        // conversion de merdeuh!
        if(og.data[i] == -1)
            tmp.data[i] = 128;
        else
            tmp.data[i] = (100 - og.data[i])*255.0/100;
    }

    cv::flip(tmp, map.grid, 0);
}

void mapToRos(const Map &map, OGrid& og){
    og.info.resolution = map.resolution;
    og.info.origin.position.x = map.origin.x;
    og.info.origin.position.y = map.origin.y;
    og.info.origin.orientation = tf::createQuaternionMsgFromYaw(map.origin.theta);
    og.info.width = map.width;
    og.info.height = map.height;

    og.data.resize(map.height*map.width);

    // FIXME
    cv::Mat tmp;
    cv::flip(map.grid, tmp, 0);
    for(int i=0; i<map.height*map.width; ++i){
        // conversion de merdeuh!
        if(tmp.data[i] == 128)
            og.data[i] = -1;
        else
            og.data[i] = 100 - tmp.data[i]*100.0/255;
    }
}

void rosToProjectedLaser(const sensor_msgs::LaserScan &s,
                         const tf::Transform &tf, vector_pts_t &l){
    pose_t robot;
    robot.x = tf.getOrigin().x();
    robot.y = tf.getOrigin().y();
    robot.theta = tf::getYaw(tf.getRotation());

    rosToLaser(s, l);

    for(uint i=0; i<l.size(); ++i)
        l[i] = oplus(robot, l[i]);
}


void rosToLaser(const Laser& s, vector_pts_t &l){
    l.clear();
    l.reserve(s.ranges.size());
    for(uint i=0; i<s.ranges.size(); ++i){
        double rho = s.ranges[i];
        double theta = s.angle_min + i*s.angle_increment;
        if(rho < s.range_max &&
                rho > s.range_min &&
                !std::isnan(rho) &&
                !std::isinf(rho)){
            double x = rho*cos(theta);
            double y = rho*sin(theta);
            l.push_back(create(x, y));
        }
    }
}

void poseToTf(const pose_t &robot, tf::Transform &tf){
    tf.setOrigin( tf::Vector3(robot.x,
                              robot.y, 0.0) );
    tf.setRotation( tf::Quaternion(tf::Vector3(0, 0, 1), robot.theta));
}

void tfToPose(const tf::Transform &tf, pose_t& robot){
    robot.x = tf.getOrigin().x();
    robot.y = tf.getOrigin().y();
    robot.theta = tf::getYaw(tf.getRotation());
}

void rosToPose(PoseCovStampPtr pose, pose_t& robot){
  robot.x = pose->pose.pose.position.x;
  robot.y = pose->pose.pose.position.y;
  robot.theta = tf::getYaw(pose->pose.pose.orientation);
}

void poseToRos(const pose_t& robot, PoseStamped &pose){
  pose.pose.position.x = robot.x;
  pose.pose.position.y = robot.y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot.theta);
}

}
