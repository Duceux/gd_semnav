#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/Tracker.h>
#include <opencv2/opencv.hpp>
#include <sn_geometry/sn_geometry.h>
#include <sn_features/laser_descriptor.h>
#include <sn_features/polar_histogram.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

struct compare_func{ bool operator()(sn_msgs::TrackerPtr const& l, sn_msgs::TrackerPtr const& r){return l->uid<r->uid;} };
typedef std::set<sn_msgs::TrackerPtr, compare_func> TrackersSet;

void load(const std::string& filename, TrackersSet& trackers)
{
    std::cout << "opening: " << filename << std::endl;
    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("/tracking/ended_tracker"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(auto m: view){
            sn_msgs::TrackerPtr g = m.instantiate<sn_msgs::Tracker>();
            if (g != NULL){
                trackers.insert(g);
                if(g->name.size() > 0)
                    (*trackers.find(g))->name = g->name;
            }
        }
        bag.close();
    }
    catch(const std::exception& e){
        ROS_ERROR("%s", e.what());
    }
    ROS_INFO("nb trackers loaded: %lu ", trackers.size());

}

void save(const std::string& filename, TrackersSet const& trackers)
{
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Write);
    long unsigned int total = 0;
    for(auto tck: trackers){
        try{
            if(tck->name.size()==0)continue;
            bag.write("/tracking/ended_tracker", ros::Time::now(), *tck);
            total++;
        }catch(const std::exception& e){
            ROS_ERROR("%s", e.what());
            bag.close();
            return;
        }
    }
    bag.close();

    ROS_INFO("Saved %lu trackers to %s", total, filename.c_str());
}



int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));

    fs::path someDir("/home/robotic/Desktop/datasets/trackers");
    fs::directory_iterator end_iter;
    typedef std::set<fs::path> result_set_t;
    result_set_t result_set;


    if ( fs::exists(someDir) && fs::is_directory(someDir))
    {
        for( fs::directory_iterator dir_iter(someDir) ; dir_iter != end_iter ; ++dir_iter)
        {
            if (fs::is_regular_file(dir_iter->status()) )
            {
                result_set.insert(*dir_iter);
            }
        }
    }
    TrackersSet trackers;
    for(auto it: result_set){
        load(it.string(), trackers);
    }

    for(auto it=trackers.begin(); it!=trackers.end();)
    {
        sn_msgs::TrackerPtr tk = *it;
        //filters
        /*
        if(tk->name == "noise"){
            std::cout << "Erasing: " << " " << tk->uid << std::endl;
            it = trackers.erase(it);
            continue;
        }
        */

        if(sn::l2_norm(tk->detections.front().robot-tk->detections.back().robot) < 1.0){
          std::cout << "Erasing: " << " " << tk->uid << std::endl;
          it = trackers.erase(it);
          continue;
        }

        if(tk->detections.size() < 150){
            std::cout << "Erasing: " << " " << tk->uid << std::endl;
            it = trackers.erase(it);
            continue;
        }

        bool has_laser = false;
        bool has_kinect = false;
        for(auto det: tk->detections){
            if(det.points.size() > 0)
                has_laser = true;
            if(det.cloud.data.size() > 0)
                has_kinect = true;
        }
        if(!has_laser || !has_kinect){
            std::cout << "Erasing: " << " " << tk->uid << std::endl;
            it = trackers.erase(it);
            continue;
        }
        ++it;
    }

    ROS_INFO("nb trackers loaded: %lu ", trackers.size());


    ros::Publisher path_pub = handle.advertise<nav_msgs::Path>("/path",1,true);
    ros::Publisher cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("/laser_cloud",1,true);
    ros::Publisher kinect_pub = handle.advertise<sensor_msgs::PointCloud2>("/kinect_cloud",1,true);
    ros::Publisher poses_pub = handle.advertise<geometry_msgs::PoseArray>("/poses", 1, true);
    for(auto it=trackers.begin(); it!=trackers.end();){
        sn_msgs::TrackerPtr tk = *it;
        nav_msgs::Path path;
        std::cout << tk->uid << std::endl;
        std::cout << tk->name << std::endl;
        std::cout << tk->detections.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB> ptcld;
        pcl::PointCloud<pcl::PointXYZRGB> ptcld2;
        geometry_msgs::PoseArray parray;
        bool has_laser = false;
        bool has_kinect = false;
        for(auto det: tk->detections){
            //Path
            geometry_msgs::PoseStamped pose;
            tf::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, det.robot.theta);
            quaternion.normalize();
            tf::quaternionTFToMsg( quaternion, pose.pose.orientation);
            pose.pose.position.x = det.robot.x;
            pose.pose.position.y = det.robot.y;
            pose.pose.position.z = 0.0;
            pose.header = det.header;
            path.poses.push_back(pose);

            //cloud
            for(sn::point_t p: det.points){
                pcl::PointXYZRGB tmp;
                tmp.x = p.x;
                tmp.y = p.y;
                tmp.z = p.z;
                ptcld.push_back(tmp);
                has_laser = true;
            }

            //pose array
            parray.poses.push_back(pose.pose);

            //kinect
            if(det.cloud.data.size() > 0){
                pcl::PointCloud<pcl::PointXYZRGB> tmp;
                pcl::fromROSMsg(det.cloud, tmp);
                ptcld2.insert(ptcld2.end(), tmp.begin(), tmp.end());
                has_kinect = true;
            }

        }
        path.header = tk->header;
        path_pub.publish(path);
        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg(ptcld, cloud);
        cloud.header = tk->header;
        cloud_pub.publish(cloud);
        parray.header = tk->header;
        poses_pub.publish(parray);
        ptcld2.insert(ptcld2.end(), ptcld.begin(), ptcld.end());
        pcl::toROSMsg(ptcld2, cloud);
        cloud.header = tk->header;
        kinect_pub.publish(cloud);



        std::string mystr;
        try {
            std::cout << "Press enter to continue. 1 to delete. 3 exit.\n";
            std::getline (std::cin, mystr);
            int x = boost::lexical_cast<int>( mystr );

            if(x == 1){
                std::cout << "Erasing: " << " " << tk->uid << std::endl;
                it = trackers.erase(it);
                continue;
            }

            if(x == 3)
                break;

        } catch( boost::bad_lexical_cast const& ) {
            if(mystr.size() > 0){
                std::cout << "New name: " << mystr  <<  std::endl;
                tk->name = mystr;
            }
        }
        ++it;

    }
    save("/home/robotic/Desktop/datasets/trackers/all.bag", trackers);

    return 0;
}
