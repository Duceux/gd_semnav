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
#include <sn_msgs/DescriptorSequence.h>
#include <visualization_msgs/MarkerArray.h>
#include <sn_dictionary/dico.h>
#include <sn_features/histogram_distance.h>

namespace fs = boost::filesystem;

struct compare_func{ bool operator()(sn_msgs::TrackerPtr const& l, sn_msgs::TrackerPtr const& r){return l->uid<r->uid;} };
typedef std::set<sn_msgs::TrackerPtr, compare_func> TrackersSet;

typedef sn_msgs::DescriptorSequence Seq;
typedef sn_msgs::Descriptor Des;
typedef std::vector<Seq> VSeq;

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
                if(g->name.size() != 0)
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

void save(const std::string& filename, VSeq const& trackers)
{
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Write);
    long unsigned int total = 0;
    for(auto tck: trackers){
        try{
            bag.write("sequence", ros::Time::now(), tck);
            total++;
        }catch(const std::exception& e){
            ROS_ERROR("%s", e.what());
            bag.close();
            return;
        }
    }
    bag.close();

    ROS_INFO("Saved %lu sequences to %s", total, filename.c_str());
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));


    TrackersSet trackers;
    load("/home/robotic/Desktop/datasets/trackers/all.bag", trackers);

    std::map<sn::Word, std::array<float, 3>> mColors;
    sn::Dictionary<sn::FastGetter> dicos;
    dicos.set("laser", 0.2, sn::Distance(sn::symmetric_chi2_distance));

    sn::TriangleLaserExtractor extractor;
    extractor.sampling_resolution = 0.001;
    extractor.smoothing_factor = 50;
    extractor.downsampling_factor = 50;
    extractor.theta_bin_size = M_PI*2.0/12.0;
    extractor.rho_bin_size = 0.1;

    ros::Publisher path_pub = handle.advertise<nav_msgs::Path>("/path",1,true);
    ros::Publisher cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("/laser_cloud",1,true);
    ros::Publisher kinect_pub = handle.advertise<sensor_msgs::PointCloud2>("/kinect_cloud",1,true);
    ros::Publisher poses_pub = handle.advertise<geometry_msgs::PoseArray>("/poses", 1, true);
    ros::Publisher marker_pub = handle.advertise<visualization_msgs::MarkerArray>("/words",1000,true);

    for(sn_msgs::TrackerPtr tk: trackers){
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
        sensor_msgs::PointCloud2 cloud, cloud2;
        pcl::toROSMsg(ptcld, cloud);
        cloud.header = tk->header;
        parray.header = tk->header;
        ptcld2.insert(ptcld2.end(), ptcld.begin(), ptcld.end());
        pcl::toROSMsg(ptcld2, cloud2);
        cloud2.header = tk->header;


        Seq seq;
        seq.tracker = tk->uid;
        seq.timestamp = tk->header.stamp;
        seq.name = tk->name;
        pcl::toROSMsg(ptcld2, seq.cloud);
        seq.cloud.header = tk->header;

        //descriptors
        for(sn_msgs::Detection det: tk->detections){
            if(det.points.size() > 0){
                Des des;
                des.type = "laser";
                des.data = extractor(det.points);
                des.tracker = seq.tracker;
                des.timestamp = det.header.stamp;
                des.robot = det.robot;
                seq.descriptors.push_back(des);
            }
        }
        //marker
        visualization_msgs::MarkerArray marray;
        for(sn_msgs::Descriptor des: seq.descriptors){
            sn::Word w = dicos.get(des);

            visualization_msgs::Marker marker;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            auto label = w;
            if(mColors.count( label ) == 0){
                static std::default_random_engine generator;
                static std::uniform_real_distribution<float> distribution(0.f,1.f);
                static auto random = std::bind ( distribution, generator );
                mColors[label][0] = random();
                mColors[label][1] = random();
                mColors[label][2] = random();
            }
            marker.pose.position.x = des.robot.x;
            marker.pose.position.y = des.robot.y;
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.01;
            marker.color.r = mColors[label][0];
            marker.color.g = mColors[label][1];
            marker.color.b = mColors[label][2];
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration();
            marker.header.frame_id = tk->header.frame_id;
            marker.ns = "cylinder";
            marker.id = marray.markers.size();
            marray.markers.push_back(marker);
        }
        static std::size_t max_size = marray.markers.size();
        max_size = std::max(marray.markers.size(), max_size);
        for(int i=marray.markers.size(); i<max_size; ++i){
            visualization_msgs::Marker marker;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::DELETE;
            marker.lifetime = ros::Duration();
            marker.header.frame_id = tk->header.frame_id;
            marker.ns = "cylinder";
            marker.id = marray.markers.size();
            marray.markers.push_back(marker);
        }

        marker_pub.publish(marray);
        cloud_pub.publish(cloud);
        path_pub.publish(path);
        poses_pub.publish(parray);
        kinect_pub.publish(cloud2);

        std::string mystr;
        std::cout << "Press enter to continue\n";
        std::getline (std::cin, mystr);
    }

    return 0;
}
