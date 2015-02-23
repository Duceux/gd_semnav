#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sn_msgs/DetectionArray.h>
#include <sn_msgs/DescriptorArray.h>
#include "sn_features/pfh_shape_reco.h"

template <typename T>
class PFHPublisherNode : public PFHShapeReco<T>
{
  typedef boost::shared_ptr<pcl::PointCloud<T> > CloudPtr;
  typedef boost::shared_ptr<const pcl::PointCloud<T> > ConstCloudPtr;

public:
  PFHPublisherNode(ros::NodeHandle n);
  virtual ~PFHPublisherNode();

  void LoadROSParams();
  bool srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void cb_VPtCldSegObjects(const sn_msgs::DetectionArrayConstPtr& msg_ptr_v_objects_clouds);

private:
  ros::Publisher pub_v_PFH;
  ros::Subscriber sub_v_objects_clouds;
  ros::ServiceServer srv_load_ros_params;
};

template <typename T>
PFHPublisherNode<T>::PFHPublisherNode(ros::NodeHandle n)
{
  pub_v_PFH = n.advertise<sn_msgs::DescriptorArray>("v_objects_pfh", 1);
  sub_v_objects_clouds = n.subscribe<sn_msgs::DetectionArray>("/pt_cld_segmentation_node/v_objects_clouds", 1, &PFHPublisherNode<T>::cb_VPtCldSegObjects, this);

  srv_load_ros_params = n.advertiseService("load_ros_params", &PFHPublisherNode<T>::srv_LoadRosParams, this);

  LoadROSParams();
  ROS_INFO_STREAM(n.getNamespace() << " spin!");
}

template <typename T>
PFHPublisherNode<T>::~PFHPublisherNode()
{
}

template <typename T>
void PFHPublisherNode<T>::LoadROSParams()
{
  ros::param::param<int>("~nbrPointsPairInHist", this->_nbrPointsPairInHist, this->_nbrPointsPairInHist);
  ros::param::param<int>("~nbrBinsAngles", this->_nbrBinsAngles, this->_nbrBinsAngles);
  ros::param::param<int>("~nbrBinsDist", this->_nbrBinsDist, this->_nbrBinsDist);
  this->SetParams();
}

template <typename T>
bool PFHPublisherNode<T>::srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  LoadROSParams();
  return true;
}

template <typename T>
void PFHPublisherNode<T>::cb_VPtCldSegObjects(const sn_msgs::DetectionArrayConstPtr &msg_ptr_v_objects_clouds)
{
  size_t nbObjects = msg_ptr_v_objects_clouds->detections.size();

  sn_msgs::DescriptorArray msg_v_pfh;
  msg_v_pfh.header = msg_ptr_v_objects_clouds->header;
  msg_v_pfh.descriptors.resize(nbObjects);

  for (size_t i = 0; i < nbObjects; ++i)
  {
    CloudPtr ptCld(new pcl::PointCloud<T>);
    pcl::fromROSMsg(msg_ptr_v_objects_clouds->detections[i].cloud, *ptCld);
    pcl::PointCloud<pcl::Normal>::Ptr nCldPtr(new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(msg_ptr_v_objects_clouds->detections[i].normals, *nCldPtr);

    this->ComputePFH(ptCld, nCldPtr);
    msg_v_pfh.descriptors[i].data = this->GetObjectModel();
  }

  if (nbObjects > 0)
  {
    pub_v_PFH.publish(msg_v_pfh);
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pfh_publisher_node");
  ros::NodeHandle n(std::string("~"));

  PFHPublisherNode<pcl::PointXYZRGB> node(n);
  ros::spin();

  return 0;
}
