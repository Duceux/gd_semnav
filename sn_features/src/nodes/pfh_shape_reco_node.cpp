#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <common_msgs_srvs/v_features.h>
#include <common_msgs_srvs/label_and_score.h>
#include <common_msgs_srvs/v_labels_and_scores.h>
#include <common_msgs_srvs/id_name_in.h>

#include "sn_features/pfh_shape_reco.h"

template <typename T>
class PFHShapeRecoNode : public PFHShapeReco<T>
{
//  typedef boost::shared_ptr<pcl::PointCloud<T> > CloudPtr;
//  typedef boost::shared_ptr<const pcl::PointCloud<T> > ConstCloudPtr;

public:
  PFHShapeRecoNode(ros::NodeHandle n);
  virtual ~PFHShapeRecoNode();

  void LoadROSParams();
  bool srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srv_AddModelToDatabase(common_msgs_srvs::id_name_in::Request& req, common_msgs_srvs::id_name_in::Response& res);
  bool srv_SaveDatabase(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool srv_LoadDatabase(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void cb_VPFHObjects(const common_msgs_srvs::v_features::ConstPtr& msg_ptr_v_pfh);

private:
  ros::Publisher pub_v_labels_and_scores;
  ros::Subscriber subVPointFeatureHistogram;
  ros::ServiceServer srvLoadRosParams;
  ros::ServiceServer srvAddNextModelToDatabase;
  ros::ServiceServer srvSaveDatabase;
  ros::ServiceServer srvLoadDatabase;

  bool _bAddNextModel;
  std::string _sModelName;
  int _iModelToAdd;

  std::string _strDatabaseFile;
};

template <typename T>
PFHShapeRecoNode<T>::PFHShapeRecoNode(ros::NodeHandle n):
  _bAddNextModel(false),
  _strDatabaseFile("database.dat")
{
  pub_v_labels_and_scores = n.advertise<common_msgs_srvs::v_labels_and_scores>("/object_reco_node/v_objects_labels_and_scores", 0);
  subVPointFeatureHistogram = n.subscribe<common_msgs_srvs::v_features>("/pfh_publisher_node/v_objects_pfh", 0, &PFHShapeRecoNode<T>::cb_VPFHObjects, this);

  srvLoadRosParams = n.advertiseService("Load_Ros_Params", &PFHShapeRecoNode<T>::srv_LoadRosParams, this);
  srvAddNextModelToDatabase = n.advertiseService("Add_Model_To_Database", &PFHShapeRecoNode<T>::srv_AddModelToDatabase, this);
  srvSaveDatabase = n.advertiseService("Save_Database", &PFHShapeRecoNode<T>::srv_SaveDatabase, this);
  srvLoadDatabase = n.advertiseService("Load_Database", &PFHShapeRecoNode<T>::srv_LoadDatabase, this);

  LoadROSParams();
  ROS_INFO_STREAM(n.getNamespace() << " spin!");
}

template <typename T>
PFHShapeRecoNode<T>::~PFHShapeRecoNode()
{
}

template <typename T>
void PFHShapeRecoNode<T>::LoadROSParams()
{
  std::stringstream ss;
  ros::param::param<std::string>("~strDatabaseFile", _strDatabaseFile, _strDatabaseFile);
  ss << ros::package::getPath("classification") << "/" << _strDatabaseFile;
  _strDatabaseFile = ss.str();
  ros::param::param<int>("pfh_publisher_node/nbrPointsPairInHist", this->_nbrPointsPairInHist, this->_nbrPointsPairInHist);
  ros::param::param<int>("pfh_publisher_node/nbrBinsAngles", this->_nbrBinsAngles, this->_nbrBinsAngles);
  ros::param::param<int>("pfh_publisher_node/nbrBinsDist", this->_nbrBinsDist, this->_nbrBinsDist);
  this->SetParams();

  this->LoadDatabase(_strDatabaseFile);
  ROS_INFO("Database loaded!");
}

template <typename T>
bool PFHShapeRecoNode<T>::srv_LoadRosParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  LoadROSParams();
  return true;
}

template <typename T>
bool PFHShapeRecoNode<T>::srv_AddModelToDatabase(common_msgs_srvs::id_name_in::Request& req, common_msgs_srvs::id_name_in::Response& res)
{
  _bAddNextModel = true;
  _sModelName = req.name_in;
  _iModelToAdd = req.id_in;
  return true;
}

template <typename T>
bool PFHShapeRecoNode<T>::srv_SaveDatabase(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  this->SaveDatabase(_strDatabaseFile);
  return true;
}

template <typename T>
bool PFHShapeRecoNode<T>::srv_LoadDatabase(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  this->LoadDatabase(_strDatabaseFile);
  ROS_INFO("Database loaded!");
  return true;
}

template <typename T>
void PFHShapeRecoNode<T>::cb_VPFHObjects(const common_msgs_srvs::v_features::ConstPtr& msg_ptr_v_pfh)
{
  size_t nbObjects = msg_ptr_v_pfh->v_features.size();
  common_msgs_srvs::v_labels_and_scores msg_v_objects_label;
  msg_v_objects_label.header = msg_ptr_v_pfh->header;
  msg_v_objects_label.vLabelsAndScores.reserve(nbObjects);

  for (size_t i = 0; i < nbObjects; ++i)
  {
    cv_bridge::CvImageConstPtr PFHPtr = cv_bridge::toCvShare(msg_ptr_v_pfh->v_features[i], msg_ptr_v_pfh);
    this->SetObjectModel(std::vector<double>(PFHPtr->image));

    // Add model to database if requested
    if (_bAddNextModel && _iModelToAdd == i)
    {
      std::cout << "Model #" << _iModelToAdd << " saved with name '" << _sModelName << "'" << std::endl;
      this->AddModelToDatabase(_sModelName);
      _bAddNextModel = false;
    }

    this->IdentifyObject();
    common_msgs_srvs::label_and_score msg_labels_and_scores;
    msg_labels_and_scores.header = msg_ptr_v_pfh->v_features[i].header;
    msg_labels_and_scores.labels.reserve(1);
    msg_labels_and_scores.labels.push_back(this->object_label_);
    msg_labels_and_scores.scores.reserve(1);
    msg_labels_and_scores.scores.push_back(this->object_score_);
    msg_v_objects_label.vLabelsAndScores.push_back(msg_labels_and_scores);
  }

  if (nbObjects > 0)
  {
    pub_v_labels_and_scores.publish(msg_v_objects_label);
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pfh_shape_reco_node");
  ros::NodeHandle n(std::string("~"));

  PFHShapeRecoNode<pcl::PointXYZRGB> node(n);
  ros::spin();

  return 0;
}
