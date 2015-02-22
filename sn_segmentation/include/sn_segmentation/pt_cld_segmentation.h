#ifndef PT_CLD_SEGMENTATION_H
#define PT_CLD_SEGMENTATION_H

#include <fstream>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/voxel_grid.h>
#include "voxel_grid_fix.hpp"
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/filters/crop_box.h>
#include "crop_box_fix.hpp"
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

/*! \brief PtCldSegmentation class.
 *         Finds object in point clouds, assuming the camera is static relative to floor plane.
 *
 *         Removes points belonging to the floor plane and points lying on large planes perpendicular to the floor plane and clusters remaining objects.
 *         Provides a vector of the objects point clouds, normal point clouds, tight images and binary mask for the images.
 *  Usage: 1. Set protected parameter variables to desired values, then call SetParams()
 *         2. Call SetInputCloud() with desired point cloud to process
 *         3. (Once) call EstimateMainPlaneCoefs() to store the coefficients of the most represented plane in the point cloud (hopefully this will be the floor plane)
 *         4. Call Segment() and get the results in protected variables vCldPtrSegObjects_, vNCldPtrSegObjects_, vBBoxSegObjects_ and vMaskSegObjects_.
 */
template <typename T>
class PtCldSegmentation
{
  typedef boost::shared_ptr<pcl::PointCloud<T> > CloudPtr;
  typedef boost::shared_ptr<const pcl::PointCloud<T> > ConstCloudPtr;
  typedef boost::shared_ptr<pcl::search::KdTree<T> > TreePtr;

  typedef boost::shared_ptr<pcl::SACSegmentation<T> > SACSegmentationPtr;
  typedef boost::shared_ptr<pcl::VoxelGrid<T> > VoxelGridPtr;
  typedef boost::shared_ptr<pcl::CropBox<T> > CropBoxPtr;
  typedef boost::shared_ptr<pcl::IntegralImageNormalEstimation<T, pcl::Normal> > IntegralImageNormalEstimationPtr;
  typedef boost::shared_ptr<pcl::ProjectInliers<T> > ProjectInliersPtr;
  typedef boost::shared_ptr<pcl::EuclideanClusterExtraction<T> > EuclideanClusterExtractionPtr;
  typedef boost::shared_ptr<pcl::ConvexHull<T> > ConvexHullPtr;
  typedef boost::shared_ptr<pcl::CropHull<T> > CropHullPtr;

  struct ImageBBox
  {
    int uMin, uMax, vMin, vMax;
  };
  struct PtCldPosAndBBox
  {
    Eigen::Vector4f pos, min, max;
  };

public:
  PtCldSegmentation(std::string strFileFloorPlane, float fltRANSACFloorDistThresh_, float fltNEMaxDepth, float fltNESmoothing, float fltMaxDistFromKinect, float fltCBFloorMaxPlaneDist, float fltCBFloorMinPlaneDist, float fltFloorAngleThresh, float fltFloorDistThresh, float fltRANSACWallsDistThresh, size_t intMinNbrPointInWall, float fltWallAngleThresh, float fltECDistance, int intECMinSize, int intECMaxSize, int int2dBorderTolerance, double flt3dBorderTolerance);
  virtual ~PtCldSegmentation();

  bool EstimateMainPlaneCoefs();

  void SetParams();
  void SetInputCloud(const ConstCloudPtr& cldPtrIn);
  const ConstCloudPtr& GetInputCloud(){
    return cldPtrInput_;
  }
  void Segment();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetDebugCldPtr();

//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  // Parameters
  std::string strFileFloorPlane_;
  double fltRANSACFloorDistThresh_;
  double fltVGLeafSize_;
  double fltNEMaxDepth_;
  double fltNESmoothing_;
  int intFloorMinNbrPoints_;
  double fltCBFloorMaxPlaneDist_;
  double fltCBFloorMinPlaneDist_;
  double fltMaxDistFromKinect_;
  double fltFloorAngleThresh_;
  double fltFloorDistThresh_;
  double fltRANSACWallsDistThresh_;
  int intMinNbrPointInWall_;
  double fltWallAngleThresh_;
  double fltECDistance_;
  int intECMinSize_;
  int intECMaxSize_;
  int int2dBorderTolerance_;
  double flt3dBorderTolerance_;
  // Kinect Parameters
  double fltFocalConstantX_;
  double fltFocalConstantY_;
  double fltCenterX_;
  double fltCenterY_;
  // Debug
  int debug_;

  bool bFloorPlaneEstimationDone;

  // Vectors of objects information
  std::vector<struct PtCldPosAndBBox> vPtCldPosAnd3dBBoxSegObjects_;
  std::vector<CloudPtr> vCldPtrSegObjects_;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> vNCldPtrSegObjects_;
  std::vector<struct ImageBBox> v2dBBoxSegObjects_;
  std::vector<std::vector<unsigned char> > vImageSegObjects_;
  std::vector<std::vector<unsigned char> > vMaskSegObjects_;
protected:
  // Functions
  void RemoveMaxDistPoints(const ConstCloudPtr& cldPtrIn, pcl::PointIndices::Ptr& indPtrOut);
  void ComputeNormals(const ConstCloudPtr& cldPtrIn, pcl::PointCloud<pcl::Normal>::Ptr& nCldPtrOut);
  void RemoveFloor(const ConstCloudPtr& cldPtrIn, const pcl::PointCloud<pcl::Normal>::ConstPtr& nCldPtrIn, const pcl::PointIndices::ConstPtr& indPtrIn, pcl::PointIndices::Ptr& indPtrOut);
  void RemoveWalls(const ConstCloudPtr& cldPtrIn, const pcl::PointCloud<pcl::Normal>::ConstPtr& nCldPtrIn, const pcl::PointIndices::ConstPtr& indPtrIn, pcl::PointIndices::ConstPtr& indPtrOut);
  void Voxelize(const ConstCloudPtr& cldPtrIn, const pcl::PointIndices::ConstPtr& indPtrIn, CloudPtr& cldPtrOut);
  void ClusterObjects(const ConstCloudPtr& cldPtrIn, CloudPtr& cldPtrOut, std::vector<pcl::PointIndices::Ptr>& vIndOut);
  void RemoveClustersCloseToBorder(const ConstCloudPtr& cldPtrIn, std::vector<pcl::PointIndices::Ptr>& vIndPtrIn, std::vector<pcl::PointIndices::Ptr>& vIndPtrOut);
  void UnVoxelize(std::vector<pcl::PointIndices::Ptr>& vIndPtrInVox, std::vector<pcl::PointIndices::Ptr>& vIndPtrOut);
  void MakeObjectsBoundingBoxesAndPos(const ConstCloudPtr& cldPtrIn, const std::vector<pcl::PointIndices::Ptr>& vIndPtrIn);
  void MakeObjectsImageAndMask(const std::vector<pcl::PointIndices::Ptr>& vIndPtrIn);
  void MakeObjectsPointAndNormalClouds(const ConstCloudPtr& cldPtrIn, const pcl::PointCloud<pcl::Normal>::ConstPtr& nCldPtrIn, const std::vector<pcl::PointIndices::Ptr>& vIndPtrIn);

  // PCL processing objects
  SACSegmentationPtr sacFloorPtr;
  VoxelGridPtr vgPtr_;
  CropBoxPtr cbMaxDistPtr_;
  IntegralImageNormalEstimationPtr iinePtr_;
  CropBoxPtr cbFloorPtr_;
  SACSegmentationPtr sacWallsPtr_;
  ProjectInliersPtr projPtr_;
  EuclideanClusterExtractionPtr ecPtr_;
  ConvexHullPtr hullPtr_;
  CropHullPtr chPtr_;

  // Variables
  ConstCloudPtr cldPtrInput_;
  pcl::ModelCoefficients::Ptr coefsPtrRANSACFloor_;
  Eigen::Vector4f v4fCoefsFloor_;

  // Debug
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cldPtrDebug_;
};

#include "pt_cld_segmentation.hpp"

#endif // PT_CLD_SEGMENTATION_H
