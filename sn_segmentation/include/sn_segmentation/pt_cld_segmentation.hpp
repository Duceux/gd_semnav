template <typename T>
PtCldSegmentation<T>::PtCldSegmentation(std::string strFileFloorPlane = "floor.dat", float fltRANSACFloorDistThresh = 0.05, float fltNEMaxDepth = 0.02, float fltNESmoothing = 10.0, float fltMaxDistFromKinect = 3.0, float fltCBFloorMaxPlaneDist = 2.0, float fltCBFloorMinPlaneDist = 0.025, float fltFloorAngleThresh = 0.98, float fltFloorDistThresh = 0.05, float fltRANSACWallsDistThresh = 0.05, std::size_t intMinNbrPointInWall = 10000, float fltWallAngleThresh = 0.05, float fltECDistance = 0.04, int intECMinSize = 200, int intECMaxSize = 1000000, int int2dBorderTolerance = 5, double flt3dBorderTolerance = 0.05)
  : debug_(1),
    strFileFloorPlane_(strFileFloorPlane),
    fltRANSACFloorDistThresh_(fltRANSACFloorDistThresh),
    fltNEMaxDepth_(fltNEMaxDepth),
    fltNESmoothing_(fltNESmoothing),
    fltMaxDistFromKinect_(fltMaxDistFromKinect),
    fltCBFloorMaxPlaneDist_(fltCBFloorMaxPlaneDist),
    fltCBFloorMinPlaneDist_(fltCBFloorMinPlaneDist),
    fltFloorAngleThresh_(fltFloorAngleThresh),
    fltFloorDistThresh_(fltFloorDistThresh),
    fltRANSACWallsDistThresh_(fltRANSACWallsDistThresh),
    intMinNbrPointInWall_(intMinNbrPointInWall),
    fltWallAngleThresh_(fltWallAngleThresh),
    fltECDistance_(fltECDistance),
    intECMinSize_(intECMinSize),
    intECMaxSize_(intECMaxSize),
    int2dBorderTolerance_(int2dBorderTolerance),
    flt3dBorderTolerance_(flt3dBorderTolerance),
    fltFocalConstantX_(525),
    fltFocalConstantY_(525),
    fltCenterX_(319.5),
    fltCenterY_(249.5),
    coefsPtrRANSACFloor_(new pcl::ModelCoefficients),
    sacFloorPtr(new pcl::SACSegmentation<T>),
    cbMaxDistPtr_(new pcl::CropBox<T>),
    vgPtr_(new pcl::VoxelGrid<T>),
    iinePtr_(new pcl::IntegralImageNormalEstimation<T, pcl::Normal>),
    cbFloorPtr_(new pcl::CropBox<T>),
    sacWallsPtr_(new pcl::SACSegmentation<T>),
    projPtr_(new pcl::ProjectInliers<T>),
    ecPtr_(new pcl::EuclideanClusterExtraction<T>),
    hullPtr_(new pcl::ConvexHull<T>),
    chPtr_(new pcl::CropHull<T>),
    bFloorPlaneEstimationDone(false)
{
  // Set parameters of PCL processing objects
  SetParams();
}

template <typename T>
PtCldSegmentation<T>::~PtCldSegmentation()
{
}

template <typename T>
void PtCldSegmentation<T>::SetParams()
{
  // Load stored floor plane coefs
  coefsPtrRANSACFloor_->values.resize(4);
  std::ifstream coefs_file(this->strFileFloorPlane_.c_str(), std::ios::in | std::ios::binary);
  if (coefs_file.is_open())
  {
    coefs_file.read( (char*) &this->coefsPtrRANSACFloor_->values[0], sizeof(this->coefsPtrRANSACFloor_->values[0]) );
    coefs_file.read( (char*) &this->coefsPtrRANSACFloor_->values[1], sizeof(this->coefsPtrRANSACFloor_->values[1]) );
    coefs_file.read( (char*) &this->coefsPtrRANSACFloor_->values[2], sizeof(this->coefsPtrRANSACFloor_->values[2]) );
    coefs_file.read( (char*) &this->coefsPtrRANSACFloor_->values[3], sizeof(this->coefsPtrRANSACFloor_->values[3]) );
    coefs_file.close();
    this->v4fCoefsFloor_[0] = this->coefsPtrRANSACFloor_->values[0];
    this->v4fCoefsFloor_[1] = this->coefsPtrRANSACFloor_->values[1];
    this->v4fCoefsFloor_[2] = this->coefsPtrRANSACFloor_->values[2];
    this->v4fCoefsFloor_[3] = this->coefsPtrRANSACFloor_->values[3];
    bFloorPlaneEstimationDone = true;
  }

  sacFloorPtr.reset(new pcl::SACSegmentation<T>);
  sacFloorPtr->setOptimizeCoefficients(true);
  sacFloorPtr->setModelType(pcl::SACMODEL_PLANE);
  sacFloorPtr->setMethodType(pcl::SAC_RANSAC);
  sacFloorPtr->setDistanceThreshold(this->fltRANSACFloorDistThresh_);

  // Maximum distance cropbox filter
  cbMaxDistPtr_.reset(new pcl::CropBox<T>(debug_));
  Eigen::Vector4f v4fCbMaxDistMax;
  v4fCbMaxDistMax[0] = fltMaxDistFromKinect_;
  v4fCbMaxDistMax[1] = fltMaxDistFromKinect_;
  v4fCbMaxDistMax[2] = fltMaxDistFromKinect_;
  v4fCbMaxDistMax[3] = 1.;
  cbMaxDistPtr_->setMax(v4fCbMaxDistMax);
  Eigen::Vector4f v4fCbMaxDistMin;
  v4fCbMaxDistMin[0] = -fltMaxDistFromKinect_;
  v4fCbMaxDistMin[1] = -fltMaxDistFromKinect_;
  v4fCbMaxDistMin[2] = -fltMaxDistFromKinect_;
  v4fCbMaxDistMin[3] = 1.;
  cbMaxDistPtr_->setMin(v4fCbMaxDistMin);

  // Voxel grid
  vgPtr_.reset(new pcl::VoxelGrid<T>);
  vgPtr_->setLeafSize(fltVGLeafSize_, fltVGLeafSize_, fltVGLeafSize_);

  // Normal estimation
  iinePtr_.reset(new pcl::IntegralImageNormalEstimation<T, pcl::Normal>);
  iinePtr_->setNormalEstimationMethod(iinePtr_->AVERAGE_3D_GRADIENT);
  iinePtr_->setMaxDepthChangeFactor(fltNEMaxDepth_);
  iinePtr_->setNormalSmoothingSize(fltNESmoothing_);

  // Floor cropbox filter
  cbFloorPtr_.reset(new pcl::CropBox<T>(debug_));
  Eigen::Vector4f v4fCbFloorMax;
  v4fCbFloorMax[0] = std::numeric_limits<float>::max();
  v4fCbFloorMax[1] = fltCBFloorMaxPlaneDist_;
  v4fCbFloorMax[2] = std::numeric_limits<float>::max();
  v4fCbFloorMax[3] = 1.;
  cbFloorPtr_->setMax(v4fCbFloorMax);
  Eigen::Vector4f v4fCbFloorMin;
  v4fCbFloorMin[0] = -std::numeric_limits<float>::max();
  v4fCbFloorMin[1] = fltCBFloorMinPlaneDist_;
  v4fCbFloorMin[2] = -std::numeric_limits<float>::max();
  v4fCbFloorMin[3] = 1.;
  cbFloorPtr_->setMin(v4fCbFloorMin);

  // Walls RANSAC
  sacWallsPtr_.reset(new pcl::SACSegmentation<T>);
  sacWallsPtr_->setModelType(pcl::SACMODEL_PLANE); // TODO: Try SACMODEL PERPENDICULAR/PARALLEL ...
  sacWallsPtr_->setMethodType(pcl::SAC_RANSAC);
  sacWallsPtr_->setOptimizeCoefficients(true);
  sacWallsPtr_->setDistanceThreshold(fltRANSACWallsDistThresh_);

  // Projection
  projPtr_.reset(new pcl::ProjectInliers<T>);
  projPtr_->setModelType(pcl::SACMODEL_PLANE);
  projPtr_->setCopyAllData(true);

  // Euclidean clustering
  ecPtr_.reset(new pcl::EuclideanClusterExtraction<T>);
  TreePtr treePtr(new pcl::search::KdTree<T>);
  ecPtr_->setSearchMethod(treePtr);
  ecPtr_->setClusterTolerance(fltECDistance_);
  ecPtr_->setMinClusterSize(intECMinSize_);
  ecPtr_->setMaxClusterSize(intECMaxSize_);

  hullPtr_.reset(new pcl::ConvexHull<T>);

  chPtr_.reset(new pcl::CropHull<T>);
}

template <typename T>
void PtCldSegmentation<T>::SetInputCloud(const ConstCloudPtr& cldPtrIn)
{
  cldPtrInput_ = cldPtrIn;
}

template <typename T>
bool PtCldSegmentation<T>::EstimateMainPlaneCoefs()
{
  if (this->cldPtrInput_ != NULL)
  {
    pcl::PointIndices::Ptr tmp(new pcl::PointIndices);

    sacFloorPtr->setInputCloud(this->cldPtrInput_);
    sacFloorPtr->segment(*tmp, *this->coefsPtrRANSACFloor_);
    // Make sure plane normal points towards the kinect
    if (this->coefsPtrRANSACFloor_->values[1] > 0)
    {
      this->v4fCoefsFloor_[0] = -this->coefsPtrRANSACFloor_->values[0];
      this->v4fCoefsFloor_[1] = -this->coefsPtrRANSACFloor_->values[1];
      this->v4fCoefsFloor_[2] = -this->coefsPtrRANSACFloor_->values[2];
      this->v4fCoefsFloor_[3] = -this->coefsPtrRANSACFloor_->values[3];
    }
    else
    {
      this->v4fCoefsFloor_[0] = this->coefsPtrRANSACFloor_->values[0];
      this->v4fCoefsFloor_[1] = this->coefsPtrRANSACFloor_->values[1];
      this->v4fCoefsFloor_[2] = this->coefsPtrRANSACFloor_->values[2];
      this->v4fCoefsFloor_[3] = this->coefsPtrRANSACFloor_->values[3];
    }

    std::cout << "Main plane coefficients: " << this->v4fCoefsFloor_[0] << ", " << this->v4fCoefsFloor_[1] << ", " << this->v4fCoefsFloor_[2] << ", " << this->v4fCoefsFloor_[3] << "." << std::endl;
    // Store floor plane coefs
    std::ofstream coefs_file(this->strFileFloorPlane_.c_str(), std::ios::out | std::ios::binary );
    if (coefs_file.is_open())
    {
      coefs_file.write( (const char*) &this->coefsPtrRANSACFloor_->values[0], sizeof(this->coefsPtrRANSACFloor_->values[0]) );
      coefs_file.write( (const char*) &this->coefsPtrRANSACFloor_->values[1], sizeof(this->coefsPtrRANSACFloor_->values[1]) );
      coefs_file.write( (const char*) &this->coefsPtrRANSACFloor_->values[2], sizeof(this->coefsPtrRANSACFloor_->values[2]) );
      coefs_file.write( (const char*) &this->coefsPtrRANSACFloor_->values[3], sizeof(this->coefsPtrRANSACFloor_->values[3]) );
      coefs_file.close();
      std::cout << "Stored in file: " << this->strFileFloorPlane_.c_str() << std::endl;
    }

    bFloorPlaneEstimationDone = true;
    return true;
  }
  else
    return false;
}

template <typename T>
void PtCldSegmentation<T>::RemoveMaxDistPoints(const ConstCloudPtr& cldPtrIn, pcl::PointIndices::Ptr& indPtrOut)
{
  cbMaxDistPtr_->setInputCloud(cldPtrIn);
  cbMaxDistPtr_->filter(indPtrOut->indices);

  // Debug
  if (debug_ == 1)
  {
    pcl::PointIndices indRemoved;
    cbMaxDistPtr_->getRemovedIndices(indRemoved);
    for (std::size_t posRem = 0; posRem < indRemoved.indices.size(); ++posRem)
    {
      cldPtrDebug_->points[indRemoved.indices[posRem]].r=255;
      cldPtrDebug_->points[indRemoved.indices[posRem]].g=255;
      cldPtrDebug_->points[indRemoved.indices[posRem]].b=255;
    }
  }
}

template <typename T>
void PtCldSegmentation<T>::ComputeNormals(const ConstCloudPtr& cldPtrIn, pcl::PointCloud<pcl::Normal>::Ptr& nCldPtrOut)
{
  iinePtr_->setInputCloud(cldPtrIn);
  iinePtr_->compute(*nCldPtrOut);
}

template <typename T>
void PtCldSegmentation<T>::RemoveFloor(const ConstCloudPtr& cldPtrIn, const pcl::PointCloud<pcl::Normal>::ConstPtr& nCldPtrIn, const pcl::PointIndices::ConstPtr& indicesIn, pcl::PointIndices::Ptr& indicesOut)
{
  // Find points whose position and normal is close to floor plane estimate from the RANSAC
  Eigen::Vector3f v3fRANSACFloorNormal(coefsPtrRANSACFloor_->values[0], coefsPtrRANSACFloor_->values[1], coefsPtrRANSACFloor_->values[2]);
  pcl::PointIndices::Ptr indicesPtrFloor(new pcl::PointIndices);
  indicesPtrFloor->indices.resize(indicesIn->indices.size());
  std::size_t j = 0;
  for (std::size_t i = 0; i < indicesIn->indices.size(); i++)
  {
    Eigen::Vector3f pointNormal(nCldPtrIn->points[indicesIn->indices[i]].normal_x,nCldPtrIn->points[indicesIn->indices[i]].normal_y,nCldPtrIn->points[indicesIn->indices[i]].normal_z);
    double dotFloorPoint = fabs(v3fRANSACFloorNormal.dot(pointNormal));
    if (dotFloorPoint > fltFloorAngleThresh_)
    {
      Eigen::Vector3f pointPos(cldPtrIn->points[indicesIn->indices[i]].x, cldPtrIn->points[indicesIn->indices[i]].y, cldPtrIn->points[indicesIn->indices[i]].z);
      double distPoint = v3fRANSACFloorNormal.dot(pointPos); // Distance from kinect to the point, along the floor plane's normal
      double distPlane = coefsPtrRANSACFloor_->values[3]; // Distance from kinect to floor, along the floor plane's normal
      if (fabs(distPlane + distPoint < fltFloorDistThresh_))
      {
        indicesPtrFloor->indices[j] = indicesIn->indices[i];
        j++;
      }
    }
  }
  indicesPtrFloor->indices.resize(j);

  // Estimate floor plane's normal
  if (indicesPtrFloor->indices.size() > intFloorMinNbrPoints_)
  {
    float tmp;
    pcl::computePointNormal(*cldPtrIn, indicesPtrFloor->indices, v4fCoefsFloor_, tmp);
    // Make sure plane normal points towards the kinect
    if (v4fCoefsFloor_[1] > 0)
    {
      v4fCoefsFloor_ = -v4fCoefsFloor_;
    }
  }

  // Set CropBox filter with estimated plane coefficients
  Eigen::Vector3f v3fRot;
  v3fRot[0] = atan2(static_cast<double>(v4fCoefsFloor_[2]),sqrt(pow(static_cast<double>(v4fCoefsFloor_[0]),2)+pow(static_cast<double>(v4fCoefsFloor_[1]),2)));
  v3fRot[1] = 0;
  v3fRot[2] = atan2(static_cast<double>(-v4fCoefsFloor_[0]),static_cast<double>(v4fCoefsFloor_[1]));
  cbFloorPtr_->setRotation(v3fRot);
  Eigen::Vector3f v3fTransl;
  v3fTransl[0] = -v4fCoefsFloor_[0]*v4fCoefsFloor_[3];
  v3fTransl[1] = -v4fCoefsFloor_[1]*v4fCoefsFloor_[3];
  v3fTransl[2] = -v4fCoefsFloor_[2]*v4fCoefsFloor_[3];
  cbFloorPtr_->setTranslation(v3fTransl);
  // This transforms from the kinect frame to a frame lying on the ground right underneath the kinect and such that the new z-axis is aligned with the kinect z-axis but parallel to the ground, the x-axis points to the left and the y-axis upwards.
  Eigen::Affine3f a3fKinectToRobot = pcl::getTransformation(0.,v4fCoefsFloor_[3],-0.2,v3fRot[0],v3fRot[1],v3fRot[2]);

  // Remove plane
  cbFloorPtr_->setInputCloud(cldPtrIn);
  cbFloorPtr_->setIndices(indicesIn);
  cbFloorPtr_->filter(indicesOut->indices);

  // Debug
  if (debug_ == 1)
  {
    pcl::PointIndices indRemoved;
    cbFloorPtr_->getRemovedIndices(indRemoved);
    for (std::size_t posRem = 0; posRem < indRemoved.indices.size(); ++posRem)
    {
      cldPtrDebug_->points[indicesIn->indices[indRemoved.indices[posRem]]].r=255;
      cldPtrDebug_->points[indicesIn->indices[indRemoved.indices[posRem]]].g=0;
      cldPtrDebug_->points[indicesIn->indices[indRemoved.indices[posRem]]].b=0;
    }
  }
}

template <typename T>
void PtCldSegmentation<T>::RemoveWalls(const ConstCloudPtr& cldPtrIn,
                                       const pcl::PointCloud<pcl::Normal>::ConstPtr& nCldPtrIn,
                                       const pcl::PointIndices::ConstPtr& indicesIn,
                                       pcl::PointIndices::ConstPtr& indicesOut)
{
  sacWallsPtr_->setInputCloud(cldPtrIn);

  pcl::PointIndices::ConstPtr indicesPtrWoPlanes = indicesIn;
  pcl::PointIndices::ConstPtr indicesPtrWoWalls = indicesIn;

  // while big planes are found
  std::size_t nbrInliersPlane;
  int nbWalls = 1;
  do
  {
    // Find a plane in the point cloud
    pcl::ModelCoefficients::Ptr coefsPtrPlane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indPtrRANSACPlane(new pcl::PointIndices);
    sacWallsPtr_->setIndices(indicesPtrWoPlanes);
    sacWallsPtr_->segment(*indPtrRANSACPlane, *coefsPtrPlane);
    if (coefsPtrPlane->values.size() > 0)
    {
      Eigen::Vector3f v3fNormalPlane(coefsPtrPlane->values[0], coefsPtrPlane->values[1], coefsPtrPlane->values[2]);
      nbrInliersPlane = indPtrRANSACPlane->indices.size();

      // Remove the plane from indicesPtrWoPlanes if it is big enough
      if (nbrInliersPlane > static_cast<std::size_t>(intMinNbrPointInWall_))
      {
        // Remove plane's point indices from indicesPtrWoPlanes
        pcl::PointIndices::Ptr indicesPtrTmp(new pcl::PointIndices);
        indicesPtrTmp->indices.resize(indicesPtrWoPlanes->indices.size()-nbrInliersPlane);
        std::size_t posTmp = 0;
        std::size_t posWo = 0;
        for (std::size_t posPlane = 0; posPlane < nbrInliersPlane; posPlane++)
        {
          while(indPtrRANSACPlane->indices[posPlane] != indicesPtrWoPlanes->indices[posWo++])
            indicesPtrTmp->indices[posTmp++] = indicesPtrWoPlanes->indices[posWo-1];
        }
        for (; posWo < indicesPtrWoPlanes->indices.size(); ++posWo)
          indicesPtrTmp->indices[posTmp++] = indicesPtrWoPlanes->indices[posWo];
        indicesPtrWoPlanes = indicesPtrTmp;

        // If it is perpendicular to the ground plane, save the plane's point indices
        Eigen::Vector3f v3fNormalFloor(v4fCoefsFloor_[0], v4fCoefsFloor_[1], v4fCoefsFloor_[2]);
        float dotFloorPlane = fabs(v3fNormalFloor.dot(v3fNormalPlane));
        if (dotFloorPlane < fltWallAngleThresh_)
        {
          // Remove plane's point indices from indicesPtrWoWalls
          pcl::PointIndices::Ptr indicesPtrTmp(new pcl::PointIndices);
          indicesPtrTmp->indices.resize(indicesPtrWoWalls->indices.size()-nbrInliersPlane);
          std::size_t posTmp = 0;
          std::size_t posWo = 0;
          for (std::size_t posPlane = 0; posPlane < nbrInliersPlane; ++posPlane)
          {
            while(indPtrRANSACPlane->indices[posPlane] != indicesPtrWoWalls->indices[posWo++])
              indicesPtrTmp->indices[posTmp++] = indicesPtrWoWalls->indices[posWo-1];
          }
          for (; posWo < indicesPtrWoWalls->indices.size(); ++posWo)
            indicesPtrTmp->indices[posTmp++] = indicesPtrWoWalls->indices[posWo];
          indicesPtrWoWalls = indicesPtrTmp;

          // Debug
          if (debug_ == 1)
          {
            for (std::size_t i = 0; i < indPtrRANSACPlane->indices.size(); ++i)
            {
              int32_t idx = indPtrRANSACPlane->indices[i];
              cldPtrDebug_->points[idx].r=0;
              cldPtrDebug_->points[idx].g=0;
              cldPtrDebug_->points[idx].b=255/nbWalls;
            }
            nbWalls++;
          }
        }
      }
    }
    else
      nbrInliersPlane = 0;
  }
  while(nbrInliersPlane > intMinNbrPointInWall_ && indicesPtrWoPlanes->indices.size() > intMinNbrPointInWall_);

  // Debug
  if (debug_ == 1)
  {
    for (std::size_t i = 0; i < indicesPtrWoWalls->indices.size(); ++i)
    {
      int32_t idx = indicesPtrWoWalls->indices[i];
      cldPtrDebug_->points[idx].r=0;
      cldPtrDebug_->points[idx].g=0;
      cldPtrDebug_->points[idx].b=0;
    }
    nbWalls++;
  }
  indicesOut = indicesPtrWoWalls;
}

template <typename T>
void PtCldSegmentation<T>::Voxelize(const ConstCloudPtr& cldPtrIn, const pcl::PointIndices::ConstPtr& indPtrIn, CloudPtr& cldPtrOut)
{
  vgPtr_->setInputCloud(cldPtrIn);
  vgPtr_->setIndices(indPtrIn);
  vgPtr_->filter(*cldPtrOut);
}

template <typename T>
void PtCldSegmentation<T>::ClusterObjects(const ConstCloudPtr& cldPtrIn, CloudPtr& cldPtrOut, std::vector<pcl::PointIndices::Ptr>& vIndPtrOut)
{
  // Project cloud on floor
  pcl::ModelCoefficients::Ptr coefsPtrFloorPlane(new pcl::ModelCoefficients);
  coefsPtrFloorPlane->values.push_back(static_cast<float>(v4fCoefsFloor_[0]));
  coefsPtrFloorPlane->values.push_back(static_cast<float>(v4fCoefsFloor_[1]));
  coefsPtrFloorPlane->values.push_back(static_cast<float>(v4fCoefsFloor_[2]));
  coefsPtrFloorPlane->values.push_back(static_cast<float>(v4fCoefsFloor_[3]));
  projPtr_->setModelCoefficients(coefsPtrFloorPlane);
  projPtr_->setInputCloud(cldPtrIn);
  projPtr_->filter(*cldPtrOut);

  // Cluster point cloud to find objects indices
  std::vector<pcl::PointIndices> vIndObjectsVox;
  ecPtr_->setInputCloud(cldPtrOut);
  ecPtr_->extract(vIndObjectsVox);

  // Make vector of PointIndices::Ptr
  std::size_t nbClusters = vIndObjectsVox.size();
  vIndPtrOut.resize(nbClusters);
  for (std::size_t i = 0; i < nbClusters; ++i)
  {
    pcl::PointIndices::Ptr indPtrVoxelObject(new pcl::PointIndices(vIndObjectsVox[i]));
    vIndPtrOut[i] = indPtrVoxelObject;
  }
}

template <typename T>
void PtCldSegmentation<T>::RemoveClustersCloseToBorder(
    const ConstCloudPtr& cldPtrIn,
    std::vector<pcl::PointIndices::Ptr>& vIndPtrIn,
    std::vector<pcl::PointIndices::Ptr>& vIndPtrOut)
{
  //  float focalConstantX = 580.0;
  //  float focalConstantY = 577.0;
  //  float centerX = 331.5;
  //  float centerY = 232.5;

  std::size_t nbClusters = vIndPtrIn.size();
  vIndPtrOut.reserve(nbClusters);
  for (std::size_t i = 0; i < nbClusters; ++i)
  {
    int uMin = 640-1, uMax = 0, vMin = 480-1, vMax = 0;
    std::size_t nbPoints = vIndPtrIn[i]->indices.size();
    for (std::size_t j = 0; j < nbPoints; ++j)
    {
      const T& point = cldPtrIn->points[vIndPtrIn[i]->indices[j]];
      int u = point.x*fltFocalConstantX_/point.z + fltCenterX_;
      int v = point.y*fltFocalConstantY_/point.z + fltCenterY_;

      uMin = std::min(u, uMin);
      uMax = std::max(u, uMax);
      vMin = std::min(v, vMin);
      vMax = std::max(v, vMax);
    }
    uMin = std::max(uMin, 0);
    uMax = std::min(uMax, 640-1);
    vMin = std::max(vMin, 0);
    vMax = std::min(vMax, 480-1);
    Eigen::Vector4f posMin, posMax;
    pcl::getMinMax3D(*cldPtrIn, *vIndPtrIn[i], posMin, posMax);

    // Keep cluster if it does not touch a border of the image or a border of the 3d max dist crop box (with tolerance)
    if ( uMin >= int2dBorderTolerance_
         && uMax < 640-int2dBorderTolerance_
         && vMin >= int2dBorderTolerance_
         && vMax < 480-int2dBorderTolerance_
         && posMin[0] >= -(fltMaxDistFromKinect_-fltVGLeafSize_-flt3dBorderTolerance_)
         && posMin[1] >= -(fltMaxDistFromKinect_-fltVGLeafSize_-flt3dBorderTolerance_)
         && posMin[2] >= -(fltMaxDistFromKinect_-fltVGLeafSize_-flt3dBorderTolerance_)
         && posMax[0] < (fltMaxDistFromKinect_-fltVGLeafSize_-flt3dBorderTolerance_)
         && posMax[1] < (fltMaxDistFromKinect_-fltVGLeafSize_-flt3dBorderTolerance_)
         && posMax[2] < (fltMaxDistFromKinect_-fltVGLeafSize_-flt3dBorderTolerance_)
         )
    {
      vIndPtrOut.push_back(vIndPtrIn[i]);
    }
  }
}

template <typename T>
void PtCldSegmentation<T>::UnVoxelize(std::vector<pcl::PointIndices::Ptr>& vIndPtrInVox, std::vector<pcl::PointIndices::Ptr>& vIndPtrOut)
{
  std::size_t nbClusters = vIndPtrInVox.size();
  vIndPtrOut.resize(nbClusters);

  for (std::size_t i = 0; i < nbClusters; ++i)
  {
    // find the number of points in the original point cloud that belong to this cluster
    std::size_t nbPoints = 0;
    std::size_t nbPointsVox = vIndPtrInVox[i]->indices.size();
    for (std::size_t j = 0; j < nbPointsVox; ++j)
    {
      nbPoints += vgPtr_->index_start_[vIndPtrInVox[i]->indices[j]+1] - vgPtr_->index_start_[vIndPtrInVox[i]->indices[j]];
    }
    // populate the output list of indices for this cluster
    pcl::PointIndices::Ptr indPtr(new pcl::PointIndices);
    indPtr->indices.reserve(nbPoints);
    for (std::size_t j = 0; j < nbPointsVox; ++j)
    {
      std::size_t start = vgPtr_->index_start_[vIndPtrInVox[i]->indices[j]];
      std::size_t end = vgPtr_->index_start_[vIndPtrInVox[i]->indices[j]+1];
      for (std::size_t k = start; k < end; ++k)
      {
        indPtr->indices.push_back(vgPtr_->index_vector_[k].cloud_point_index);
      }
    }
    vIndPtrOut[i] = indPtr;
  }
}

template <typename T>
void PtCldSegmentation<T>::MakeObjectsBoundingBoxesAndPos(
    const ConstCloudPtr& cldPtrIn,
    const std::vector<pcl::PointIndices::Ptr>& vIndPtrIn
    )
{
  std::size_t nbObjects = vIndPtrIn.size();
  v2dBBoxSegObjects_.resize(nbObjects);
  vPtCldPosAnd3dBBoxSegObjects_.resize(nbObjects);

  for (std::size_t i = 0; i < nbObjects; ++i)
  {
    // 2d bounding box
    v2dBBoxSegObjects_[i].uMin = 640-1;
    v2dBBoxSegObjects_[i].uMax = 0;
    v2dBBoxSegObjects_[i].vMin = 480-1;
    v2dBBoxSegObjects_[i].vMax = 0;

    // Find the bounding box
    std::size_t nbPoints = vIndPtrIn[i]->indices.size();
    for (std::size_t j = 0; j < nbPoints; ++j)
    {
      // Compute the point's u v position in image
      int u = vIndPtrIn[i]->indices[j] % 640;
      int v = vIndPtrIn[i]->indices[j] / 640;
      v2dBBoxSegObjects_[i].uMin = std::min(u, v2dBBoxSegObjects_[i].uMin);
      v2dBBoxSegObjects_[i].uMax = std::max(u, v2dBBoxSegObjects_[i].uMax);
      v2dBBoxSegObjects_[i].vMin = std::min(v, v2dBBoxSegObjects_[i].vMin);
      v2dBBoxSegObjects_[i].vMax = std::max(v, v2dBBoxSegObjects_[i].vMax);
    }

    // 3d bounding box
    pcl::getMinMax3D(*cldPtrIn, *vIndPtrIn[i], vPtCldPosAnd3dBBoxSegObjects_[i].min, vPtCldPosAnd3dBBoxSegObjects_[i].max);
    // Compute object 3d position
    pcl::compute3DCentroid(*cldPtrIn, *vIndPtrIn[i], vPtCldPosAnd3dBBoxSegObjects_[i].pos);
  }
}

template <typename T>
void PtCldSegmentation<T>::MakeObjectsImageAndMask(
    const std::vector<pcl::PointIndices::Ptr>& vIndPtrIn
    )
{
  std::size_t nbObjects = vIndPtrIn.size();
  vImageSegObjects_.resize(nbObjects);
  vMaskSegObjects_.resize(nbObjects);

  for (std::size_t i = 0; i < nbObjects; ++i)
  {
    // Size the mask
    int uSize = v2dBBoxSegObjects_[i].uMax-v2dBBoxSegObjects_[i].uMin+1;
    int vSize = v2dBBoxSegObjects_[i].vMax-v2dBBoxSegObjects_[i].vMin+1;
    vImageSegObjects_[i].resize(3*uSize*vSize);
    vMaskSegObjects_[i].assign(uSize*vSize, 0);

    // Mark object pixels in the mask
    for (std::size_t j = 0; j < vIndPtrIn[i]->indices.size(); ++j)
    {
      const T& point = cldPtrInput_->points[vIndPtrIn[i]->indices[j]];
      // Point cloud is organized so pixel position is tied to point indice
      int u = vIndPtrIn[i]->indices[j] % 640;
      int v = vIndPtrIn[i]->indices[j] / 640;

      int lImg = v-v2dBBoxSegObjects_[i].vMin, cImg = u-v2dBBoxSegObjects_[i].uMin;
      int posMask = lImg*uSize+cImg;
      vMaskSegObjects_[i][posMask] = 255;
    }
    // Populate image
    for (std::size_t j = v2dBBoxSegObjects_[i].uMin; j < v2dBBoxSegObjects_[i].uMax; ++j)
    {
      for (std::size_t k = v2dBBoxSegObjects_[i].vMin; k < v2dBBoxSegObjects_[i].vMax; ++k)
      {
        const T& point = cldPtrInput_->points[j+640*k];
        int posImg = 3* ( (j-v2dBBoxSegObjects_[i].uMin) + uSize*(k-v2dBBoxSegObjects_[i].vMin) );

        vImageSegObjects_[i][posImg] = point.b;
        vImageSegObjects_[i][posImg+1] = point.g;
        vImageSegObjects_[i][posImg+2] = point.r;
      }
    }
  }
  return;
}

template <typename T>
void PtCldSegmentation<T>::MakeObjectsPointAndNormalClouds(const ConstCloudPtr& cldPtrIn, const pcl::PointCloud<pcl::Normal>::ConstPtr& nCldPtrIn, const std::vector<pcl::PointIndices::Ptr>& vIndPtrIn)
{
  std::size_t nbObjects = vIndPtrIn.size();
  vCldPtrSegObjects_.resize(nbObjects);
  vNCldPtrSegObjects_.resize(nbObjects);

  // Create point cloud, normals cloud, bounding box and mask for each object
  for (std::size_t i = 0; i < nbObjects; ++i)
  {
    // Make object point cloud from indices
    CloudPtr cldPtrTmp(new pcl::PointCloud<T>(*cldPtrIn, vIndPtrIn[i]->indices));
    vCldPtrSegObjects_[i] = cldPtrTmp;
    // Make object point cloud normals
    pcl::PointCloud<pcl::Normal>::Ptr nCldPtrTmp(new pcl::PointCloud<pcl::Normal>(*nCldPtrIn, vIndPtrIn[i]->indices));
    vNCldPtrSegObjects_[i] = nCldPtrTmp;

    // Debug
    if (debug_ == 1)
    {
      for (std::size_t j = 0; j < vIndPtrIn[i]->indices.size(); j++)
      {
        int32_t idx = vIndPtrIn[i]->indices[j];
        cldPtrDebug_->points[idx].r=0;
        cldPtrDebug_->points[idx].g=127+128/(i+1);
        cldPtrDebug_->points[idx].b=0;
      }
    }
  }
}

template <typename T>
void PtCldSegmentation<T>::Segment()
{
  if (debug_ == 10)
  {
    cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrInput_));
    for (size_t i = 0; i < cldPtrDebug_->size(); ++i)
    {
      T& point = cldPtrDebug_->points[i];
      int u = point.x*fltFocalConstantX_/point.z + fltCenterX_;
      int v = point.y*fltFocalConstantY_/point.z + fltCenterY_;
      //u = std::abs(std::min(0, std::max(640, u))-320);
      //v = std::abs(std::min(0, std::max(480, v))-240);
      if (u<0 || u>639 || v<0 || v>479)
      {
        point.r=0;
        point.g=0;
        point.b=0;
      }
      else
      {
        point.r=255;
        point.g=255;
        point.b=255;
      }
    }
  }
  else
  {
    if (debug_ == 1)
      cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrInput_));

    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr nCldPtrInput(new pcl::PointCloud<pcl::Normal>);
    ComputeNormals(cldPtrInput_, nCldPtrInput);

    // Remove points lying too far from the kinect
    pcl::PointIndices::Ptr indPtrCroped(new pcl::PointIndices);
    RemoveMaxDistPoints(cldPtrInput_, indPtrCroped);
    if (debug_ == 2)
      cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrInput_, indPtrCroped->indices));

    // Remove floor plane
    pcl::PointIndices::Ptr indPtrWoFloor(new pcl::PointIndices);
    RemoveFloor(cldPtrInput_, nCldPtrInput, indPtrCroped, indPtrWoFloor);
    if (debug_ == 3)
      cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrInput_, indPtrWoFloor->indices));

    // Remove walls
    pcl::PointIndices::ConstPtr indPtrWoWalls;
    RemoveWalls(cldPtrInput_, nCldPtrInput, indPtrWoFloor, indPtrWoWalls);
    if (debug_ == 4)
      cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrInput_, indPtrWoWalls->indices));

    // Voxelize point cloud
    CloudPtr cldPtrObjectsVox(new pcl::PointCloud<T>);
    Voxelize(cldPtrInput_, indPtrWoWalls, cldPtrObjectsVox);
    if (debug_ == 5)
      cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrObjectsVox));

    // Project on floor plane and cluster objects
    CloudPtr cldPtrProj(new pcl::PointCloud<T>);
    std::vector<pcl::PointIndices::Ptr> vIndPtrClustersVox;
    ClusterObjects(cldPtrObjectsVox, cldPtrProj, vIndPtrClustersVox);
    if (debug_ == 6)
      cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrProj));

    // Remove clusters too close to a border of the image or the max dist box
    std::vector<pcl::PointIndices::Ptr> vIndPtrObjectsVox;
    RemoveClustersCloseToBorder(cldPtrObjectsVox, vIndPtrClustersVox, vIndPtrObjectsVox);

    // Extract from cldPtrInput_ the point indices of the objects
    std::vector<pcl::PointIndices::Ptr> vIndPtrObjects;
    UnVoxelize(vIndPtrObjectsVox, vIndPtrObjects);
    if (debug_ == 7)
    {
      pcl::PointIndices indObjects;
      std::size_t nbrIndices = 0;
      std::size_t nbObjects = vIndPtrObjects.size();
      for (std::size_t i = 0; i < nbObjects; ++i)
        nbrIndices += vIndPtrObjects[i]->indices.size();
      indObjects.indices.reserve(nbrIndices);
      for (std::size_t i = 0; i < nbObjects; ++i)
        indObjects.indices.insert(indObjects.indices.end(), vIndPtrObjects[i]->indices.begin(), vIndPtrObjects[i]->indices.end());
      cldPtrDebug_ = CloudPtr(new pcl::PointCloud<T>(*cldPtrInput_, indObjects.indices));
    }

    // Remove planar objects

    // Cluster superposed objects

    // Compute 2d- and 3d-bounding boxes
    MakeObjectsBoundingBoxesAndPos(cldPtrInput_, vIndPtrObjects);
    // Make output vectors of image and image mask
    MakeObjectsImageAndMask(vIndPtrObjects);
    // Make output vectors of point and normal cloud
    MakeObjectsPointAndNormalClouds(cldPtrInput_, nCldPtrInput, vIndPtrObjects);
  }
}

template<typename T>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PtCldSegmentation<T>::GetDebugCldPtr()
{
  return cldPtrDebug_;
}
