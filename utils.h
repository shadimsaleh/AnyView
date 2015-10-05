#ifndef KOUROSH_TSDF_UTILS_H
#define KOUROSH_TSDF_UTILS_H

#include <vector>
#include <tuple>
#include <string>

// #include <engine-data/pointcloud.h>

#define UNUSED(x)  ((void)(x));

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
// #include <geometry-toolbox/transformation.h>
// #include <tango-hal/tango-hal.h>

// #include "engine-data/PointCloud.pb.h"

typedef float PointType;
  typedef int ColorType;

struct PointCloud {
  // PointCloud();
  // explicit PointCloud(const PointCloudProto& proto);

  // void FillProto(PointCloudProto* proto) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // TODO(idyanov): Can we save these as PLY properties?
  // size_t seq;
  // double timestamp;
  // double timestamp_android;

  // The sensor origin, w.r.t. to the same coordinate frame that the points are
  // expressed. This will typically be Identity, unless the point cloud has
  // been transformed.
  // geometry_toolbox::Transformation origin;
  // Eigen::Matrix<PointType, 3, 4> origin;
  Eigen::Matrix<float, 3, 3> rot;
  Eigen::Matrix<float, 3, 1> translation;


  // points is a 3xN matrix. Each column is an [x, y, z] point observation, in
  // meters.
  Eigen::Matrix<PointType, 3, Eigen::Dynamic> points;
  Eigen::Matrix<ColorType, 3, Eigen::Dynamic> colors;

  long num;

  // This will only be populated when using the HAL, and is here for recording
  // purposes.
  // std::shared_ptr<TangoHalPointCloud> tango_hal_point_cloud;
};

inline Eigen::Matrix<float, 3, 3> QuaternionToRotationMatrix(const Eigen::Matrix<float,4,1> &q);
std::vector<std::tuple<std::string, std::string>> GetPointCloudToColorImageMapFromFile(const std::string& filename);
std::vector<std::string> ReadFileLines(const std::string& filename);

PointCloud * ReadPointcloudFromFile(const std::string& filename);

bool CheckFileExists(const std::string& filename);


#endif //KOUROSH_TSDF_UTILS_H

