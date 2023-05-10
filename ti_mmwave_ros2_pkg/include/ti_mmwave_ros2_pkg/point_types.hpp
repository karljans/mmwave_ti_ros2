/** \file
 *
 *  Customized Point Cloud Library point structures for mmWave1843Boost data.
 *
 *  @author Claude
 */
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

/* customized structure X, Y, Z, Intensity, Velocity, Range, Bearing */
namespace radar_pcl
{
  struct PointXYZIVRB
  {
    PCL_ADD_POINT4D;
    float intensity;
    float velocity;                 // velocity in cartesian
    float range;                    // range in cartesian
    float bearing;                    // bearing of the object in degrees
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
  } EIGEN_ALIGN16;
} // namespace radar_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(radar_pcl::PointXYZIVRB,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity)(float, range, range)(float, bearing, bearing))

#endif // TI_MMWAVE_ROSPKG_SRC_TI_MMWAVE_ROSPKG_INCLUDE_POINT_TYPES_H_
