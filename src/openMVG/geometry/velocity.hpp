// This file is customized for being part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2022 Austin LEE.

// #ifndef OPENMVG_GEOMETRY_POSE3_HPP
// #define OPENMVG_GEOMETRY_POSE3_HPP

#include "openMVG/multiview/projection.hpp"

namespace openMVG
{
namespace geometry
{

/**
* @brief Defines a velocity
*/
class TranslationVelocity
{
  protected:
    /// Center of rotation
    Vec3 velocity_;

  public:
    /**
    * @brief Constructor
    * @param v Velocity
    * @note Default (without args) defines an Identity pose.
    */
    TranslationVelocity
    (
      const Vec3& v = std::move(Vec3::Zero())
    )
    : velocity_( v ) {}  

    /**
    * @brief Get center of rotation
    * @return center of rotation
    */
    const Vec3& velocity() const
    {
      return velocity_;
    }

    /**
    * @brief Get center of rotation
    * @return Center of rotation
    */
    Vec3& velocity()
    {
      return velocity_;
    }


};
} // namespace geometry
} // namespace openMVG

// #endif  // OPENMVG_GEOMETRY_POSE3_HPP
