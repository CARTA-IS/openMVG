// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_BA_ceres.hpp"

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

#include "ceres/problem.h"
#include "ceres/solver.h"
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/geometry/Similarity3.hpp"
#include "openMVG/geometry/Similarity3_Kernel.hpp"
//- Robust estimation - LMeds (since no threshold can be defined)
#include "openMVG/robust_estimation/robust_estimator_LMeds.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres_camera_functor.hpp"
#include "openMVG/sfm/sfm_data_transform.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/types.hpp"
//3d gcp residual
#include "openMVG/multiview/triangulation_nview.hpp"

#include <ceres/rotation.h>
#include <ceres/types.h>

#include <iostream>
#include <limits>

namespace openMVG
{
  namespace sfm
  {

    using namespace openMVG::cameras;
    using namespace openMVG::geometry;

    // Ceres CostFunctor used for SfM pose center to GPS pose center minimization
    struct PoseCenterConstraintCostFunction
    {
      Vec3 weight_;
      Vec3 pose_center_constraint_;

      PoseCenterConstraintCostFunction(
          const Vec3 &center,
          const Vec3 &weight) : weight_(weight), pose_center_constraint_(center)
      {
      }

      template <typename T>
      bool
      operator()(
          const T *const cam_extrinsics, // R_t
          T *residuals)
          const
      {
        using Vec3T = Eigen::Matrix<T, 3, 1>;
        Eigen::Map<const Vec3T> cam_R(&cam_extrinsics[0]);
        Eigen::Map<const Vec3T> cam_t(&cam_extrinsics[3]);
        const Vec3T cam_R_transpose(-cam_R);

        Vec3T pose_center;
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(cam_R_transpose.data(), cam_t.data(), pose_center.data());
        pose_center = pose_center * T(-1);

        Eigen::Map<Vec3T> residuals_eigen(residuals);
        residuals_eigen = weight_.cast<T>().cwiseProduct(pose_center - pose_center_constraint_.cast<T>());

        return true;
      }
    };

    /// Create the appropriate cost functor according the provided input camera intrinsic model.
    /// The residual can be weighetd if desired (default 0.0 means no weight).
    ceres::CostFunction *IntrinsicsToCostFunction(
        IntrinsicBase *intrinsic,
        const Vec2 &observation,
        const double weight)
    {
      switch (intrinsic->getType())
      {
      case PINHOLE_CAMERA:
        return ResidualErrorFunctor_Pinhole_Intrinsic::Create(observation, weight);
      case PINHOLE_CAMERA_RADIAL1:
        return ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K1::Create(observation, weight);
      case PINHOLE_CAMERA_RADIAL3:
        return ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K3::Create(observation, weight);
      case PINHOLE_CAMERA_BROWN:
        return ResidualErrorFunctor_Pinhole_Intrinsic_Brown_T2::Create(observation, weight);
      case PINHOLE_CAMERA_FISHEYE:
        return ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye::Create(observation, weight);
      case CAMERA_SPHERICAL:
        return ResidualErrorFunctor_Intrinsic_Spherical::Create(intrinsic, observation, weight);
      default:
        return {};
      }
    }

    Bundle_Adjustment_Ceres::BA_Ceres_options::BA_Ceres_options(
        const bool bVerbose,
        bool bmultithreaded)
        : bVerbose_(bVerbose),
          nb_threads_(1),
          parameter_tolerance_(1e-8), //~= numeric_limits<float>::epsilon()
          bUse_loss_function_(true)
    {
#ifdef OPENMVG_USE_OPENMP
      nb_threads_ = omp_get_max_threads();
#endif // OPENMVG_USE_OPENMP
      if (!bmultithreaded)
        nb_threads_ = 1;

      bCeres_summary_ = false;

      // Default configuration use a DENSE representation
      linear_solver_type_ = ceres::DENSE_SCHUR;
      preconditioner_type_ = ceres::JACOBI;
      // If Sparse linear solver are available
      // Descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
      if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
      {
        sparse_linear_algebra_library_type_ = ceres::SUITE_SPARSE;
        linear_solver_type_ = ceres::SPARSE_SCHUR;
      }
      else
      {
        if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
        {
          sparse_linear_algebra_library_type_ = ceres::CX_SPARSE;
          linear_solver_type_ = ceres::SPARSE_SCHUR;
        }
        else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
        {
          sparse_linear_algebra_library_type_ = ceres::EIGEN_SPARSE;
          linear_solver_type_ = ceres::SPARSE_SCHUR;
        }
      }
    }

    Bundle_Adjustment_Ceres::Bundle_Adjustment_Ceres(
        const Bundle_Adjustment_Ceres::BA_Ceres_options &options)
        : ceres_options_(options)
    {
    }

    Bundle_Adjustment_Ceres::BA_Ceres_options &
    Bundle_Adjustment_Ceres::ceres_options()
    {
      return ceres_options_;
    }
    
    // skew‐symmetric matrix for cross(b,·)
    template <typename T>
    Eigen::Matrix<T,3,3> Skew(const Eigen::Matrix<T,3,1>& b) {
      Eigen::Matrix<T,3,3> m;
      m <<    T(0),  -b[2],   b[1],
            b[2],    T(0),  -b[0],
            -b[1],   b[0],    T(0);
      return m;
    }

    // Jet<T> Algebraic N‑View Triangulation
    template <typename T>
    Eigen::Matrix<T,3,1> TriangulateAlgebraicJet(
        const std::vector<Eigen::Matrix<T,3,1>>& bearings,
        const std::vector<Eigen::Matrix<T,3,4>>& poses) {
      const int N = int(bearings.size());
      // A X = -b  (3N×3) · (3×1) = (3N×1)
      Eigen::Matrix<T, Eigen::Dynamic, 3> A(3*N, 3);
      Eigen::Matrix<T, Eigen::Dynamic, 1> b_vec(3*N);

      for (int i = 0; i < N; ++i) {
        // cross(b_i, R_i X + t_i) = 0  ⇒  skew(b_i)*R_i * X = -skew(b_i)*t_i
        auto& bi = bearings[i];
        const auto& P = poses[i];
        auto Ri = P.template block<3,3>(0,0);
        auto ti = P.template block<3,1>(0,3);

        Eigen::Matrix<T,3,3> Bi = Skew(bi);
        // fill A, b_vec
        A.template block<3,3>(3*i,0) = Bi * Ri;
        b_vec.template segment<3>(3*i)  = -Bi * ti;
      }

      // normal equations: (AᵀA) X = Aᵀ b_vec
      Eigen::Matrix<T,3,3> AtA = A.template transpose() * A;
      Eigen::Matrix<T,3,1> Atb = A.template transpose() * b_vec;

      // solve X = (AtA)^{-1} Atb
      // return AtA.ldlt().solve(Atb);
      return AtA.inverse() * Atb;
    }

    struct GCP3DResidual {
      GCP3DResidual(
        const std::vector<IndexT>           &view_ids,
        const std::vector<Vec2>             &pixels,
        const SfM_Data                      &sfm_data,
        const Vec3                          &gt_point,
        const Vec3                          &weight,
        const std::function<const double*(IndexT)> &get_pose_ptr_fn,
        const Vec3                          &centroid,
        double                                inv_avg_dist)
        : view_ids_(view_ids)
        , pixels_(pixels)
        , sfm_data_(sfm_data)
        , gt_point_(gt_point)
        , weight_(weight)
        , get_pose_ptr_fn_(get_pose_ptr_fn)
        , centroid_(centroid)
        , inv_avg_dist_(inv_avg_dist)
      {}

      template <typename T>
      bool operator()(T const* const* /*unused*/, T* residuals) const {
        const int N = int(view_ids_.size());
        if (N < 2) {
          residuals[0] = residuals[1] = residuals[2] = T(0);
          return true;
        }

        // Bearings & normalized poses
        std::vector<Eigen::Matrix<T,3,1>>   bearings_T(N);
        std::vector<Eigen::Matrix<T,3,4>>   poses_T   (N);
        for (int i = 0; i < N; ++i) {
          // undistort → bearing(double) → cast<T>
          auto intr = sfm_data_
            .GetIntrinsics()
            .at(sfm_data_.views.at(view_ids_[i])->id_intrinsic);
          Vec2 ud = intr->get_ud_pixel(pixels_[i]);
          bearings_T[i] = (*intr)(ud).cast<T>();

          // angle‐axis + translation from get_pose_ptr_fn_
          const double* p = get_pose_ptr_fn_(view_ids_[i]);
          Eigen::Matrix<T,3,1> aa; aa << T(p[0]),T(p[1]),T(p[2]);
          Eigen::Matrix<T,3,1> tt; tt << T(p[3]),T(p[4]),T(p[5]);
          T theta = aa.norm();
          Eigen::Matrix<T,3,3> R =
            (theta==T(0))
            ? Eigen::Matrix<T,3,3>::Identity()
            : Eigen::AngleAxis<T>(theta, aa/theta).toRotationMatrix();
          // world → camera: X_cam = R*X + t
          Eigen::Matrix<T,3,4> P;
          P.template block<3,3>(0,0) = R;
          P.template block<3,1>(0,3) =  tt;
          poses_T[i] = P;
        }

        // Algebraic triangulation
        Eigen::Matrix<T,3,1> X_est =
          TriangulateAlgebraicJet<T>(bearings_T, poses_T);

        // residual = weight ∘ (X_est - gt_point)
        Eigen::Matrix<T,3,1> diff = X_est - gt_point_.cast<T>();
        residuals[0] = T(weight_[0]) * diff[0];
        residuals[1] = T(weight_[1]) * diff[1];
        residuals[2] = T(weight_[2]) * diff[2];
        return true;
      }
      const std::vector<IndexT>               view_ids_;
      const std::vector<Vec2>                 pixels_;
      const SfM_Data                         &sfm_data_;
      const Vec3                              gt_point_;
      const Vec3                              weight_;
      const std::function<const double*(IndexT)> get_pose_ptr_fn_;
      const Vec3                              centroid_;
      const double                            inv_avg_dist_;
    };
    // GCP i, j 사이 원래 거리를 보존하는 residual
    struct GCPPairwiseResidual {
      GCPPairwiseResidual(double dij_gt, double weight)
        : dij_gt_(dij_gt), w_(weight) {}

      template <typename T>
      bool operator()(const T* const Xi,  // [X,Y,Z] of GCP i
                      const T* const Xj,  // [X,Y,Z] of GCP j
                      T* residual) const {
        // 차벡터
        T dx = Xi[0] - Xj[0];
        T dy = Xi[1] - Xj[1];
        T dz = Xi[2] - Xj[2];
        // 거리
        T dist = ceres::sqrt(dx*dx + dy*dy + dz*dz);
        // 원래 거리와의 차이에 weight
        residual[0] = T(w_) * (dist - T(dij_gt_));
        return true;
      }

      double dij_gt_;  // ground-truth distance between i and j
      double w_;       // weight for this constraint
    };
    struct TriangulationResidual {
      TriangulationResidual(
          const std::vector<IndexT>& view_ids,
          const std::vector<Vec2>& pixels,
          const SfM_Data& sfm_data,
          const std::function<const double*(IndexT)>& get_pose_ptr,
          const Vec3& centroid,
          double inv_avg_dist)
        : view_ids_(view_ids)
        , pixels_(pixels)
        , sfm_data_(sfm_data)
        , get_pose_ptr_(get_pose_ptr)
        , centroid_(centroid)
        , inv_avg_dist_(inv_avg_dist)
      {}

      template <typename T>
      bool operator()(T const* const* parameters,
                      T* residuals) const {
        const size_t N = view_ids_.size();
        
        // 1) 각 뷰에 대한 bearing 및 투영 행렬 구성
        std::vector<Eigen::Matrix<T,3,1>> bearings; bearings.reserve(N);
        std::vector<Eigen::Matrix<T,3,4>> Pn;       Pn.reserve(N);
        for (size_t i = 0; i < N; ++i) {
          IndexT vid = view_ids_[i];
          
          // Intrinsics -> pixel to bearing
          const auto* cam = dynamic_cast<const cameras::Pinhole_Intrinsic*>(
            sfm_data_.GetIntrinsics().at(
              sfm_data_.views.at(vid)->id_intrinsic).get());
          Vec2 undist = cam->get_ud_pixel(pixels_[i]);
          auto b3 = (*cam)(undist);
          bearings.emplace_back(b3.template cast<T>());

          // Pose parameters (6 DOF)
          const T* p6 = parameters[i];  // 0..5
          Eigen::Matrix<T,3,1> axis(p6[0], p6[1], p6[2]);
          Eigen::Matrix<T,3,1> trans(p6[3], p6[4], p6[5]);
          T theta = axis.norm();
          Eigen::Matrix<T,3,3> R = (theta == T(0))
            ? Eigen::Matrix<T,3,3>::Identity()
            : Eigen::AngleAxis<T>(theta, axis / theta).toRotationMatrix();

          Eigen::Matrix<T,3,4> Pni;
          Pni.template block<3,3>(0,0) = R;
          // translation 조정: centroid & inv_avg_dist
          Pni.template block<3,1>(0,3) = 
            (trans + R * centroid_.template cast<T>()) * T(inv_avg_dist_);
          Pn.emplace_back(Pni);
        }

        // 2) 3D 점 파라미터 추출
        const T* x_ptr = parameters[N];  // parameters[N][0..2]
        Eigen::Matrix<T,3,1> X_param;
        X_param << x_ptr[0], x_ptr[1], x_ptr[2];

        // 3) Algebraic Jet Triangulation
        Eigen::Matrix<T,3,1> X_triang = 
          TriangulateAlgebraicJet<T>(bearings, Pn);

        // 4) Residual 계산
        residuals[0] = X_triang[0] - X_param[0];
        residuals[1] = X_triang[1] - X_param[1];
        residuals[2] = X_triang[2] - X_param[2];
        return true;
      }

      // helper: cost function 생성
      static ceres::CostFunction* Create(
          const std::vector<IndexT>& view_ids,
          const std::vector<Vec2>& pixels,
          const SfM_Data& sfm_data,
          const std::function<const double*(IndexT)>& get_pose_ptr,
          const Vec3& centroid,
          double inv_avg_dist) {
        auto* functor = new TriangulationResidual(
          view_ids, pixels, sfm_data, get_pose_ptr, centroid, inv_avg_dist);
        auto* cost_fn = new ceres::DynamicAutoDiffCostFunction<
          TriangulationResidual, 3>(functor);
        // 뷰마다 6 DOF 파라미터 블록 추가
        for (size_t i = 0; i < view_ids.size(); ++i)
          cost_fn->AddParameterBlock(6);
        // 3D 점 블록 추가
        cost_fn->AddParameterBlock(3);
        cost_fn->SetNumResiduals(3);
        return cost_fn;
      }

    private:
      const std::vector<IndexT> view_ids_;
      const std::vector<Vec2>   pixels_;
      const SfM_Data&           sfm_data_;
      const std::function<const double*(IndexT)> get_pose_ptr_;
      const Vec3                centroid_;
      const double              inv_avg_dist_;
    };
    struct PairwiseDistanceResidual {
      PairwiseDistanceResidual(double d_gt, double w)
        : d_gt_(d_gt), w_(w) {}
      template<typename T>
      bool operator()(const T* const Xi,
                      const T* const Xj,
                      T* residual) const {
        T dx = Xi[0] - Xj[0],
          dy = Xi[1] - Xj[1],
          dz = Xi[2] - Xj[2];
        T d = ceres::sqrt(dx*dx + dy*dy + dz*dz);
        residual[0] = T(w_) * (d - T(d_gt_));
        return true;
      }
      double d_gt_, w_;
    };
    bool Bundle_Adjustment_Ceres::Adjust(
        SfM_Data &sfm_data, // the SfM scene to refine
        const Optimize_Options &options)
    {
      //----------
      // Add camera parameters
      // - intrinsics
      // - poses [R|t]

      // Create residuals for each observation in the bundle adjustment problem. The
      // parameters for cameras and points are added automatically.
      //----------

      double pose_center_robust_fitting_error = 0.0;
      openMVG::geometry::Similarity3 sim_to_center;
      bool b_usable_prior = false;
      if (options.use_motion_priors_opt && sfm_data.GetViews().size() > 3)
      {
        // - Compute a robust X-Y affine transformation & apply it
        // - This early transformation enhance the conditionning (solution closer to the Prior coordinate system)
        {
          // Collect corresponding camera centers
          std::vector<Vec3> X_SfM, X_GPS;
          for (const auto &view_it : sfm_data.GetViews())
          {
            const sfm::ViewPriors *prior = dynamic_cast<sfm::ViewPriors *>(view_it.second.get());
            if (prior != nullptr && prior->b_use_pose_center_ && sfm_data.IsPoseAndIntrinsicDefined(prior))
            {
              X_SfM.push_back(sfm_data.GetPoses().at(prior->id_pose).center());
              X_GPS.push_back(prior->pose_center_);
            }
          }
          openMVG::geometry::Similarity3 sim;

          // Compute the registration:
          if (X_GPS.size() > 3)
          {
            const Mat X_SfM_Mat = Eigen::Map<Mat>(X_SfM[0].data(), 3, X_SfM.size());
            const Mat X_GPS_Mat = Eigen::Map<Mat>(X_GPS[0].data(), 3, X_GPS.size());
            geometry::kernel::Similarity3_Kernel kernel(X_SfM_Mat, X_GPS_Mat);
            const double lmeds_median = openMVG::robust::LeastMedianOfSquares(kernel, &sim);
            if (lmeds_median != std::numeric_limits<double>::max())
            {
              b_usable_prior = true; // PRIOR can be used safely

              // Compute the median residual error once the registration is applied
              for (Vec3 &pos : X_SfM) // Transform SfM poses for residual computation
              {
                pos = sim(pos);
              }
              Vec residual = (Eigen::Map<Mat3X>(X_SfM[0].data(), 3, X_SfM.size()) - Eigen::Map<Mat3X>(X_GPS[0].data(), 3, X_GPS.size())).colwise().norm();
              std::sort(residual.data(), residual.data() + residual.size());
              pose_center_robust_fitting_error = residual(residual.size() / 2);

              // Apply the found transformation to the SfM Data Scene
              openMVG::sfm::ApplySimilarity(sim, sfm_data);

              // Move entire scene to center for better numerical stability
              Vec3 pose_centroid = Vec3::Zero();
              for (const auto &pose_it : sfm_data.poses)
              {
                pose_centroid += (pose_it.second.center() / (double)sfm_data.poses.size());
              }
              sim_to_center = openMVG::geometry::Similarity3(openMVG::sfm::Pose3(Mat3::Identity(), pose_centroid), 1.0);
              openMVG::sfm::ApplySimilarity(sim_to_center, sfm_data, true);
            }
          }
        }
      }

      ceres::Problem problem;

      // Data wrapper for refinement:
      Hash_Map<IndexT, std::vector<double>> map_intrinsics;
      Hash_Map<IndexT, std::vector<double>> map_poses;

      // Setup Poses data & subparametrization
      for (const auto &pose_it : sfm_data.poses)
      {
        const IndexT indexPose = pose_it.first;

        const Pose3 &pose = pose_it.second;
        const Mat3 R = pose.rotation();
        const Vec3 t = pose.translation();

        double angleAxis[3];
        ceres::RotationMatrixToAngleAxis((const double *)R.data(), angleAxis);
        // angleAxis + translation
        map_poses[indexPose] = {angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2)};

        double *parameter_block = &map_poses.at(indexPose)[0];
        problem.AddParameterBlock(parameter_block, 6);
        if (options.extrinsics_opt == Extrinsic_Parameter_Type::NONE)
        {
          // set the whole parameter block as constant for best performance
          problem.SetParameterBlockConstant(parameter_block);
        }
        else // Subset parametrization
        {
          std::vector<int> vec_constant_extrinsic;
          // If we adjust only the translation, we must set ROTATION as constant
          if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_TRANSLATION)
          {
            // Subset rotation parametrization
            vec_constant_extrinsic.insert(vec_constant_extrinsic.end(), {0, 1, 2});
          }
          // If we adjust only the rotation, we must set TRANSLATION as constant
          if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_ROTATION)
          {
            // Subset translation parametrization
            vec_constant_extrinsic.insert(vec_constant_extrinsic.end(), {3, 4, 5});
          }
          if (!vec_constant_extrinsic.empty())
          {
            ceres::SubsetParameterization *subset_parameterization =
                new ceres::SubsetParameterization(6, vec_constant_extrinsic);
            problem.SetParameterization(parameter_block, subset_parameterization);
          }
        }
      }

      // Setup Intrinsics data & subparametrization
      for (const auto &intrinsic_it : sfm_data.intrinsics)
      {
        const IndexT indexCam = intrinsic_it.first;

        if (isValid(intrinsic_it.second->getType()))
        {
          map_intrinsics[indexCam] = intrinsic_it.second->getParams();
          if (!map_intrinsics.at(indexCam).empty())
          {
            double *parameter_block = &map_intrinsics.at(indexCam)[0];
            problem.AddParameterBlock(parameter_block, map_intrinsics.at(indexCam).size());
            if (options.intrinsics_opt == Intrinsic_Parameter_Type::NONE)
            {
              // set the whole parameter block as constant for best performance
              problem.SetParameterBlockConstant(parameter_block);
            }
            else
            {
              const std::vector<int> vec_constant_intrinsic =
                  intrinsic_it.second->subsetParameterization(options.intrinsics_opt);
              if (!vec_constant_intrinsic.empty())
              {
                ceres::SubsetParameterization *subset_parameterization =
                    new ceres::SubsetParameterization(
                        map_intrinsics.at(indexCam).size(), vec_constant_intrinsic);
                problem.SetParameterization(parameter_block, subset_parameterization);
              }
            }
          }
        }
        else
        {
          std::cerr << "Unsupported camera type." << std::endl;
        }
      }

      // Set a LossFunction to be less penalized by false measurements
      //  - set it to nullptr if you don't want use a lossFunction.
      ceres::LossFunction *p_LossFunction =
          ceres_options_.bUse_loss_function_ ? new ceres::HuberLoss(Square(4.0))
                                             : nullptr;

      // For all visibility add reprojections errors:
      for (auto &structure_landmark_it : sfm_data.structure)
      {
        const Observations &obs = structure_landmark_it.second.obs;

        for (const auto &obs_it : obs)
        {
          // Build the residual block corresponding to the track observation:
          const View *view = sfm_data.views.at(obs_it.first).get();

          // Each Residual block takes a point and a camera as input and outputs a 2
          // dimensional residual. Internally, the cost function stores the observed
          // image location and compares the reprojection against the observation.
          ceres::CostFunction *cost_function =
              IntrinsicsToCostFunction(sfm_data.intrinsics.at(view->id_intrinsic).get(),
                                       obs_it.second.x);

          if (cost_function)
          {
            if (!map_intrinsics.at(view->id_intrinsic).empty())
            {
              problem.AddResidualBlock(cost_function,
                                       p_LossFunction,
                                       &map_intrinsics.at(view->id_intrinsic)[0],
                                       &map_poses.at(view->id_pose)[0],
                                       structure_landmark_it.second.X.data());
            }
            else
            {
              problem.AddResidualBlock(cost_function,
                                       p_LossFunction,
                                       &map_poses.at(view->id_pose)[0],
                                       structure_landmark_it.second.X.data());
            }
          }
          else
          {
            std::cerr << "Cannot create a CostFunction for this camera model." << std::endl;
            return false;
          }
        }
        if (options.structure_opt == Structure_Parameter_Type::NONE)
          problem.SetParameterBlockConstant(structure_landmark_it.second.X.data());
      }
      auto get_pose_ptr = [&](IndexT vid) -> const double* {
        return &map_poses.at(vid)[0];
      };
      if (options.control_point_opt.bUse_control_points)
      {
        // Use Ground Control Point:
        // - fixed 3D points with weighted observations
        for (auto &gcp_landmark_it : sfm_data.control_points)
        {
          const Observations &obs = gcp_landmark_it.second.obs;

          for (const auto &obs_it : obs)
          {
            // Build the residual block corresponding to the track observation:
            const View *view = sfm_data.views.at(obs_it.first).get();

            // Each Residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.
            ceres::CostFunction *cost_function =
                IntrinsicsToCostFunction(
                    sfm_data.intrinsics.at(view->id_intrinsic).get(),
                    obs_it.second.x,
                    options.control_point_opt.weight);

            if (cost_function)
            {
              if (!map_intrinsics.at(view->id_intrinsic).empty())
              {
                problem.AddResidualBlock(cost_function,
                                         nullptr,
                                         &map_intrinsics.at(view->id_intrinsic)[0],
                                         &map_poses.at(view->id_pose)[0],
                                         gcp_landmark_it.second.X.data());
              }
              else
              {
                problem.AddResidualBlock(cost_function,
                                         nullptr,
                                         &map_poses.at(view->id_pose)[0],
                                         gcp_landmark_it.second.X.data());
              }
            }
          }
          if (obs.empty())
          {
            std::cerr
                << "Cannot use this GCP id: " << gcp_landmark_it.first
                << ". There is not linked image observation." << std::endl;
          }
          else
          {
            // Set the 3D point as FIXED (it's a valid GCP)
            problem.SetParameterBlockConstant(gcp_landmark_it.second.X.data());
          }
        }
        // Setup triangulated GCPs as parameters and add residual blocks
        // Compute the centroid of all 3D control points
        Vec3 centroid = Vec3::Zero();
        const auto & cps = sfm_data.control_points;
        const int num_cps = int(cps.size());
        for (auto const & cp_pair : cps) {
          // Sum up the position vectors of each control point
          centroid += cp_pair.second.X;
        }
        // Divide by the number of control points to get the mean position
        centroid /= static_cast<double>(num_cps);
        
        // Compute the average distance from each control point to the centroid
        double avg_dist = 0.0;
        for (auto const & cp_pair : cps) {
          // Accumulate the Euclidean distance
          avg_dist += (cp_pair.second.X - centroid).norm();
        }
        // Finalize average by dividing by the number of control points
        avg_dist /= double(num_cps);
        // Compute the inverse of the average distance for normalizing residuals
        double inv_avg_dist = 1.0 / avg_dist;

        // 1) 카메라 focal 평균 및 baseline 평균 계산
        double sum_f = 0.0;
        std::vector<Vec3> centers;
        for (auto const& view_it : sfm_data.views) {
          const View* v = view_it.second.get();
          if (!sfm_data.IsPoseAndIntrinsicDefined(v)) continue;
          // focal
          auto* cam = sfm_data.GetIntrinsics().at(v->id_intrinsic).get();
          // Pinhole_Intrinsic* 로 다운캐스트
          const openMVG::cameras::Pinhole_Intrinsic* pin =
              dynamic_cast<const openMVG::cameras::Pinhole_Intrinsic*>(cam);

          double f = 0.0;
          if (pin) {
            f = pin->focal();  // 픽셀 단위 초점거리
            sum_f += f;
          }
          // camera center
          centers.push_back(sfm_data.GetPoseOrDie(v).center());
        }
        int C = (int)centers.size();
        double f_avg = (C>0? sum_f/C : 1.0);
        // baseline: average pairwise distance
        double sum_base = 0;
        int cnt = 0;
        for (int i = 0; i < C; ++i) {
          for (int j = i+1; j < C; ++j) {
            sum_base += (centers[i] - centers[j]).norm();
            ++cnt;
          }
        }
        double baseline_avg = (cnt>0 ? sum_base / cnt : 1.0);

        for (auto const & cp_pair : cps) {
          IndexT cp_id      = cp_pair.first;
          const auto & landmark   = cp_pair.second;

          // Collect all observation view IDs and corresponding image pixels
          std::vector<IndexT> view_ids;
          std::vector<Vec2>   pixels;
          view_ids.reserve(landmark.obs.size());
          pixels  .reserve(landmark.obs.size());
          for (auto const & obs : landmark.obs) {
            view_ids.push_back(obs.first);
            pixels  .push_back(obs.second.x);
          }

          // Need at least two observations to triangulate
          if (view_ids.size() < 2) 
            continue;
          
          // 3) 픽셀 단위 weight 계산 (평균 Z = centroid.z)
          double Zc = centroid[2];  // 또는 GCP 깊이 평균을 쓸 수도 있습니다
          Vec3 weight_pixel;
          weight_pixel[0] = f_avg  / Zc;                     // ΔX(m) → Δu(px)
          weight_pixel[1] = f_avg  / Zc;                     // ΔY(m) → Δv(px)
          weight_pixel[2] = f_avg  * baseline_avg / (Zc*Zc); // ΔZ(m) → Δ시차(px)

          // Create the residual functor for 3D GCP reprojection error
          auto* functor = new GCP3DResidual(
              view_ids,                 // IDs of views observing this GCP
              pixels,                   // 2D image measurements
              sfm_data,                 // full SfM dataset
              landmark.X,               // ground-truth 3D point
              Vec3(0, 0, 1e8),          // weighting vector (x, y, z)
              get_pose_ptr,             // pose lookup function
              centroid,                 // centroid for normalization
              inv_avg_dist              // inverse average distance for scaling
            );

          // Wrap the functor in an auto-differentiable cost function
          auto* cost_fn = new ceres::DynamicAutoDiffCostFunction<GCP3DResidual, 3>(functor);
          // Add one 6-DOF pose parameter block per observing view
          for (size_t i = 0; i < view_ids.size(); ++i)
            cost_fn->AddParameterBlock(6);
          // There are 3 residuals (x, y, z)
          cost_fn->SetNumResiduals(3);

          // Gather pointers to all pose parameter blocks
          std::vector<double*> param_blocks;
          param_blocks.reserve(view_ids.size());
          for (auto vid : view_ids)
            param_blocks.push_back(&map_poses.at(vid)[0]);

          // Add the residual block to the optimization problem
          problem.AddResidualBlock(
            cost_fn,
            new ceres::HuberLoss(1.0),  // robust loss to lessen effect of outliers
            param_blocks                // list of pointers to parameter blocks
          );
        }
        //pairwise distance constraint
        std::vector<std::array<double,3>> map_Xparam;
        map_Xparam.reserve(sfm_data.control_points.size());

        for (size_t idx = 0; idx < sfm_data.control_points.size(); ++idx)
        {
          const auto &cp_pair  = *std::next(sfm_data.control_points.begin(), idx);
          const auto &landmark = cp_pair.second;

          // view_ids, pixels 재수집
          std::vector<IndexT> view_ids;
          std::vector<Vec2>   pixels;
          view_ids.reserve(landmark.obs.size());
          pixels  .reserve(landmark.obs.size());
          for (auto const& obs : landmark.obs) {
            view_ids.push_back(obs.first);
            pixels  .push_back(obs.second.x);
          }
          if (view_ids.size() < 2) 
            continue;

          // X 파라미터 블록 초기값 추가
          map_Xparam.push_back({landmark.X[0], landmark.X[1], landmark.X[2]});
          // TriangulationResidual 비용 함수 생성
          ceres::CostFunction* cf_tri = TriangulationResidual::Create(
              view_ids, pixels,
              sfm_data, get_pose_ptr,
              centroid, inv_avg_dist);

          // 파라미터 블록 포인터 모으기
          std::vector<double*> blocks;
          blocks.reserve(view_ids.size() + 1);

          // 1) 각 뷰의 pose 블록 포인터
          for (auto vid : view_ids) {
            // get_pose_ptr(vid) → map_poses[vid].data()
            blocks.push_back(const_cast<double*>( get_pose_ptr(vid) ));
          }

          // 2) 최적화할 GCP 3D 위치 블록
          //    map_Xparam[idx] 은 std::array<double,3>
          blocks.push_back( map_Xparam[idx].data() );

          // Residual block 추가
          problem.AddResidualBlock(cf_tri,
                                  /*loss=*/ nullptr,
                                  /*parameter_blocks=*/ blocks);
        }
      }

      // Add Pose prior constraints if any
      if (b_usable_prior)
      {
        for (const auto &view_it : sfm_data.GetViews())
        {
          const sfm::ViewPriors *prior = dynamic_cast<sfm::ViewPriors *>(view_it.second.get());
          if (prior != nullptr && prior->b_use_pose_center_ && sfm_data.IsPoseAndIntrinsicDefined(prior))
          {
            // Add the cost functor (distance from Pose prior to the SfM_Data Pose center)
            ceres::CostFunction *cost_function =
                new ceres::AutoDiffCostFunction<PoseCenterConstraintCostFunction, 3, 6>(
                    new PoseCenterConstraintCostFunction(prior->pose_center_, prior->center_weight_));

            problem.AddResidualBlock(
                cost_function,
                new ceres::HuberLoss(
                    Square(pose_center_robust_fitting_error)),
                &map_poses.at(prior->id_view)[0]);
          }
        }
      }
      // Configure a BA engine and run it
      //  Make Ceres automatically detect the bundle structure.
      ceres::Solver::Options ceres_config_options;
      ceres_config_options.max_num_iterations = 500;
      ceres_config_options.preconditioner_type =
          static_cast<ceres::PreconditionerType>(ceres_options_.preconditioner_type_);
      ceres_config_options.linear_solver_type =
          static_cast<ceres::LinearSolverType>(ceres_options_.linear_solver_type_);
      ceres_config_options.sparse_linear_algebra_library_type =
          static_cast<ceres::SparseLinearAlgebraLibraryType>(ceres_options_.sparse_linear_algebra_library_type_);
      ceres_config_options.minimizer_progress_to_stdout = ceres_options_.bVerbose_;
      // ceres_config_options.logging_type = ceres::PER_MINIMIZER_ITERATION; //SILENT
      ceres_config_options.num_threads = ceres_options_.nb_threads_;
#if CERES_VERSION_MAJOR < 2
      ceres_config_options.num_linear_solver_threads = ceres_options_.nb_threads_;
#endif
      // ceres_config_options.parameter_tolerance = ceres_options_.parameter_tolerance_;
      ceres_config_options.parameter_tolerance = 1e-10;
      ceres_config_options.function_tolerance = 1e-6;
      // ceres_config_options.gradient_tolerance = ??;
      // Solve BA
      ceres::Solver::Summary summary;
      ceres::Solve(ceres_config_options, &problem, &summary);
      if (ceres_options_.bCeres_summary_)
        std::cout << summary.FullReport() << std::endl;
      // If no error, get back refined parameters
      if (!summary.IsSolutionUsable())
      {
        if (ceres_options_.bVerbose_)
          std::cout << "Bundle Adjustment failed." << std::endl;
        return false;
      }
      
      else // Solution is usable
      {
        if (ceres_options_.bVerbose_)
        {
          // Display statistics about the minimization
          std::cout << std::endl
                    << "Bundle Adjustment statistics (approximated RMSE):\n"
                    << " #views: " << sfm_data.views.size() << "\n"
                    << " #poses: " << sfm_data.poses.size() << "\n"
                    << " #intrinsics: " << sfm_data.intrinsics.size() << "\n"
                    << " #tracks: " << sfm_data.structure.size() << "\n"
                    << " #residuals: " << summary.num_residuals << "\n"
                    << " Initial RMSE: " << std::sqrt(2 * summary.initial_cost / summary.num_residuals) << "\n"
                    << " Final RMSE: " << std::sqrt(2 * summary.final_cost / summary.num_residuals) << "\n"
                    << " Termination type: " << summary.message << "\n"
                    << " Time (s): " << summary.total_time_in_seconds << "\n"
                    << std::endl;
          if (options.use_motion_priors_opt)
            std::cout << "Usable motion priors: " << (int)b_usable_prior << std::endl;
        }

        // Update camera poses with refined data
        if (options.extrinsics_opt != Extrinsic_Parameter_Type::NONE)
        {
          for (auto &pose_it : sfm_data.poses)
          {
            const IndexT indexPose = pose_it.first;

            Mat3 R_refined;
            ceres::AngleAxisToRotationMatrix(&map_poses.at(indexPose)[0], R_refined.data());
            Vec3 t_refined(map_poses.at(indexPose)[3], map_poses.at(indexPose)[4], map_poses.at(indexPose)[5]);
            // Update the pose
            Pose3 &pose = pose_it.second;
            pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
          }
        }

        // Update camera intrinsics with refined data
        if (options.intrinsics_opt != Intrinsic_Parameter_Type::NONE)
        {
          for (auto &intrinsic_it : sfm_data.intrinsics)
          {
            const IndexT indexCam = intrinsic_it.first;

            const std::vector<double> &vec_params = map_intrinsics.at(indexCam);
            intrinsic_it.second->updateFromParams(vec_params);
          }
        }

        // Structure is already updated directly if needed (no data wrapping)

        if (b_usable_prior)
        {
          // set back to the original scene centroid
          openMVG::sfm::ApplySimilarity(sim_to_center.inverse(), sfm_data, true);

          //--
          // - Compute some fitting statistics
          //--

          // Collect corresponding camera centers
          std::vector<Vec3> X_SfM, X_GPS;
          for (const auto &view_it : sfm_data.GetViews())
          {
            const sfm::ViewPriors *prior = dynamic_cast<sfm::ViewPriors *>(view_it.second.get());
            if (prior != nullptr && prior->b_use_pose_center_ && sfm_data.IsPoseAndIntrinsicDefined(prior))
            {
              X_SfM.push_back(sfm_data.GetPoses().at(prior->id_pose).center());
              X_GPS.push_back(prior->pose_center_);
            }
          }
          // Compute the registration fitting error (once BA with Prior have been used):
          if (X_GPS.size() > 3)
          {
            // Compute the median residual error
            Vec residual = (Eigen::Map<Mat3X>(X_SfM[0].data(), 3, X_SfM.size()) - Eigen::Map<Mat3X>(X_GPS[0].data(), 3, X_GPS.size())).colwise().norm();
            std::cout
                << "Pose prior statistics (user units):\n"
                << " - Starting median fitting error: " << pose_center_robust_fitting_error << "\n"
                << " - Final fitting error:";
            minMaxMeanMedian<Vec::Scalar>(residual.data(), residual.data() + residual.size());
          }
        }
        return true;
      }
    }

  } // namespace sfm
} // namespace openMVG
