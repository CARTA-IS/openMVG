#pragma once
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/cameras/Camera_Intrinsics.hpp>
#include <openMVG/geometry/pose3.hpp>
#include <openMVG/types.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace sfm_sigma {

// ---------- Robust 통계 유틸 ----------
inline double Median(std::vector<double>& v) {
  if (v.empty()) return 0.0;
  const size_t n2 = v.size()/2;
  std::nth_element(v.begin(), v.begin()+n2, v.end());
  double med = v[n2];
  if (v.size() % 2 == 0) {
    std::nth_element(v.begin(), v.begin()+n2-1, v.end());
    med = 0.5 * (med + v[n2-1]);
  }
  return med;
}
inline double MAD(std::vector<double> v) {
  if (v.empty()) return 0.0;
  const double med = Median(v);
  for (double& x : v) x = std::abs(x - med);
  return Median(v);
}

// ---------- 결과 패킹 ----------
struct SigmaPack {
  // robust sigma (MAD 기반)
  double sx = 0.0, sy = 0.0; // 축별 (px)
  double s  = 0.0;           // 스칼라 = sqrt(sx*sy)

  // NEW: RMS (평균제곱근) 통계
  double rmsx = 0.0;         // sqrt(mean(rx^2)) (px)
  double rmsy = 0.0;         // sqrt(mean(ry^2)) (px)
  double rms_radial = 0.0;   // sqrt( mean(rx^2 + ry^2) ) (px)

  // counts
  size_t n_obs = 0;          // 유효 관측 수(= rays)
  size_t n_items = 0;        // 사용된 트랙/포인트 수
  size_t n_items_total = 0;  // 전체 트랙/포인트 수

  bool   has_data = false;
};

// ---------- 내부: 재투영 잔차(px) ----------
inline bool ProjectResidualPx(const openMVG::sfm::SfM_Data& sfm,
                              openMVG::IndexT viewId,
                              const openMVG::Vec3& Xw,
                              const openMVG::Vec2& x_obs,
                              bool enforce_cheirality,
                              double& rx, double& ry)
{
  using namespace openMVG; using namespace openMVG::sfm;
  const View* view = sfm.views.at(viewId).get();
  const auto& pose = sfm.poses.at(view->id_pose);
  const auto  cam  = sfm.intrinsics.at(view->id_intrinsic);

  const Vec3 Xc = pose(Xw);
  if (enforce_cheirality && Xc(2) <= 0.0) return false;

  const Vec2 x_pred = cam->project(Xc);
  const Vec2 r = x_obs - x_pred;
  rx = r(0); ry = r(1);
  return true;
}

// ---------- 트랙 σ 추정 ----------
inline SigmaPack ComputeTrackSigmaPx(const openMVG::sfm::SfM_Data& sfm,
                                     bool enforce_cheirality = true)
{
  using namespace openMVG; using namespace openMVG::sfm;
  std::vector<double> rx, ry; rx.reserve(100000); ry.reserve(100000);

  // NEW: RMS 누적
  double sumsq_x = 0.0, sumsq_y = 0.0;

  size_t tracks_used = 0;
  const  size_t tracks_total = sfm.structure.size();

  for (const auto& kv : sfm.structure) {
    const auto& lm = kv.second;
    bool any_valid = false;

    for (const auto& ov : lm.obs) {
      double ex=0, ey=0;
      if (ProjectResidualPx(sfm, ov.first, lm.X, ov.second.x, enforce_cheirality, ex, ey)) {
        rx.push_back(ex); ry.push_back(ey);
        sumsq_x += ex*ex;          // NEW
        sumsq_y += ey*ey;          // NEW
        any_valid = true;
      }
    }
    if (any_valid) ++tracks_used;
  }

  SigmaPack sp;
  sp.n_obs = rx.size();
  sp.n_items = tracks_used;
  sp.n_items_total = tracks_total;
  sp.has_data = (sp.n_obs > 0);
  if (!sp.has_data) return sp;

  // MAD → robust sigma
  const double madx = MAD(rx);
  const double mady = MAD(ry);
  sp.sx = std::max(1e-12, 1.4826 * madx);
  sp.sy = std::max(1e-12, 1.4826 * mady);
  sp.s  = std::sqrt(sp.sx * sp.sy);

  // NEW: RMS
  const double invN = 1.0 / static_cast<double>(sp.n_obs);
  sp.rmsx = std::sqrt(std::max(0.0, sumsq_x * invN));
  sp.rmsy = std::sqrt(std::max(0.0, sumsq_y * invN));
  sp.rms_radial = std::sqrt(std::max(0.0, (sumsq_x + sumsq_y) * invN));

  return sp;
}

// ---------- GCP σ 추정 (2D 재투영 기준) ----------
inline SigmaPack ComputeGCPSigmaPx(const openMVG::sfm::SfM_Data& sfm,
                                   bool enforce_cheirality = true)
{
  using namespace openMVG; using namespace openMVG::sfm;
  std::vector<double> rx, ry;

  double sumsq_x = 0.0, sumsq_y = 0.0;  // NEW

  size_t gcps_used = 0;
  const  size_t gcps_total = sfm.control_points.size();

  for (const auto& kv : sfm.control_points) {
    const auto& gcp = kv.second;
    const auto& X   = gcp.X;

    bool any_valid = false;
    for (const auto& ov : gcp.obs) {
      double ex=0, ey=0;
      if (ProjectResidualPx(sfm, ov.first, X, ov.second.x, enforce_cheirality, ex, ey)) {
        rx.push_back(ex); ry.push_back(ey);
        sumsq_x += ex*ex;          // NEW
        sumsq_y += ey*ey;          // NEW
        any_valid = true;
      }
    }
    if (any_valid) ++gcps_used;
  }

  SigmaPack sp;
  sp.n_obs = rx.size();
  sp.n_items = gcps_used;
  sp.n_items_total = gcps_total;
  sp.has_data = (sp.n_obs > 0);
  if (!sp.has_data) return sp;

  const double madx = MAD(rx);
  const double mady = MAD(ry);
  sp.sx = std::max(1e-12, 1.4826 * madx);
  sp.sy = std::max(1e-12, 1.4826 * mady);
  sp.s  = std::sqrt(sp.sx * sp.sy);

  // NEW: RMS
  const double invN = 1.0 / static_cast<double>(sp.n_obs);
  sp.rmsx = std::sqrt(std::max(0.0, sumsq_x * invN));
  sp.rmsy = std::sqrt(std::max(0.0, sumsq_y * invN));
  sp.rms_radial = std::sqrt(std::max(0.0, (sumsq_x + sumsq_y) * invN));

  return sp;
}
// ---------- 한 번에 둘 다 ----------
inline std::pair<SigmaPack, SigmaPack>
ComputeSigmasPx(const openMVG::sfm::SfM_Data& sfm, bool enforce_cheirality = true)
{
  return { ComputeTrackSigmaPx(sfm, enforce_cheirality),
           ComputeGCPSigmaPx (sfm, enforce_cheirality) };
}

// ---------- 출력 헬퍼 ----------
inline void PrintSigma(const SigmaPack& sp, const std::string& label) {
  std::cout << "[sigma " << label << "] "
            << "MAD: sx=" << sp.sx << " px, sy=" << sp.sy << " px, s=" << sp.s << " px | "
            << "RMS: rmsx=" << sp.rmsx << " px, rmsy=" << sp.rmsy << " px, radial=" << sp.rms_radial << " px | "
            << "obs=" << sp.n_obs << ", items_used=" << sp.n_items << "/" << sp.n_items_total
            << "\n";
}

// ---------- (옵션) BA_Ceres_options에 바로 주입 ----------
#ifdef OPENMVG_SFM_SFM_DATA_BA_CERES_HPP
// 위 매크로는 보통 해당 헤더가 include되면 정의됩니다.
inline void FillBAOptionsWithSigmas(
    openMVG::sfm::Bundle_Adjustment_Ceres::BA_Ceres_options& opt,
    const SigmaPack& track, const SigmaPack& gcp,
    double lambda_gcp = 15.0)
{
  opt.use_whitening_  = true;
  opt.sigma_track_px_ = track.has_data ? track.s : 1.0; // fallback
  // GCP가 비어있으면 트랙보다 약간 크게
  opt.sigma_gcp_px_   = gcp.has_data ? gcp.s : std::max(1.0, track.s*1.5);
  opt.lambda_gcp_     = lambda_gcp;
}
#endif

} // namespace sfm_sigma
