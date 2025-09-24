#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/cameras/cameras.hpp>
#include <openMVG/geometry/pose3.hpp>
#include <openMVG/numeric/numeric.h>

#include <unordered_map>
#include <vector>
#include <cmath>

using namespace openMVG::sfm;
using openMVG::Vec2;
using openMVG::Vec3;

// 사용자 설정값
struct TrackCleanParams {
  int    min_obs_per_track    = 3;      // 최소 관측 수
  double min_triang_deg       = 15;    // 최소 삼각각(도)
  double max_reproj_px        = 16.0;    // 픽셀 재투영 한계 (선택)
  double chi2_df2_threshold   = 9.210;  // df=2, (99% 9.210) (95%는 5.991)
  double sigma_px             = 1.0;    // 관측 표준편차(등방성 가정 시)
  bool   use_chi2_gating      = true;   // 정규화 잔차 게이팅 사용
  bool   enforce_cheirality   = false;   // 체럴리티(Z>0) 검사
};

// 보조: 두 벡터 사이 각도(도)
static inline double AngleDeg(const Vec3& a, const Vec3& b) {
  const double d = std::max(-1.0, std::min(1.0, a.normalized().dot(b.normalized())));
  return std::acos(d) * 180.0 / M_PI;
}

// 트랙의 최대 삼각각(카메라 중심에서 X로 향하는 두 광선 사이의 최대 각)
static double MaxTriangulationAngleDeg(const openMVG::sfm::Landmark& lm,
                                       const openMVG::sfm::SfM_Data& sfm_data)
{
  std::vector<Vec3> rays;
  rays.reserve(lm.obs.size());
  for (const auto& kv : lm.obs) {
    const auto viewId = kv.first;
    const auto* view  = sfm_data.views.at(viewId).get();
    const auto& pose  = sfm_data.poses.at(view->id_pose);
    const Vec3  C     = pose.center();             // 카메라 중심
    const Vec3  ray   = (lm.X - C).normalized();   // 중심->포인트 방향
    rays.push_back(ray);
  }
  double max_deg = 0.0;
  for (size_t i = 0; i + 1 < rays.size(); ++i)
    for (size_t j = i + 1; j < rays.size(); ++j)
      max_deg = std::max(max_deg, AngleDeg(rays[i], rays[j]));
  return max_deg;
}

// 보조: 한 관측의 재투영 잔차(px)와 정규화 잔차 제곱(d2=chi^2)
static bool ResidualsForObservation(const openMVG::sfm::SfM_Data& sfm_data,
                                    const openMVG::sfm::Landmark& lm,
                                    openMVG::IndexT viewId,
                                    double sigma_px,
                                    double* out_px_norm,
                                    double* out_chi2)
{
  const auto* view = sfm_data.views.at(viewId).get();
  const auto& pose = sfm_data.poses.at(view->id_pose);
  const auto  cam  = sfm_data.intrinsics.at(view->id_intrinsic);

  // 관측 픽셀 좌표
  const Vec2 x = lm.obs.at(viewId).x;   // Observation: (x, id_feat) 구조

  // 3D -> 카메라좌표 -> 투영 -> 잔차  (openMVG 권장 형태)
  // 참고: cam.residual(pose(X), x) 또는 proj=cam(pose(X)); r = x - proj
  // pose(X)[2]는 체럴리티 검사에 사용 가능(월드->카메라 변환)  :contentReference[oaicite:1]{index=1}
  const Vec3 Xc = pose(lm.X);
  if (out_px_norm) *out_px_norm = 0.0;
  if (out_chi2)    *out_chi2    = 0.0;

  // 체럴리티: Z>0
  if (Xc(2) <= 0.0) return false;
  const Vec2 proj = cam->project(Xc);        // cam(pose(X)) = reproject  :contentReference[oaicite:2]{index=2}
  const Vec2 r    = x - proj;
  const double px2 = r.squaredNorm();           // 픽셀 잔차 제곱

  if (out_px_norm) *out_px_norm = std::sqrt(px2);
  if (out_chi2)    *out_chi2    = px2 / (sigma_px * sigma_px); // 등방성 σ 가정
  return true;
}

// 핵심: 트랙 정리(관측·트랙 제거). 반환: (지운 관측 수, 지운 트랙 수)
static std::pair<size_t,size_t>
CleanTracksAfterAddingGCP(openMVG::sfm::SfM_Data& sfm_data, const TrackCleanParams& P)
{
  size_t removed_obs = 0, removed_tracks = 0;
  std::vector<openMVG::IndexT> tracks_to_erase;

  // 모든 Landmark(=일반 점 + GCP 포함) 순회  :contentReference[oaicite:3]{index=3}
  for (auto& kv : sfm_data.structure) {
    const openMVG::IndexT lmId = kv.first;
    auto& lm = kv.second;

    // (D) 삼각각 검사: 너무 작으면 트랙 자체 제거 후보  :contentReference[oaicite:4]{index=4}
    const double max_triang = MaxTriangulationAngleDeg(lm, sfm_data);
    if (max_triang < P.min_triang_deg) {
      tracks_to_erase.push_back(lmId);
      continue;
    }

    // 관측별 검사
    std::vector<openMVG::IndexT> obs_to_erase;
    obs_to_erase.reserve(lm.obs.size());

    for (const auto& ov : lm.obs) {
      const openMVG::IndexT viewId = ov.first;

      double px_err = 0.0, chi2 = 0.0;
      bool ok = ResidualsForObservation(sfm_data, lm, viewId, P.sigma_px,
                                        &px_err, &chi2);
      if (!ok && P.enforce_cheirality) {
        obs_to_erase.push_back(viewId); // (C) 체럴리티 위반 -> 제거  :contentReference[oaicite:5]{index=5}
        continue;
      }

      // (B) 픽셀 재투영 한계
      if (P.max_reproj_px > 0.0 && px_err > P.max_reproj_px) {
        obs_to_erase.push_back(viewId);
        continue;
      }
      // (A) 카이제곱 게이팅(df=2) 95%:5.991, 99%:9.210  :contentReference[oaicite:6]{index=6}
      if (P.use_chi2_gating && chi2 > P.chi2_df2_threshold) {
        obs_to_erase.push_back(viewId);
        continue;
      }
    }

    // 관측 제거 실행
    for (auto vid : obs_to_erase) {
      auto it = lm.obs.find(vid);
      if (it != lm.obs.end()) {
        lm.obs.erase(it);
        ++removed_obs;
      }
    }

    // (E) 최소 관측 수 미달이면 트랙 제거
    if (static_cast<int>(lm.obs.size()) < P.min_obs_per_track) {
      tracks_to_erase.push_back(lmId);
    }
  }

  // 트랙 일괄 제거
  for (auto lmId : tracks_to_erase) {
    auto it = sfm_data.structure.find(lmId);
    if (it != sfm_data.structure.end()) {
      removed_obs += it->second.obs.size();
      sfm_data.structure.erase(it);
      ++removed_tracks;
    }
  }

  return {removed_obs, removed_tracks};
}
