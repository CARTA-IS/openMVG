#pragma once
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/cameras/Camera_Intrinsics.hpp>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace sfm_radw {

struct Params {
  double gamma   = 1.5;   // 곡률 (1~2 권장)
  double alpha   = 0.3;   // 바닥 가중치(중앙이 0이 되지 않게)
  bool   normalize_by_view = true; // 뷰별 평균 1 정규화
};

struct RadialWeighter {
  Params P;
  // 뷰ID -> (principal point, rmax, mean_w)
  std::unordered_map<openMVG::IndexT, openMVG::Vec2> pp_;
  std::unordered_map<openMVG::IndexT, double>        rmax_;
  std::unordered_map<openMVG::IndexT, double>        mean_w_; // 정규화용

  static openMVG::Vec2 GetPrincipalPoint(const openMVG::cameras::IntrinsicBase* cam,
                                         const openMVG::sfm::View* view)
  {
    using namespace openMVG;
    // 핀홀 계열이면 principal_point()를 쓰고, 아니면 이미지 중심으로 폴백
    if (const auto* pin = dynamic_cast<const cameras::Pinhole_Intrinsic*>(cam)) {
      return pin->principal_point();
    }
    const double cx = (view && view->ui_width ) ? view->ui_width  * 0.5 : 0.0;
    const double cy = (view && view->ui_height) ? view->ui_height * 0.5 : 0.0;
    return Vec2(cx, cy);
  }

  static double GetRmax(const openMVG::Vec2& pp, const openMVG::sfm::View* view)
  {
    if (!view || !view->ui_width || !view->ui_height) return 1.0;
    const double w = view->ui_width, h = view->ui_height;
    const double dx = std::max(pp(0), w - pp(0));
    const double dy = std::max(pp(1), h - pp(1));
    return std::max(1e-6, std::sqrt(dx*dx + dy*dy));
  }

  // 1-pass 준비(뷰별 pp/rmax), 2-pass로 mean_w 계산
  void Build(const openMVG::sfm::SfM_Data& sfm)
  {
    using namespace openMVG; using namespace openMVG::sfm;
    pp_.clear(); rmax_.clear(); mean_w_.clear();

    // pass 0: 뷰별 pp/rmax
    for (const auto& kv : sfm.views) {
      const auto* v = kv.second.get();
      const auto  cam = sfm.intrinsics.at(v->id_intrinsic);
      const Vec2  pp  = GetPrincipalPoint(cam.get(), v);
      const double rM = GetRmax(pp, v);
      pp_[v->id_view]   = pp;
      rmax_[v->id_view] = rM;
      mean_w_[v->id_view] = 0.0; // 초기화
    }

    if (!P.normalize_by_view) return;

    // pass 1: raw weight 합/개수
    std::unordered_map<IndexT, double> sum_w;
    std::unordered_map<IndexT, size_t> cnt;
    for (const auto& lm_kv : sfm.structure) {
      const auto& lm = lm_kv.second;
      for (const auto& ov : lm.obs) {
        const IndexT vid = ov.first;
        const auto*  v   = sfm.views.at(vid).get();
        const auto   itR = rmax_.find(vid);
        if (itR == rmax_.end()) continue;
        const double rmax = itR->second;
        const openMVG::Vec2& pp = pp_.at(vid);
        const openMVG::Vec2  x  = ov.second.x;
        const double rn = std::min(1.0, (x - pp).norm() / rmax);
        const double w_raw = P.alpha + (1.0 - P.alpha) * std::pow(rn, P.gamma);
        sum_w[vid] += w_raw;
        cnt[vid]   += 1;
      }
    }
    for (auto& kv : mean_w_) {
      const IndexT vid = kv.first;
      const double s = sum_w[vid];
      const size_t n = std::max<size_t>(1, cnt[vid]);
      kv.second = s / n; // mean_w
      if (!(kv.second > 0.0)) kv.second = 1.0;
    }
  }

  // 개별 관측 가중치 반환 (정규화 포함)
  double Weight(openMVG::IndexT viewId, const openMVG::Vec2& x_px) const
  {
    auto itR = rmax_.find(viewId);
    if (itR == rmax_.end()) return 1.0;
    const double rmax = itR->second;
    const openMVG::Vec2& pp = pp_.at(viewId);
    const double rn = std::min(1.0, (x_px - pp).norm() / rmax);
    double w = P.alpha + (1.0 - P.alpha) * std::pow(rn, P.gamma);
    if (P.normalize_by_view) {
      const double m = mean_w_.at(viewId);
      w /= m;
    }
    return w;
  }
};

} // namespace sfm_radw