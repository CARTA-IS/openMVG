#pragma once
// Brown-T2 intrinsic correlation → adaptive plan (lock/weights/schedule).
// - Input  : ceres::Problem (after a BA solve), the intrinsic parameter vector
//            of a Brown-T2 camera: [f, cx, cy, k1, k2, k3, p1, p2] (size=8).
// - Output : correlation matrix & an adaptive plan telling what to lock next,
//            how to tweak Huber/Lambda and radial weighting, etc.
//
// Dependencies: Ceres, Eigen

#include <ceres/problem.h>
#include <ceres/covariance.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace brown_corr_adapt {

constexpr int BROWN_SIZE = 8; // [f,cx,cy,k1,k2,k3,p1,p2]

// ---------- Correlation computation (intrinsic-only) -----------------------
struct BrownCorr {
  Eigen::Matrix<double, BROWN_SIZE, BROWN_SIZE> corr = Eigen::Matrix<double,8,8>::Zero();
  Eigen::Matrix<double, BROWN_SIZE, BROWN_SIZE> cov  = Eigen::Matrix<double,8,8>::Zero();
  double sigma[BROWN_SIZE] = {0,0,0,0,0,0,0,0}; // std of each param
  bool ok = false;
};

// 안전 버전: 로컬 차원/상수/포인터 검증 + 자세한 이유 반환
struct CorrStatus {
  bool ok = false;
  std::string reason;   // 실패 원인 메시지
  int local_size = 0;   // 로컬 차원(Subset/상수 반영)
  bool is_constant = false;
};

inline std::pair<BrownCorr, CorrStatus>
ComputeCorr_BrownT2(ceres::Problem& problem,
                    std::vector<double>& intrinsics)
{
  BrownCorr out; CorrStatus st;

  // 0) 사이즈 체크 (Brown-8)
  if (intrinsics.size() != BROWN_SIZE) {
    st.reason = "intrinsic size != 8 (Brown-T2 expected)";
    return {out, st};
  }

  // 1) 파라미터 블록 포인터
  double* A = intrinsics.data();
  if (!problem.HasParameterBlock(A)) {
    st.reason = "parameter block not found in ceres::Problem";
    return {out, st};
  }

  // 2) 현재 상태 점검 (local_size==0 이면 공분산 불가)
  st.is_constant = problem.IsParameterBlockConstant(A);
  st.local_size  = problem.ParameterBlockLocalSize(A);

  // intrinsic 블록이 상수였다면 잠깐 Variable로 전환(끝에 복원)
  const bool flip_to_variable = st.is_constant;
  if (flip_to_variable) {
    problem.SetParameterBlockVariable(A);
    st.is_constant = false;
    st.local_size  = problem.ParameterBlockLocalSize(A);
  }

  // 주의: 이 함수는 **문제 내 다른 블록은 변경하지 않습니다.**
  // → extrinsic/structure를 고정(Constant)해 "intrinsic만의 공분산"을 원하면
  //    **호출부에서** 미리 Constant로 설정해 주세요.

  if (st.local_size <= 0) {
    st.reason = "local_size==0 (all DOFs locked) → covariance undefined";
    if (flip_to_variable) problem.SetParameterBlockConstant(A); // 복원
    return {out, st};
  }

  // 3) SPARSE 공분산 (intrinsic 자기 블록 A-A만 요청)
  ceres::Covariance::Options copts;
  copts.algorithm_type = ceres::SPARSE_QR;
#if defined(CERES_HAVE_SUITE_SPARSE)
  copts.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
#elif defined(CERES_HAVE_EIGEN_SPARSE)
  copts.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
#endif
  ceres::Covariance cov(copts);

  std::vector<std::pair<const double*, const double*>> req = {{A, A}};
  if (!cov.Compute(req, &problem)) {
    st.reason = "ceres::Covariance::Compute failed";
    if (flip_to_variable) problem.SetParameterBlockConstant(A); // 복원
    return {out, st};
  }

  // 4) 로컬 공분산을 받아 8×8로 구성 (로컬=8일 때만 full)
  const int d = st.local_size;
  Eigen::MatrixXd C_local(d, d);
  cov.GetCovarianceBlock(A, A, C_local.data());

  out.cov.setZero();
  out.corr.setZero();

  if (d == BROWN_SIZE) {
    // full 8×8 (Subset 잠금 없음)
    out.cov = C_local;
    for (int i=0;i<BROWN_SIZE;++i)
      out.sigma[i] = std::sqrt(std::max(0.0, out.cov(i,i)));
    for (int r=0;r<BROWN_SIZE;++r)
      for (int c=0;c<BROWN_SIZE;++c) {
        double denom = std::max(1e-15, out.sigma[r]*out.sigma[c]);
        out.corr(r,c) = out.cov(r,c) / denom;
        out.corr(r,c) = std::max(-1.0, std::min(1.0, out.corr(r,c)));
      }
    out.ok = true; st.ok = true; st.reason.clear();
  } else {
    // 일부 잠금(Subset)으로 로컬 차원이 8이 아님.
    // 글로벌 8×8 임베딩에는 '고정 인덱스 목록'이 필요하므로 여기서는
    // 로컬만 계산했다는 사실을 사유에 남긴다.
    st.reason = "local_size != 8 (some intrinsics locked). "
                "To embed local→global 8×8, pass your const-index mapping at call site.";
    // out.ok=false 유지(호출부에서 처리)
  }

  // 5) 상태 복원
  if (flip_to_variable)
    problem.SetParameterBlockConstant(A);

  return {out, st};
}

// Pretty print small matrix (optional)
inline void PrintCorr(const BrownCorr& C, const char* title="Brown-T2 intrinsic corr")
{
  if (!C.ok) { std::cout << "[corr] not available\n"; return; }
  static const char* names[BROWN_SIZE] = {"f","cx","cy","k1","k2","k3", "p1","p2"};
  std::cout << "== " << title << " ==\n";
  for (int r=0;r<BROWN_SIZE;++r) {
    for (int c=0;c<BROWN_SIZE;++c) {
      std::cout << (c? "  ":"") << std::fixed << std::setprecision(3) << C.corr(r,c);
    }
    std::cout << "   // " << names[r] << "  sigma=" << C.sigma[r] << "\n";
  }
}

// ---------- Adaptive policy (rules → plan) ---------------------------------
struct Plan {
  // what to lock next BA (Brown indices to keep constant)
  std::vector<int> lock_indices; // e.g. {4,5,6} → lock k2,p1,p2

  // loss/weights knobs for next BA
  double huber_delta_track_scale = 1.0; // <1 tighter
  double lambda_gcp_scale        = 1.0; // <1 relax binding
  double radial_gamma            = 1.4; // 1.2~2.0 (outer weighting)
  double radial_alpha            = 0.3; // floor weight for center

  // schedule hints
  bool short_open_k1 = false;  // open only k1 for 5-10 iters
  bool lock_cxcy     = false;  // lock principal point
  std::string note;            // human-readable rationale
};

// Helper: push unique index
inline void PushUniq(std::vector<int>& v, int idx){
  if (std::find(v.begin(), v.end(), idx) == v.end()) v.push_back(idx);
}

// Decide plan from correlation (absolute thresholds tuned conservatively)
inline Plan DecidePlan(const BrownCorr& C)
{
  Plan P;
  if (!C.ok) { P.note = "corr not ok; keep defaults"; return P; }

auto A = [&](int i,int j){ return std::abs(C.corr(i,j)); };

double fk1 = A(0,3), fk2 = A(0,4), fk3 = A(0,5);
double r21 = A(4,3), r32 = A(5,4), r31 = A(5,3);
double fpp = std::max(A(0,1), A(0,2));
double k1pp= std::max(A(3,1), A(3,2));
double p12 = A(6,7);
double cxcy= A(1,2);
//f , cx, cy, k1,k2,k3,p1,p2,
// 1) f–radial 최대치로 심각도 판단
double fk_max = std::max({fk1,fk2,fk3});
if (fk_max >= 0.90) {
  // 강결합 플랜
  P.lock_indices = {4,5,6,7};              // k2,k3,p1,p2
  if (fpp >= 0.70 || k1pp >= 0.70) {       // pp도 끼면 함께 잠금
    P.lock_cxcy = true; PushUniq(P.lock_indices,1); PushUniq(P.lock_indices,2);
  }
  P.short_open_k1 = true;                  // k1은 짧게만 열어 정렬
  P.huber_delta_track_scale = 0.85;
  P.radial_gamma = std::max(P.radial_gamma, 1.8);
  P.lambda_gcp_scale = std::min(P.lambda_gcp_scale, 0.85);
}
else if (fk_max >= 0.80) {
  // 중강 플랜
  P.lock_indices = {4,5};                  // k2,k3
  if (fpp >= 0.70 || k1pp >= 0.70) {
    P.lock_cxcy = true; PushUniq(P.lock_indices,1); PushUniq(P.lock_indices,2);
  }
  P.short_open_k1 = true;
  P.huber_delta_track_scale = std::min(P.huber_delta_track_scale, 0.90);
  P.radial_gamma = std::max(P.radial_gamma, 1.6);
}
else if (fk_max >= 0.70) {
  // 경고 플랜
  PushUniq(P.lock_indices,5);              // k3
  if (r21 >= 0.85) PushUniq(P.lock_indices,4); // k2도 잠금 검토
  if (fpp >= 0.70 || k1pp >= 0.70) { P.lock_cxcy = true; PushUniq(P.lock_indices,1); PushUniq(P.lock_indices,2); }
  P.radial_gamma = std::max(P.radial_gamma, 1.5);
}

// 2) 라디얼 내부 결합 보정
if (r32 >= 0.90 || r31 >= 0.90) PushUniq(P.lock_indices,5); // k3 잠금 강제
if (r21 >= 0.90) PushUniq(P.lock_indices,4);                // k2 잠금 강제

// 3) p1–p2
if (p12 >= 0.80) { PushUniq(P.lock_indices,6); PushUniq(P.lock_indices,7); }

// 4) cx–cy 자체 결합이 높으면 둘을 함께 다룸
if (cxcy >= 0.70) { P.lock_cxcy = true; PushUniq(P.lock_indices,1); PushUniq(P.lock_indices,2); }

// 최종: lock_indices 정렬·중복 제거는 BuildConstIndexList(P)에서 처리

  // 기본 가이드(너무 세게 가지 않도록)
  P.lambda_gcp_scale = std::min(P.lambda_gcp_scale, 0.85);
  return P;
}

// Build SubsetParameterization "constant indices" for Brown vector
// from a Plan; returns sorted unique indices.
inline std::vector<int> BuildConstIndexList(const Plan& P) {
  std::vector<int> v = P.lock_indices;
  if (P.lock_cxcy) { PushUniq(v,1); PushUniq(v,2); }
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
  return v;
}

} // namespace brown_corr_adapt
