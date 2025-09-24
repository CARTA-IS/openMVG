#pragma once
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/cameras/Camera_Intrinsics.hpp>
#include <openMVG/geometry/pose3.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace gcp_quality {

// GCP 품질 평가 결과
struct GCPQualityMetrics {
    double triangulation_error = 0.0;    // 삼각측량 오차 (픽셀)
    double reprojection_rms = 0.0;       // 재투영 RMS (픽셀)
    double geometric_consistency = 0.0;  // 기하학적 일관성 점수 (0-1)
    double observation_count = 0;        // 관측 수
    double triangulation_angle = 0.0;    // 최대 삼각각 (도)
    bool is_valid = false;               // 전체적으로 유효한지 여부
};

// GCP 품질 평가 함수
inline GCPQualityMetrics EvaluateGCPQuality(
    const openMVG::sfm::Landmark& gcp,
    const openMVG::sfm::SfM_Data& sfm_data,
    double min_triang_angle = 15.0,      // 최소 삼각각 (도)
    double max_reproj_error = 5.0,       // 최대 재투영 오차 (픽셀)
    int min_observations = 3)            // 최소 관측 수
{
    GCPQualityMetrics metrics;
    
    if (gcp.obs.size() < min_observations) {
        return metrics; // 관측 수 부족
    }
    
    // 삼각각 계산
    std::vector<openMVG::Vec3> rays;
    for (const auto& obs : gcp.obs) {
        const auto* view = sfm_data.views.at(obs.first).get();
        const auto& pose = sfm_data.poses.at(view->id_pose);
        const openMVG::Vec3 ray = (gcp.X - pose.center()).normalized();
        rays.push_back(ray);
    }
    
    double max_angle = 0.0;
    for (size_t i = 0; i < rays.size(); ++i) {
        for (size_t j = i + 1; j < rays.size(); ++j) {
            const double cos_angle = std::max(-1.0, std::min(1.0, rays[i].dot(rays[j])));
            const double angle = std::acos(cos_angle) * 180.0 / M_PI;
            max_angle = std::max(max_angle, angle);
        }
    }
    
    metrics.triangulation_angle = max_angle;
    metrics.observation_count = gcp.obs.size();
    
    if (max_angle < min_triang_angle) {
        return metrics; // 삼각각 부족
    }
    
    // 재투영 오차 계산
    double sum_squared_error = 0.0;
    int valid_observations = 0;
    
    for (const auto& obs : gcp.obs) {
        const auto* view = sfm_data.views.at(obs.first).get();
        const auto& pose = sfm_data.poses.at(view->id_pose);
        const auto* cam = sfm_data.intrinsics.at(view->id_intrinsic).get();
        
        const openMVG::Vec3 Xc = pose(gcp.X);
        if (Xc(2) <= 0.0) continue; // 체럴리티 위반
        
        const openMVG::Vec2 projected = cam->project(Xc);
        const openMVG::Vec2 residual = obs.second.x - projected;
        const double error = residual.norm();
        
        if (error <= max_reproj_error) {
            sum_squared_error += error * error;
            valid_observations++;
        }
    }
    
    if (valid_observations < min_observations) {
        return metrics; // 유효한 관측 부족
    }
    
    metrics.reprojection_rms = std::sqrt(sum_squared_error / valid_observations);
    
    // 기하학적 일관성 점수 (관측 수와 삼각각 기반)
    const double obs_score = std::min(1.0, metrics.observation_count / 6.0);
    const double angle_score = std::min(1.0, metrics.triangulation_angle / 60.0);
    metrics.geometric_consistency = (obs_score + angle_score) / 2.0;
    
    // 전체 유효성 판단
    metrics.is_valid = (metrics.reprojection_rms <= max_reproj_error && 
                       metrics.triangulation_angle >= min_triang_angle &&
                       valid_observations >= min_observations);
    
    return metrics;
}

// 모든 GCP 품질 평가 및 필터링
inline std::vector<openMVG::IndexT> FilterLowQualityGCPs(
    openMVG::sfm::SfM_Data& sfm_data,
    double quality_threshold = 0.7,  // 품질 임계값 (0-1)
    double max_reproj_error = 3.0)   // 최대 재투영 오차
{
    std::vector<openMVG::IndexT> gcp_ids_to_remove;
    
    for (const auto& gcp_pair : sfm_data.control_points) {
        const auto& gcp_id = gcp_pair.first;
        const auto& gcp = gcp_pair.second;
        
        auto metrics = EvaluateGCPQuality(gcp, sfm_data, 15.0, max_reproj_error, 3);
        
        if (!metrics.is_valid || metrics.geometric_consistency < quality_threshold) {
            gcp_ids_to_remove.push_back(gcp_id);
            std::cout << "Removing low-quality GCP " << gcp_id 
                      << " (consistency: " << metrics.geometric_consistency 
                      << ", reproj_rms: " << metrics.reprojection_rms 
                      << ", angle: " << metrics.triangulation_angle << "°)" << std::endl;
        }
    }
    
    // 품질이 낮은 GCP 제거
    for (auto gcp_id : gcp_ids_to_remove) {
        sfm_data.control_points.erase(gcp_id);
    }
    
    return gcp_ids_to_remove;
}

} // namespace gcp_quality
