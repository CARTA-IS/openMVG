#pragma once
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_BA_ceres.hpp>
#include <openMVG/cameras/Camera_Intrinsics.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace adaptive_ba {

// Bundle Adjustment 단계별 설정
struct BAStage {
    int max_iterations = 50;
    openMVG::cameras::Intrinsic_Parameter_Type intrinsic_opt = 
        openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL;
    openMVG::sfm::Extrinsic_Parameter_Type extrinsic_opt = 
        openMVG::sfm::Extrinsic_Parameter_Type::ADJUST_ALL;
    openMVG::sfm::Structure_Parameter_Type structure_opt = 
        openMVG::sfm::Structure_Parameter_Type::ADJUST_ALL;
    double gcp_weight = 1.0;
    double convergence_threshold = 1e-6;
    std::string description;
};

// 적응형 BA 스케줄러
class AdaptiveBAScheduler {
private:
    std::vector<BAStage> stages_;
    
public:
    AdaptiveBAScheduler() {
        InitializeDefaultStages();
    }
    
    void InitializeDefaultStages() {
        stages_.clear();
        
        // Stage 1: 초기 정렬 (내재적 파라미터 고정)
        BAStage stage1;
        stage1.max_iterations = 30;
        stage1.intrinsic_opt = openMVG::cameras::Intrinsic_Parameter_Type::NONE;
        stage1.gcp_weight = 0.5;  // 낮은 GCP 가중치로 시작
        stage1.description = "Initial alignment (intrinsics fixed)";
        stages_.push_back(stage1);
        
        // Stage 2: 초점거리만 조정
        BAStage stage2;
        stage2.max_iterations = 40;
        stage2.intrinsic_opt = openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH;
        stage2.gcp_weight = 1.0;
        stage2.description = "Focal length adjustment";
        stages_.push_back(stage2);
        
        // Stage 3: 초점거리 + 주점 조정
        BAStage stage3;
        stage3.max_iterations = 50;
        stage3.intrinsic_opt = openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH | 
                              openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_PRINCIPAL_POINT;
        stage3.gcp_weight = 2.0;
        stage3.description = "Focal length + principal point";
        stages_.push_back(stage3);
        
        // Stage 4: 전체 내재적 파라미터 조정
        BAStage stage4;
        stage4.max_iterations = 60;
        stage4.intrinsic_opt = openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL;
        stage4.gcp_weight = 3.0;
        stage4.description = "Full intrinsic adjustment";
        stages_.push_back(stage4);
        
        // Stage 5: 최종 정밀 조정
        BAStage stage5;
        stage5.max_iterations = 80;
        stage5.intrinsic_opt = openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL;
        stage5.gcp_weight = 5.0;  // 높은 GCP 가중치로 최종 정밀도 향상
        stage5.convergence_threshold = 1e-8;
        stage5.description = "Final precision adjustment";
        stages_.push_back(stage5);
    }
    
    // 공분산 분석 결과에 따른 스테이지 조정
    void AdaptStagesBasedOnCorrelation(const std::vector<int>& locked_params) {
        if (locked_params.empty()) return;
        
        // 잠긴 파라미터가 있으면 해당 스테이지들을 건너뛰거나 조정
        for (auto& stage : stages_) {
            if (std::find(locked_params.begin(), locked_params.end(), 0) != locked_params.end()) {
                // 초점거리가 잠겼으면 초점거리 조정 스테이지 건너뛰기
                if (stage.intrinsic_opt == openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH) {
                    stage.max_iterations = 10;  // 최소한의 반복만 수행
                }
            }
            
            if (std::find(locked_params.begin(), locked_params.end(), 1) != locked_params.end() ||
                std::find(locked_params.begin(), locked_params.end(), 2) != locked_params.end()) {
                // 주점이 잠겼으면 주점 조정 스테이지 건너뛰기
                if (stage.intrinsic_opt & openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_PRINCIPAL_POINT) {
                    stage.intrinsic_opt = openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH;
                }
            }
        }
    }
    
    // 스테이지 실행
    bool ExecuteStage(const BAStage& stage, 
                     openMVG::sfm::SfM_Data& sfm_data,
                     openMVG::sfm::Bundle_Adjustment_Ceres::BA_Ceres_options& ba_options) {
        
        std::cout << "Executing BA Stage: " << stage.description << std::endl;
        std::cout << "  Max iterations: " << stage.max_iterations << std::endl;
        std::cout << "  GCP weight: " << stage.gcp_weight << std::endl;
        
        // BA 옵션 업데이트
        ba_options.max_num_iterations_ = stage.max_iterations;
        ba_options.parameter_tolerance_ = stage.convergence_threshold;
        
        // Bundle Adjustment 실행
        openMVG::sfm::Bundle_Adjustment_Ceres bundle_adjustment_obj(ba_options);
        openMVG::sfm::Control_Point_Parameter control_point_opt(stage.gcp_weight, true);
        
        return bundle_adjustment_obj.Adjust(sfm_data,
            openMVG::sfm::Optimize_Options(
                stage.intrinsic_opt,
                stage.extrinsic_opt,
                stage.structure_opt,
                control_point_opt));
    }
    
    // 전체 적응형 BA 실행
    bool ExecuteAdaptiveBA(openMVG::sfm::SfM_Data& sfm_data,
                          openMVG::sfm::Bundle_Adjustment_Ceres::BA_Ceres_options& ba_options,
                          const std::vector<int>& locked_params = {}) {
        
        // 공분산 분석 결과에 따른 스테이지 조정
        AdaptStagesBasedOnCorrelation(locked_params);
        
        bool overall_success = true;
        
        for (size_t i = 0; i < stages_.size(); ++i) {
            const auto& stage = stages_[i];
            
            std::cout << "\n=== BA Stage " << (i+1) << "/" << stages_.size() << " ===" << std::endl;
            
            bool stage_success = ExecuteStage(stage, sfm_data, ba_options);
            
            if (!stage_success) {
                std::cout << "Warning: Stage " << (i+1) << " failed, continuing with next stage..." << std::endl;
                overall_success = false;
            }
            
            // 중간 결과 출력
            std::cout << "Stage " << (i+1) << " completed." << std::endl;
        }
        
        return overall_success;
    }
    
    const std::vector<BAStage>& GetStages() const { return stages_; }
};

} // namespace adaptive_ba
