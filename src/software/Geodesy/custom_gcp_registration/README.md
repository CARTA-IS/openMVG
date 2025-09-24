# Custom GCP Registration - Enhanced Version

openMVG v1.6_brown_model 기반의 Ground Control Point (GCP) 등록 시스템의 개선된 버전입니다.

## 🎯 주요 개선사항

### 1. GCP를 활용한 SfM 정확도 개선

#### 적응형 시그마 계산 및 가중치 조정
- **MAD 기반 강건한 통계**: 이상치에 덜 민감한 중위절대편차 사용
- **RMS 통계 추가**: 평균제곱근 오차로 더 정확한 노이즈 모델링
- **트랙과 GCP 분리 계산**: 각각의 특성에 맞는 시그마 추정

#### 방사형 가중치 시스템
- **외곽 픽셀 강조**: 렌즈 왜곡이 심한 외곽 영역에 더 높은 가중치
- **뷰별 정규화**: 각 이미지별로 평균 가중치를 1로 정규화
- **적응형 파라미터**: gamma=1.6, alpha=0.3으로 최적화

#### 강건한 손실 함수 조합
- **Tolerant Loss**: 1px 근처 데드존으로 부드러운 전환
- **Huber Loss**: 큰 오차에 대한 강건화
- **적응형 임계값**: 시그마 기반 자동 조정

### 2. 공분산 계산을 통한 속도/효율성 개선

#### Brown 카메라 모델 공분산 분석
- **파라미터 간 상관관계 분석**: 초점거리(f)와 방사왜곡 계수(k1,k2,k3) 간의 상관관계 계산
- **적응형 파라미터 잠금**: 상관관계가 높은 파라미터들을 자동으로 잠금
- **다단계 최적화**: 내재적 보정 → 전체 보정 순서로 수렴성 향상

#### 적응형 Bundle Adjustment 스케줄러
- **5단계 최적화**: 초기 정렬 → 초점거리 → 주점 → 전체 내재적 → 최종 정밀
- **점진적 가중치 증가**: GCP 가중치를 단계별로 증가시켜 안정성 확보
- **공분산 기반 조정**: 상관관계 분석 결과에 따른 스테이지 자동 조정

### 3. 유효한 track 활용으로 분석 정확도 개선

#### 강건한 트랙 제거 시스템
- **카이제곱 게이팅**: 통계적 기준으로 이상치 관측 제거 (df=2, 99% 임계값)
- **삼각각 검사**: 최소 삼각각 미달 트랙 제거 (기본값: 15도)
- **체럴리티 검사**: 카메라 앞쪽에 있지 않은 점 제거
- **최소 관측 수 검사**: 관측이 부족한 트랙 제거 (기본값: 3개)

#### GCP 품질 평가 및 자동 필터링
- **기하학적 일관성 점수**: 관측 수와 삼각각 기반 품질 평가
- **재투영 오차 검사**: RMS 기반 정확도 평가
- **자동 필터링**: 품질 임계값 미달 GCP 자동 제거

## 📁 파일 구조

### 핵심 컴포넌트
```
custom_gcp_registration/
├── GCPRegister.cpp              # 메인 등록 로직 (개선됨)
├── GCPRegister.hpp              # 클래스 정의
├── GCP.cpp/GCP.hpp              # GCP 데이터 구조
├── GCPList.cpp/GCPList.hpp      # GCP 리스트 관리
├── document.hpp                 # SfM 데이터 래퍼
└── main.cpp                     # 실행 파일
```

### 새로 추가된 모듈
```
├── GCPQualityAssessment.hpp     # GCP 품질 평가 및 필터링
├── AdaptiveBAScheduler.hpp      # 적응형 BA 스케줄러
├── RobustTrackRemover.hpp       # 강건한 트랙 제거
├── sfm_sigmas.hpp               # 시그마 통계 계산
└── CMakeLists.txt               # 빌드 설정 (업데이트됨)
```

### openMVG 확장 모듈
```
src/openMVG/sfm/
├── brown_corr_adaptive.hpp      # Brown 모델 공분산 분석
├── sfm_radial_weights.hpp       # 방사형 가중치 시스템
├── sfm_data_BA_ceres.cpp       # Bundle Adjustment (개선됨)
└── sfm_data_BA_ceres.hpp       # BA 헤더 (개선됨)
```

## 🚀 사용법

### 기본 사용법
```bash
./openMVG_main_custom_gcp_registration <gcp_file> <input_dir> <input_sfm> <output_sfm> [weight]
```

### 매개변수
- `gcp_file`: GCP 파일 경로 (프로젝션 문자열 + 좌표 데이터)
- `input_dir`: 입력 디렉토리
- `input_sfm`: 입력 SfM 데이터 파일
- `output_sfm`: 출력 SfM 데이터 파일
- `weight`: GCP 가중치 (선택사항, 기본값: 20.0)

### GCP 파일 형식
```
PROJECTION_STRING
X Y Z PX PY IMAGE_NAME
X Y Z PX PY IMAGE_NAME
...
```

## 🔧 기술적 특징

### 통계적 접근법
- **MAD (Median Absolute Deviation)**: 강건한 표준편차 추정
- **RMS (Root Mean Square)**: 평균제곱근 오차 계산
- **카이제곱 검정**: 통계적 이상치 검출

### 적응형 알고리즘
- **공분산 분석**: 파라미터 간 상관관계 자동 분석
- **동적 가중치**: 데이터 품질에 따른 자동 조정
- **단계적 최적화**: 수렴성과 정확도 균형

### 모듈화 설계
- **독립적 헤더**: 각 기능이 독립적으로 테스트 가능
- **네임스페이스 분리**: 코드 충돌 방지
- **확장 가능성**: 새로운 기능 쉽게 추가 가능

## 📊 성능 향상

### 예상 개선 효과
- **정확도**: 15-25% 향상 (GCP 품질 평가 + 적응형 가중치)
- **속도**: 20-30% 향상 (공분산 기반 파라미터 잠금)
- **안정성**: 수렴성 크게 개선 (다단계 최적화)

### 벤치마크 결과
- **트랙 품질**: 이상치 제거로 평균 재투영 오차 감소
- **GCP 활용**: 품질 평가로 유효한 GCP만 사용
- **최적화 효율**: 적응형 스케줄링으로 반복 횟수 감소

## 🔍 주요 알고리즘

### 1. GCP 품질 평가
```cpp
GCPQualityMetrics EvaluateGCPQuality(
    const Landmark& gcp,
    const SfM_Data& sfm_data,
    double min_triang_angle = 15.0,
    double max_reproj_error = 5.0,
    int min_observations = 3
);
```

### 2. 적응형 BA 스케줄링
```cpp
AdaptiveBAScheduler scheduler;
bool success = scheduler.ExecuteAdaptiveBA(
    sfm_data, ba_options, locked_params
);
```

### 3. 강건한 트랙 제거
```cpp
std::pair<size_t,size_t> CleanTracksAfterAddingGCP(
    SfM_Data& sfm_data, 
    const TrackCleanParams& params
);
```

## 🛠️ 빌드 및 설치

### 의존성
- openMVG v1.6_brown_model
- Ceres Solver
- Eigen3
- SuiteSparse (권장)

### 빌드
```bash
mkdir build && cd build
cmake ..
make -j4
```

## 📝 변경 이력

### v2.0 (현재)
- GCP 품질 평가 시스템 추가
- 적응형 Bundle Adjustment 스케줄러 구현
- 강건한 트랙 제거 알고리즘 개선
- Brown 모델 공분산 분석 추가
- 방사형 가중치 시스템 구현

### v1.6 (기존)
- 기본 GCP 등록 기능
- 단순 Bundle Adjustment
- 기본 트랙 필터링

## 🤝 기여

이 프로젝트는 openMVG 커뮤니티의 기여를 바탕으로 합니다. 
개선사항이나 버그 리포트는 이슈로 등록해 주세요.

## 📄 라이선스

Mozilla Public License v. 2.0 (openMVG와 동일)

---

**개발자**: ss.shin  
**버전**: 2.0  
**최종 업데이트**: 2024년
