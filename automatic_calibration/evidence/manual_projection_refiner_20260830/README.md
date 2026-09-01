# Manual Projection Refiner 대표 증거

`feature_projection.png`은 카메라 edge mask(흰색)와 manual-reference 기반 LiDAR
feature projection(녹색)을 함께 표시한 진단 이미지다. 이 파일은 projection과
z-buffer/가시성 처리 상태를 확인하기 위한 대표 산출물이며 제품 RT 승인 증거가 아니다.

소스와 구현 설명은 `exp-manual-reference-refiner` 브랜치의
`manual_calibration/apps/manual_projection_refiner.cpp`와
`manual_calibration/docs/MANUAL_PROJECTION_REFINER.md`를 확인한다.
