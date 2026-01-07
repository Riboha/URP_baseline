# URP_glim
original repo
https://github.com/koide3/glim.git

## Run
```bash
ros2 run glim_ros glim_rosbag [bag file path]
```

## Eval
SLAM 결과는 /tmp/dump 경로에 저장


## Code 수정할 부분
### Covariance regularization (matching error)
glim/src/glim/common/cloud_covariance_estimation.cpp

코드 수정 후 다시 빌드 -> 실험
