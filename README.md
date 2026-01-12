# URP_glim
original repo
https://github.com/koide3/glim.git

## Run
```bash
ros2 run glim_ros glim_rosbag [bag file path]
```

## Eval
SLAM 결과는 /tmp/dump 경로에 저장

## Settings
### ROS Topic
- glim/config/config_ros.json
  - "imu_topic", "points_topic" 수정
- LiDAR/IMU Topic 이름을 bag에 녹화된 것에 맞춰 수정
- Bag에 녹화된 Topic 이름은 아래 명령어로 topic list를 출력하여 확인
  ```bash
  ros2 topic list
  ```

### LiDAR - IMU Extrinsic
- glim/config/config_sensors.json
  - "T_lidar_imu" 수정
- LiDAR와 IMU 사이의 위치 관계를 config file에 설정해줘야 함
- MULRAN dataset 사이트에서 참고하여 설정

### !!! Config파일 수정 후 빌드 다시 해줘야 함!

## Code 수정할 부분
### Covariance regularization (matching error)
glim/src/glim/common/cloud_covariance_estimation.cpp

코드 수정 후 다시 빌드 -> 실험
