import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

from evo.core.trajectory import PoseTrajectory3D
from evo.core import sync, metrics
from evo.tools import plot
import evo.main_ape as main_ape
import matplotlib.pyplot as plt

def load_custom_gt(file_path):
    """GT: [timestamp, r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz]"""
    df = pd.read_csv(file_path, header=None)
    timestamps = df.iloc[:, 0].values
    matrices_flat = df.iloc[:, 1:13].values
    positions = matrices_flat[:, [3, 7, 11]] # tx, ty, tz 추출
    
    timestamps = np.array(timestamps).astype(np.float64) * 1e-9

    # 3x3 회전 행렬들을 쿼터니언으로 변환
    quats = []
    for row in matrices_flat:
        rot_mat = row.reshape(3, 4)[:3, :3]
        quats.append(R.from_matrix(rot_mat).as_quat()) # xyzw
        
    return PoseTrajectory3D(
        positions_xyz=np.array(positions),
        orientations_quat_wxyz=np.array(quats),
        timestamps=np.array(timestamps).astype(np.float64)
    )

def load_custom_est(file_path):
    """EST: [timestamp, x, y, z, qx, qy, qz, qw]"""
    # 공백 혹은 탭 구분자 처리
    df = pd.read_csv(file_path, header=None, sep=r'\s+')
    timestamps = df.iloc[:, 0].values
    positions = df.iloc[:, 1:4].values
    quats = df.iloc[:, 4:8].values # qx, qy, qz, qw
    
    return PoseTrajectory3D(
        positions_xyz=positions,
        orientations_quat_wxyz=quats,
        timestamps=np.array(timestamps).astype(np.float64)
    )

def evaluate_ate(gt_path, est_path, correct_scale=False):
    # 1. 데이터 로드 (PoseTrajectory3D 생성)
    traj_ref = load_custom_gt(gt_path)
    traj_est = load_custom_est(est_path)

    # 2. 타임스탬프 동기화 (제공된 코드의 핵심 로직)
    # max_diff 내에 있는 가장 가까운 데이터끼리 매칭
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.1)

    # 3. ATE 계산 (main_ape 활용)
    # 내부적으로 align과 rmse 계산을 동시에 수행합니다.
    result = main_ape.ape(
        traj_ref, 
        traj_est, 
        est_name='estimated_traj',
        pose_relation=metrics.PoseRelation.translation_part, 
        align=True, 
        correct_scale=correct_scale
    )

    # 결과 요약 출력 (pretty_str 활용)
    print("\n" + "="*50)
    print("ATE Metric Results")
    print("="*50)
    print(result.pretty_str())
    
    # 4. 시각화
    # 정렬된 상태의 궤적을 그리기 위해 복사본 생성 후 정렬
    import copy
    traj_est_aligned = copy.deepcopy(traj_est)
    traj_est_aligned.align(traj_ref, correct_scale=correct_scale)

    fig = plt.figure(figsize=(10, 8))
    # 'reference', 'estimate' 등의 키값이 범례로 사용됨
    traj_by_label = {
        "Ground Truth (Reference)": traj_ref,
        "SLAM Estimate (Aligned)": traj_est_aligned
    }
    
    # XYZ 3D 궤적 플롯
    # plot.trajectories(fig, traj_by_label, plot.PlotMode.xyz)
    plot.trajectories(fig, traj_by_label, plot.PlotMode.xy)
    plt.show()

if __name__ == "__main__":
    GT_FILE = "/home/lairmsi/DCC01/global_pose.csv"
    EST_FILE = "/tmp/dump/traj_lidar.txt"
    
    # RGBD GS-SLAM의 경우 correct_scale=False (절대 크기 보존)
    # Monocular의 경우 True로 설정
    evaluate_ate(GT_FILE, EST_FILE, correct_scale=False)
