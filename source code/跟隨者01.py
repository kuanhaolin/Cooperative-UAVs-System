import time
import airsim
import numpy as np
import math


# def calculate_right_offset(yaw, offset_distance):
#     right_x = -math.sin(yaw)  # 右方的x分量
#     right_y = math.cos(yaw)   # 右方的y分量
#     return np.array([right_x, right_y, 0.0]) * offset_distance


# def calculate_acceleration_and_yaw(a, b, speed=5.0):
#     delta = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]])
#     distance = np.linalg.norm(delta)
#
#     if distance < 0.1:
#         return 0, 0, 0, 0
#     direction = delta / distance
#     velocity = speed * direction
#     ax, ay, az = velocity[0], velocity[1], velocity[2]
#     yaw = math.atan2(-ay, ax)
#     return ax, ay, az, yaw

def follow_leader_path(client, leader_name, follower_name, offset_distance, speed=2.0):
    # leader_trajectory = []  # 記錄 leader 的路徑
    # follower_trajectory = []  # 記錄 follower 的路徑

    while True:
        #獲取leader的位置和偏航角
        leader_state = client.getMultirotorState(vehicle_name=leader_name)
        # leader_position = np.array([
        #     leader_state.kinematics_estimated.position.x_val,
        #     leader_state.kinematics_estimated.position.y_val,
        #     leader_state.kinematics_estimated.position.z_val
        # ])
        leader_orientation = leader_state.kinematics_estimated.orientation
        yaw = airsim.to_eularian_angles(leader_orientation)[2]
        #print(f"{leader_position}, {math.degrees(yaw):.2f}")

        #follower_state = client.getMultirotorState(vehicle_name=follower_name)
        follower_position = np.array([
            leader_state.kinematics_estimated.position.x_val - 2*math.sin(yaw),
            leader_state.kinematics_estimated.position.y_val + 2*math.cos(yaw),
            leader_state.kinematics_estimated.position.z_val,
        ])
        print(follower_position)

        #target_pos = np.array([leader_position[0]-2*math.sin(yaw), leader_position[1]+2*math.cos(yaw), -1])

        client.moveToPositionAsync(follower_position[0], follower_position[1], -1, 3, vehicle_name="Drone2")


    #     leader_orientation = leader_state.kinematics_estimated.orientation
    #     yaw = airsim.to_eularian_angles(leader_orientation)[2]  # 偏航角
    #
    #     #將leader當前位置加入軌跡
    #     leader_trajectory.append(leader_position)
    #
    #     # 如果有至少一個歷史點，計算目標位置
    #     if len(leader_trajectory) > 1:
    #         # 取得 leader 的上一個位置
    #         prev_leader_position = leader_trajectory[-2]
    #         direction_vector = leader_position - prev_leader_position
    #         direction_yaw = math.atan2(direction_vector[1], direction_vector[0])
    #
    #         # 根據 leader 的運動方向計算右側偏移
    #         right_offset = calculate_right_offset(direction_yaw, offset_distance)
    #         target_position = leader_position + right_offset
    #     else:
    #         # 初始時，目標位置直接用偏移
    #         right_offset = calculate_right_offset(yaw, offset_distance)
    #         target_position = leader_position + right_offset
    #
    #     # 獲取 follower 當前位置
    #     follower_state = client.getMultirotorState(vehicle_name=follower_name)
    #     follower_position = np.array([
    #         follower_state.kinematics_estimated.position.x_val,
    #         follower_state.kinematics_estimated.position.y_val,
    #         follower_state.kinematics_estimated.position.z_val
    #     ])
    #
    #     # 計算加速度與偏航角
    #     ax, ay, az, yaw = calculate_acceleration_and_yaw(follower_position, target_position, speed)
    #
    #     # 移動 follower
    #     client.moveByVelocityAsync(ax, ay, az, 0.1, vehicle_name=follower_name).join()

        # # 記錄 follower 的路徑
        # follower_trajectory.append((follower_position[0], follower_position[1], follower_position[2]))
        # client.simPlotLineStrip(
        #     [airsim.Vector3r(*pos) for pos in follower_trajectory],
        #     color_rgba=[0, 1, 0, 1],
        #     thickness=5.0,
        #     is_persistent=True
        # )
        #
        # # 繪製 leader 的路徑
        # client.simPlotLineStrip(
        #     [airsim.Vector3r(*pos) for pos in leader_trajectory],
        #     color_rgba=[1, 0, 0, 1],
        #     thickness=5.0,
        #     is_persistent=True
        # )
        #
        # time.sleep(0.1)


def main():
    #連接到AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    leader_name = "Drone0"
    follower_name = "Drone2"

    #初始化follower
    client.enableApiControl(True, vehicle_name=follower_name)
    client.armDisarm(True, vehicle_name=follower_name)

    # follower 起飛
    follower_z = -1  # 高度固定
    client.takeoffAsync(vehicle_name=follower_name).join()
    client.moveToZAsync(follower_z, 2, vehicle_name=follower_name).join()

    #定義 follower 相對 leader 右側的偏移距離
    offset_distance = 2.0  # 距離右側 2 米

    try:
        follow_leader_path(client, leader_name, follower_name, offset_distance, speed=2.0)
    except KeyboardInterrupt:
        print("中斷跟隨動作。")
    finally:
        # 降落
        client.landAsync(vehicle_name=follower_name).join()
        client.armDisarm(False, vehicle_name=follower_name)
        client.enableApiControl(False, vehicle_name=follower_name)

if __name__ == "__main__":
    main()
