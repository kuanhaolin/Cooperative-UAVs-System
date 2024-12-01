import time
import airsim
import math
import numpy as np
"""
  +x
-y + +y
  -y
"""

#計算從當前位置到目標位置的加速度和偏航角
def calculate_acceleration_and_yaw(a, b, speed=5.0):

    delta = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]]) #兩點向量
    distance = np.linalg.norm(delta) #歐式距離

    if distance < 0.1:
        return 0, 0, 0, 0
    direction = delta / distance
    velocity = speed * direction
    ax, ay, az = velocity[0], velocity[1], velocity[2]
    yaw = math.atan2(-ay, ax)  #計算偏航角
    return ax, ay, az, yaw


def fly_to_point(client, P1, P2, height, speed=2.0):

    current_pos = P1
    target_pos = np.array([P2[0], P2[1], height])

    #初始化軌跡列表
    trajectory = [airsim.Vector3r(*current_pos)]

    while True:
        #計算加速度和偏航角
        ax, ay, az, yaw = calculate_acceleration_and_yaw(current_pos, target_pos, speed)
        client.moveByRollPitchYawZAsync(0, 0, yaw, height, 0.1).join()
        client.moveByVelocityAsync(ax, ay, az, 0.1).join()

        #更新當前位置
        state = client.getMultirotorState()
        current_pos = np.array([
            state.kinematics_estimated.position.x_val,
            state.kinematics_estimated.position.y_val,
            state.kinematics_estimated.position.z_val
        ])

        #記錄軌跡點
        trajectory.append(airsim.Vector3r(current_pos[0], current_pos[1], current_pos[2]))
        client.simPlotLineStrip(trajectory, color_rgba=[0, 0, 1, 1], thickness=5.0, is_persistent=True)

        #判斷是否接近目標
        distance_to_target = np.linalg.norm(current_pos - target_pos)
        if distance_to_target < 1:
            print("到達目標點")
            break

def main():
    # 連接到 AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    #z軸(固定)
    z = -7
    #起飛
    client.takeoffAsync().join()
    client.moveToZAsync(z, 2).join()

    #設定目標點
    points = [(0,0,z), (5,5,z), (5,-5,z), (-5,5,z)]

    #畫座標點
    client.simSetTraceLine([1, 0, 0, 1], thickness=5)
    client.simFlushPersistentMarkers()
    points1 = [airsim.Vector3r(0,0,z), airsim.Vector3r(5,5,z), airsim.Vector3r(5,-5,z), airsim.Vector3r(-5,5,z)]
    client.simPlotPoints(points1, color_rgba=[0, 1, 0, 1], size=30, is_persistent=True)

    #trajectory = []

    for i in range(len(points) - 1):
        fly_to_point(client, points[i], points[i + 1], z)
        time.sleep(0.5)

    #降落
    client.landAsync(2).join()
    client.armDisarm(False)
    client.enableApiControl(False)

if __name__ == "__main__":
    main()