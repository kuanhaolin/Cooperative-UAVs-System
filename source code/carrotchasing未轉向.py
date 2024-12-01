import airsim
import numpy as np
import math
import time
import matplotlib.pyplot as plt

full_path = []

def move_by_path(client, Va, Path, Pz, delta=0.8, K=0.8, K2=0, dt=0.02):
    #距離
    def distance(A, B):
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)
    #角度
    def myatan(A, B):
        x1, y1, x2, y2 = A[0], A[1], B[0], B[1]
        if x1 != x2:
            if x1 > x2:
                return math.atan((y1 - y2) / (x1 - x2)) + math.pi
            else:
                return math.atan((y1 - y2) / (x1 - x2))
        if x1 == x2 and y1 == y2:
            return None
        if x1 == x2 and y1 != y2:
            if y1 > y2:
                return -math.pi / 2
            else:
                return math.pi / 2

    state = client.simGetGroundTruthKinematics()
    psi = airsim.to_eularian_angles(state.orientation)[2]
    Px = state.position.x_val
    Py = state.position.y_val
    Wb = [Px, Py]

    for i in range(len(Path)):
        Wa = Wb
        Wb = [Path[i].x_val, Path[i].y_val]
        theta = myatan(Wa, Wb)
        while True:
            theta_u = myatan(Wa, [Px, Py])
            if theta_u == None:
                theta_u = theta
            beta = theta - theta_u
            Ru = distance(Wa, [Px, Py])
            R = Ru * math.cos(beta)
            e = Ru * math.sin(beta)
            print(Px, Py, e)
            xt = Wa[0] + (R + delta) * math.cos(theta)
            yt = Wa[1] + (R + delta) * math.sin(theta)
            if i == len(Path) - 1:
                if (Px - Wb[0]) * (Wb[0] - Wa[0]) > 0 \
                        or (Py - Wb[1]) * (Wb[1] - Wa[1]) > 0:
                    break
            elif (xt - Wb[0]) * (Wb[0] - Wa[0]) > 0 \
                    or (yt - Wb[1]) * (Wb[1] - Wa[1]) > 0:
                break
            psi_d = myatan([Px, Py], [xt, yt])
            u = K * (psi_d - psi) * Va + K2 * e
            if u > 1:  #限制u的范围
                u = 1
            psi = psi_d
            Vy = Va * math.sin(psi) + u * dt
            if abs(Vy) >= Va:
                Vy = Va
                Vx = 0
            else:
                Vx = np.sign(math.cos(psi)) * math.sqrt(Va ** 2 - Vy ** 2)
            client.moveByVelocityZAsync(Vx, Vy, Pz, dt).join()
            #繪製路線
            full_path.append([Px, Py])
            plot_p1 = [airsim.Vector3r(Px, Py, Pz)]
            state = client.simGetGroundTruthKinematics()
            Px = state.position.x_val
            Py = state.position.y_val
            plot_p2 = [airsim.Vector3r(Px, Py, Pz)]
            client.simPlotArrows(plot_p1, plot_p2, arrow_size=8.0, color_rgba=[0.0, 0.0, 1.0, 1.0])
            client.simPlotLineList(plot_p1 + plot_p2, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)


client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
client.moveToZAsync(-7, 1).join()
client.simSetTraceLine([1, 0, 0, 1], thickness=5) #軌跡呈現
client.simFlushPersistentMarkers()

points = [airsim.Vector3r(5, 5, -7),
          airsim.Vector3r(5, -5, -7),
          airsim.Vector3r(-5, 5, -7)]
client.simPlotPoints(points, color_rgba=[0, 1, 0, 1], size=30, is_persistent=True)

move_by_path(client, 2, points, -7)

#client.goHomeAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)

# 繪製飛行路徑
x_coords, y_coords = zip(*full_path)  # 解壓縮為 x, y
plt.figure(figsize=(8, 8))
plt.plot(y_coords, x_coords, marker="o", color="b", label="Flight Path")
plt.scatter([p.y_val for p in points], [p.x_val for p in points], color="r", label="Waypoints", s=100)
plt.xlabel("Y-axis")
plt.ylabel("X-axis")
plt.title("Flight Path Visualization")
plt.legend()
plt.grid(True)
plt.show()