import numpy as np
import matplotlib.pyplot as plt

# 参数初始化，确保使用浮点数定义起始位置，避免数据类型不匹配的问题
#goal = np.array([10.0, 10.0])  # 目标位置
goals = [np.array([3.0, 6.0]), np.array([7.0, 4.0]), np.array([10.0, 10.0])]
obstacles = [np.array([5.0, 4.0]), np.array([3.0, 8.0]), np.array([6.0, 6.0]), np.array([6.0, 7.0]), np.array([6.0, 8.0]), np.array([9.0, 9.0])]  # 障碍物位置列表
start = np.array([0.0, 0.0])  # 起始位置

# 势场参数
attraction_coefficient = 1  # 吸引力系数，略微增加吸引力
repulsion_coefficient = 10  # 显著增加斥力系数
repulsion_threshold = 1  # 增加斥力作用的距离阈值
max_iterations = 1000  # 最大迭代次数，防止死循环
step_size = 0.1  # 机器人每一步的步长


# 计算吸引力
def calculate_attraction(position, goal, coefficient):
    return coefficient * (goal - position)


# 计算斥力
def calculate_repulsion(position, obstacles, coefficient, threshold):
    repulsion_force = np.array([0.0, 0.0])
    for obstacle in obstacles:
        obstacle_vec = position - obstacle
        distance = np.linalg.norm(obstacle_vec)
        if distance < threshold:
            # 增加距离判断，以防除以0
            if distance == 0:
                distance = 0.01  # 避免除以0
            repulsion_force += coefficient * (1 / threshold - 1 / distance) * (obstacle_vec / distance ** 3)
    return repulsion_force


# 主函数
def main():
    position = start.copy()
    path = [start.copy()]

    for goal in goals:
        for _ in range(max_iterations):
            attraction = calculate_attraction(position, goal, attraction_coefficient)
            repulsion = calculate_repulsion(position, obstacles, repulsion_coefficient, repulsion_threshold)
            force = attraction - repulsion  # 合成总力
            # 更新位置，增加步长控制
            position += step_size * force / np.linalg.norm(force)
            path.append(position.copy())

            if np.linalg.norm(position - goal) <= 0.5:  # 检查是否足够接近目标
                break

    # 转换路径为NumPy数组以便绘图
    path = np.array(path)
    plt.plot(path[:, 0], path[:, 1], '-o', label='Path')
    plt.plot(goal[0], goal[1], 'r*', label='Goal')
    for i, goal in enumerate(goals):
        plt.plot(goal[0], goal[1], 'r*', label=f'Goal {i + 1}')
    for obstacle in obstacles:
        plt.plot(obstacle[0], obstacle[1], 'ks', label='Obstacle')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Path Planning with APF')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()