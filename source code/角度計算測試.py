import math

def calculate_angle(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    angle_rad = math.atan2(y2 - y1, x2 - x1)
    angle_deg = math.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360
    return angle_deg

def calculate_turning_angles(points):
    angles = []
    for i in range(1, len(points) - 1):
        start = points[i-1]
        middle = points[i]
        end = points[i+1]

        angle_start_to_middle = calculate_angle(start, middle)
        angle_middle_to_end = calculate_angle(middle, end)

        # 計算轉向角度
        turn_angle = angle_middle_to_end - angle_start_to_middle
        if turn_angle > 180:
            turn_angle -= 360  # 轉為逆時針角度
        elif turn_angle < -180:
            turn_angle += 360  # 轉為順時針角度

        # 判斷轉向方向
        direction = "順時針" if turn_angle < 0 else "逆時針"
        angles.append((direction, abs(turn_angle)))

    return angles


# 測試點
points = [(0, 0), (5, 0), (0, 5), (5, 5)]
turning_angles = calculate_turning_angles(points)

# 輸出每段之間的轉向角度
for i, (direction, angle) in enumerate(turning_angles):
    print(f"從點 {points[i]} 到 {points[i + 1]} 再到 {points[i + 2]}，角度為 {direction} {angle:.1f} 度")
