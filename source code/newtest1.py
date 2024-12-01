import airsim
import numpy as np
import cv2
import os

# 連接到 AirSim 模擬器
client = airsim.VehicleClient()
client.confirmConnection()

# 左右相機的名稱
left_camera_name = "2"
right_camera_name = "1"
image_type = airsim.ImageType.Scene

# 獲取相機影像
def get_camera_image(camera_name):
    raw_image = client.simGetImage(camera_name, image_type)
    if raw_image:
        return cv2.imdecode(airsim.string_to_uint8_array(raw_image), cv2.IMREAD_COLOR)
    else:
        return None

# 設定相機參數（需要根據實際的相機參數進行調整）
focal_length = 512  # 焦距（根據相機校正的參數）
baseline = 1  # 兩個相機之間的距離（米）

# 計算距離的函數
def calculate_depth(disparity, focal_length, baseline):
    # 深度 = (焦距 * 基線距離) / 視差
    depth = np.zeros(disparity.shape, np.float32)
    disparity[disparity < 1] = 1  # 避免 0 或過小的視差，設為最小值 1
    depth = (focal_length * baseline) / disparity
    return depth

def save_depth_to_csv(depth_map, filename="depth_data.csv"):
    # 檢查是否已有文件，若有則覆蓋
    file_exists = os.path.isfile(filename)

    # 保存數據
    np.savetxt(filename, depth_map, delimiter=",")
    print(f"深度數據已保存到: {filename}")

while True:
    # 捕捉左右相機的影像
    left_image = get_camera_image(left_camera_name)
    right_image = get_camera_image(right_camera_name)

    if left_image is not None and right_image is not None:
        # 轉換為灰度圖像
        gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # 計算視差圖，增加 numDisparities 和調整 blockSize
        stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)
        disparity = stereo.compute(gray_left, gray_right)

        # 顯示視差圖以檢查其質量
        disparity_display = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disparity_display = np.uint8(disparity_display)
        cv2.imshow("Disparity", disparity_display)

        # 計算深度圖（距離圖）
        depth_map = calculate_depth(disparity, focal_length, baseline)

        # 正規化深度圖以便顯示
        min_depth, max_depth = np.min(depth_map), np.max(depth_map)
        depth_map_display = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
        depth_map_display = np.uint8(depth_map_display)

        # 顯示原圖和深度圖
        cv2.imshow("Left Image", left_image)
        cv2.imshow("Right Image", right_image)
        cv2.imshow("Depth Map", depth_map_display)

        save_depth_to_csv(depth_map, "depth_data.csv")

        # 偵測到按鍵 q 結束
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# 關閉所有視窗
cv2.destroyAllWindows()
