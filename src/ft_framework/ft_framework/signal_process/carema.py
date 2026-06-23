import cv2

# 1. 指定设备索引。如果设备是 /dev/video1，则 index 为 1
camera_idx = 0 

# 2. 打开摄像头，并强制指定使用 V4L2 后端
cap = cv2.VideoCapture(camera_idx, cv2.CAP_V4L2)

if not cap.isOpened():
    print(f"无法打开摄像头 /dev/video{camera_idx}")
    exit()

# 3. 配置摄像头参数 (根据前面 v4l2-ctl 查出来的参数修改)
# 设置分辨率为 1280x720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
# 尝试设置像素格式为 MJPEG (有些摄像头在大分辨率下 YUYV 帧率很低，MJPEG 帧率高)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
# 设置帧率
cap.set(cv2.CAP_PROP_FPS, 30)

print("开始获取视频流，按 'q' 退出...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("未能接收到图像帧 (stream end?)")
        break

    # 在这里可以对 frame (numpy 数组) 进行你的 AI 推理或图像处理

    # 显示图像
    cv2.imshow('Jetson Orin V4L2 USB Camera', frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
