# 感谢大佬 @港来港去 的开源分享
import cv2
import mediapipe as mp
import numpy as np
import serial

# 常量设置
joint_list = [
    [19, 18, 17],
    [18, 17, 0],
    [15, 14, 13],
    [14, 13, 0],
    [11, 10, 9],
    [10, 9, 0],
    [7, 6, 5],
    [6, 5, 0],
    [4, 3, 2],
    [3, 2, 1],
    [2, 1, 0]
]  # 计算关节角度所使用的的手部节点
angle_ranges = [
    [0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0],
    [0,45, 50,60, 65,75, 80,90, 95,105, 110,120, 125,135, 140,150, 155,165, 170,180],
    [0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0],
    [0,50, 55,65, 69,79, 84,94, 98,108, 113,123, 127,137, 142,152, 156,166, 171,180],
    [0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0],
    [0,60, 64,73, 77,86, 90,99, 103,112, 116,125, 129,138, 142,151, 155,165, 169,178],
    [0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0],
    [0,80, 83,91, 94,102, 105,113, 116,124, 127,135, 138,146, 149,157, 160,168, 170,180],
    [0,70, 74,82, 86,94, 98,106, 110,118, 122,130, 134,142, 146,152, 156,164, 168,180],
    [0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0],
    [0,145, 145,146, 146,147, 147,148, 148,149, 149,150, 150,151, 151,152, 152,153, 153,180],
]  # 角度映射分段函数的分段范围

# 变量初始化
joint_angle = [180.0] * 110    # 手指关节角度
Converted_angle = ["9"] * 11

# 初始化串口
ser = serial.Serial('COM4', 9600, timeout=1)  # 不插入串口设备无法执行代码！记得修改自己的串口号和波特率

# 初始化Mediapipe
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic

# 初始化摄像头
cap = cv2.VideoCapture(0)

with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
    while cap.isOpened():
        success, image = cap.read()  # 截取摄像头中的一帧画面
        if not success:
            print("Ignoring empty camera frame.")
            break
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # 对图像进行色彩空间的转换
        results = holistic.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # 转换回来
        mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                  landmark_drawing_spec=mp_drawing_styles.get_default_hand_landmarks_style())  # 绘制关节点
        # 监测到右手，执行
        if results.right_hand_landmarks:
            RHL = results.right_hand_landmarks
            # 计算角度
            i = 0
            ser.write("B0,".encode("utf8"))  # 用于体感手套控制机械手的老代码进行起始检测
            for joint in joint_list:  # 生成向量
                a = np.array([RHL.landmark[joint[0]].x, RHL.landmark[joint[0]].y])
                b = np.array([RHL.landmark[joint[1]].x, RHL.landmark[joint[1]].y])
                c = np.array([RHL.landmark[joint[2]].x, RHL.landmark[joint[2]].y])
                # 计算弧度
                radians_fingers = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
                angle = np.abs(radians_fingers * 180.0 / np.pi)  # 弧度转角度
                if angle > 180.0:  # 防止角度超出范围
                    angle = 360 - angle
                joint_angle[i] = round(angle, 1)  # round()函数用于将浮点数四舍五入到指定的小数位数。
                for j in range(0, 18, 2):  # 把角度映射到0~9
                    if angle_ranges[i][j] <= joint_angle[i] <= angle_ranges[i][j + 1]:
                        Converted_angle[i] = str(j // 2)
                if i in [1, 3, 5, 7, 8, 10]:
                    ser.write(Converted_angle[i].encode("utf8"))    # 串口输出读到的数据，如果没有读到新的数据则输出上次的数据或者初始数据
                    ser.write(",".encode("utf8"))  # 用逗号分隔串口输出读到的数据，用于配套手套控制机械手的代码
                cv2.putText(image, str(round(angle, 2)), tuple(np.multiply(b, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)  # 将角度值放置在关节点旁边
                i = i + 1
            ser.write("0E\r\n".encode("utf8"))  # 用于体感手套控制机械手的老代码进行结束检测
            print(joint_angle[0], joint_angle[1], joint_angle[2], joint_angle[3], joint_angle[4], joint_angle[5],
                  joint_angle[6], joint_angle[7], joint_angle[8], joint_angle[9], joint_angle[10])  # 把角度输出到信息栏
            print(Converted_angle[1], Converted_angle[3], Converted_angle[5],
                  Converted_angle[7], Converted_angle[8], Converted_angle[10])  # 把映射后的角度输出到信息栏
        cv2.imshow('Mediapipe Holistic', image)  # 取消镜面翻转
        if cv2.waitKey(5) == ord('q'):
            break
cap.release()
