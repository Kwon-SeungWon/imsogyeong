#!/usr/bin/env python3

import cv2
import numpy as np


block_scale = 31


if __name__ == "__main__":
    f = open("/home/kwon/catkin_ws/src/imsogyeong_action/map.txt", "r")
    #row, col = map(int, f.readline().split())
    row = 10
    col = 10

    read_map = [[0 for j in range(col)] for i in range(row)]
    for i in range(row):
        read_map[i] = f.readline().split()

    canvas_map = np.zeros((row * block_scale, col * block_scale, 3), np.uint8) + 255     # 흰 배경으로 초기 맵 선언 # size가 아닌 (row, col)로 넣네

    for r in range(row):
        for c in range(col):
            center_x = c * block_scale + block_scale // 2   # center_x를 c로 구하는 이유: x좌표는 픽셀 size이기 때문
            center_y = r * block_scale + block_scale // 2

            # hsv 검사 넣기

            if read_map[r][c] == '1':     # block road
                cv2.rectangle(canvas_map, (center_x - block_scale // 2, center_y - block_scale // 2), (center_x + block_scale // 2, center_y + block_scale // 2), (0, 0, 0), -1)
            elif read_map[r][c] == '4':   # 선호하는 대상
                cv2.rectangle(canvas_map, (center_x - block_scale // 2, center_y - block_scale // 2), (center_x + block_scale // 2, center_y + block_scale // 2), (0, 255, 0), -1)
            elif read_map[r][c] == '5':   # 선호하지 않는 대상
                cv2.rectangle(canvas_map, (center_x - block_scale // 2, center_y - block_scale // 2), (center_x + block_scale // 2, center_y + block_scale // 2), (0, 0, 255), -1)
            elif read_map[r][c] == '2':   # start point
                cv2.circle(canvas_map, (center_x, center_y), block_scale // 4, (0, 0, 0), 2)
            elif read_map[r][c] == '3':   # finish point
                cv2.circle(canvas_map, (center_x, center_y), block_scale // 4, (0, 0, 0), -1)

            cv2.rectangle(canvas_map, (center_x - block_scale // 2, center_y - block_scale // 2), (center_x + block_scale // 2, center_y + block_scale // 2), (0, 0, 0), 1)

    
    while cv2.waitKey(1) != ord('q'):
        cv2.imshow("iamge_color_name", canvas_map)

    cv2.imwrite("/home/kwon/catkin_ws/src/imsogyeong_action/map.jpg", canvas_map)
    print('save ok')