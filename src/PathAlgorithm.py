#!/usr/bin/env python3

import ast
from cmath import sqrt
import math
import re
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D
import cv2

block_scale = 31
map_cell_scale = 2

station = (-1, -1)
path = 0
overlap_prevenstion = [[0 for j in range(10)] for i in range(10)]

dl_d1 = [(0, -1), (0, 1), (-1, 0), (1, 0)]      # 1 cell 차이 - 수직, 수평 이동
di_d1 = [(1, -1), (-1, 1), (-1, -1), (1, 1)]    # 1 cell 차이 - 대각선 이동
d1 = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]     # 1 cell 차이
d2 = [(-2, -2), (-2, -1), (-2, 0), (-2, 1), (-2, 2), (-1, 2), (-1, -2), (0, 2), (0, -2), (1, 2), (1, -2), (2, -2), (2, -1), (2, 0), (2, 1), (2, 2)]     # 2 cell 차이
d3 = [(-3, -3), (-3, -2), (-3, -1), (-3, 0), (-3, 1), (-3, 2), (-3, 3), (-2, -3), (-2, 3), (-1, -3), (-1, 3), (0, -3), (0, 3), (1, -3), (1, 3), (2, -3), (2, 3), (3, -3), (3, -2), (3, -1), (3, 0), (3, 1), (3, 2), (3, 3)]     # 3 cell 차이

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def pose_callback(pose):  
    global station
    global map_cell_scale
    dis = 100000
    present = (-1, -1)

    path_pub = rospy.Publisher('/path_coord', Float32MultiArray, queue_size=10)
    path_c = Float32MultiArray()

    for i in path:
        if d > pow((i[0] * 0.2 + 0.1 - pose.x), 2) + pow((i[1] * 0.2 + 0.1 - pose.y), 2):
            present = (i[0], i[1])
            dis = pow((i[0] * 0.2 + 0.1 - pose.x), 2) + pow((i[1] * 0.2 + 0.1 - pose.y), 2)
    
    path_c.data.clear()
    if (present[0] == 0 and present[1] == 0):
        path_c.data = [-0.1, -0.1, path[present[1]] * 0.2 + 0.1, path[present[0]] * 0.2 + 0.1, path[present[1] + 1] * 0.2 + 0.1, path[present[0] + 1] * 0.2 + 0.1]    
    else:
        path_c.data = [path[present[1] - 1] * 0.2 + 0.1, path[present[0] - 1] * 0.2 + 0.1, path[present[1]] * 0.2 + 0.1, path[present[0]] * 0.2 + 0.1, path[present[1] + 1] * 0.2 + 0.1, path[present[0] + 1] * 0.2 + 0.1]

    path_pub.publish(path_c)

def heuristic(node, goal, D=1, D2=2 ** 0.5):  # Diagonal Distance
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


def aStar(maze, start, end): 
    # startNode와 endNode 초기화
    startNode = Node(None, start)
    endNode = Node(None, end)

    # openList, closedList 초기화
    openList = []
    closedList = []

    # openList에 시작 노드 추가
    openList.append(startNode)

    # endNode를 찾을 때까지 실행
    while openList:

        # 현재 노드 지정
        currentNode = openList[0]
        currentIdx = 0

        # 이미 같은 노드가 openList에 있고, f 값이 더 크면
        # currentNode를 openList안에 있는 값으로 교체
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                currentNode = item
                currentIdx = index

        # openList에서 제거하고 closedList에 추가
        openList.pop(currentIdx)
        closedList.append(currentNode)

        # 현재 노드가 목적지면 current.position 추가하고
        # current의 부모로 이동
        if currentNode == endNode:
            path = []
            current = currentNode
            while current is not None:
                # maze 길을 표시하려면 주석 해제
                # x, y = current.position
                # maze[x][y] = 7 
                path.append(current.position)
                current = current.parent
            return path[::-1]  # reverse

        children = []

        # 인접한 xy좌표 전부 - 수직, 수평
        for newPosition in dl_d1:

            # 노드 위치 업데이트
            nodePosition = (
                currentNode.position[0] + newPosition[0],  # X
                currentNode.position[1] + newPosition[1])  # Y
                
            # 미로 maze index 범위 안에 있어야함
            within_range_criteria = [
                nodePosition[0] > (len(maze) - 1),
                nodePosition[0] < 0,
                nodePosition[1] > (len(maze[len(maze) - 1]) - 1),
                nodePosition[1] < 0,
            ]

            if any(within_range_criteria):  # 하나라도 true면 범위 밖임
                continue

            # 장애물이 있으면 다른 위치 불러오기
            if maze[nodePosition[0]][nodePosition[1]] != 0:
                continue

            new_node = Node(currentNode, nodePosition)
            children.append(new_node)
        # 자식들 모두 loop
        for child in children:

            # 자식이 closedList에 있으면 continue
            if child in closedList:
                continue

            # f, g, h값 업데이트
            child.g = currentNode.g + 1
            child.h = float(math.sqrt((child.position[0] - endNode.position[0]) **
                       2) + ((child.position[1] - endNode.position[1]) ** 2))
            # child.h = heuristic(child, endNode) 다른 휴리스틱
            # print("position:", child.position) 거리 추정 값 보기
            # print("from child to goal:", child.h)
            
            child.f = child.g + child.h

            # 자식이 openList에 있으고, f값이 더 크면 continue
            if len([openNode for openNode in openList
                    if child == openNode and child.f > openNode.f]) > 0:
                continue
                    
            openList.append(child)
        
        # 인접한 xy좌표 전부 - 대각선
        for newPosition in di_d1:
            # 노드 위치 업데이트
            nodePosition = (
                currentNode.position[0] + newPosition[0],  # X
                currentNode.position[1] + newPosition[1])  # Y
                
            # 미로 maze index 범위 안에 있어야함
            within_range_criteria = [
                nodePosition[0] > (len(maze) - 1),
                nodePosition[0] < 0,
                nodePosition[1] > (len(maze[len(maze) - 1]) - 1),
                nodePosition[1] < 0,
            ]

            if any(within_range_criteria):  # 하나라도 true면 범위 밖임
                continue

            # 장애물이 있으면 다른 위치 불러오기
            if maze[nodePosition[0]][nodePosition[1]] != 0:
                continue

            # 대각선 이동이 안 될 경우 continue
            if ((maze[nodePosition[0]][currentNode.position[1]] == 1) or (maze[currentNode.position[0]][nodePosition[1]] == 1)):
                continue

            new_node = Node(currentNode, nodePosition)
            children.append(new_node)
        # 자식들 모두 loop
        for child in children:

            # 자식이 closedList에 있으면 continue
            if child in closedList:
                continue

            # f, g, h값 업데이트
            child.g = currentNode.g + 1.414
            child.h = float(math.sqrt((child.position[0] - endNode.position[0]) **
                       2) + ((child.position[1] - endNode.position[1]) ** 2))
            # child.h = heuristic(child, endNode) 다른 휴리스틱
            # print("position:", child.position) 거리 추정 값 보기
            # print("from child to goal:", child.h)
            
            child.f = child.g + child.h

            # 자식이 openList에 있으고, f값이 더 크면 continue
            if len([openNode for openNode in openList
                    if child == openNode and child.f > openNode.f]) > 0:
                continue
                    
            openList.append(child)

def target(overlap_prevenstion, read_map, start):
    global station

    for serch in d3:
        nx = start[0] + serch[0]
        ny = start[1] + serch[1]

        # 미로 read_map index 범위 안에 있어야함
        within_range_criteria = [
            nx > (len(read_map) - 1),
            nx < 0,
            ny > (len(read_map[len(read_map) - 1]) - 1),
            ny < 0,
        ]

        if any(within_range_criteria):  # 하나라도 true면 범위 밖임
            continue
        
        if overlap_prevenstion[nx][ny] == 1:    # 이미 경유했거나 경유한 지점 근처면 Pass
            continue

        if read_map[nx][ny] == '4':
            min = 10000
            min_x = -1
            min_y = -1
            for near in d1:
                nnx = nx + near[0]
                nny = ny + near[1]

                if read_map[nnx][nny] == '0':
                    if min > abs(nnx - start[0]) + abs(nny - start[1]):
                        min = abs(nnx - start[0]) + abs(nny - start[1])
                        min_x = nnx
                        min_y = nny
                    
            station = (min_x, min_y)
            overlap_prevenstion[min_x][min_y]
            for near_blocking in d1:
                nbx = min_x + near_blocking[0]
                nby = min_y + near_blocking[1]
                overlap_prevenstion[nbx][nby] = 1
            for near_blocking in d2:
                nbx = min_x + near_blocking[0]
                nby = min_y + near_blocking[1]
                overlap_prevenstion[nbx][nby] = 1

            if min_x != -1:
                return 1

    return 0

def main():
    global path
    rospy.init_node('Astar')
    rospy.Subscriber('/odom2', Pose2D, pose_callback)

    picture_map = cv2.imread("/home/kwon/catkin_ws/src/imsogyeong_action/map.jpg")
    f = open("/home/kwon/catkin_ws/src/imsogyeong_action/map.txt", "r")
    #row, col = map(int, f.readline().split())
    row = 10
    col = 10

    read_map = [[0 for j in range(col)] for i in range(row)]
    maze = [[0 for j in range(col)] for i in range(row)]

    for i in range(row):
        read_map[i] = f.readline().split()

    for r in range(row):
        for c in range(col):
            if read_map[r][c] ==  '1':
                maze[r][c] = 1      
            elif read_map[r][c] == '2':
                start = (r, c)
            elif read_map[r][c] == '3':
                end = (r, c)
            elif read_map[r][c] == '4':
                maze[r][c] = 1  
            elif read_map[r][c] == '5':
                maze[r][c] = 1   

    #if (target(overlap_prevenstion, read_map, start) == 1):        # 바운더리 내에 선호하는 대상이 있다면
    #    path_1 = aStar(maze, start, station)
    #    start = station
    #    path_2 = aStar(maze, start, end)
    #    path = path_1 + path_2
    #else:    
    path = aStar(maze, start, end)

    for i in path:
        center_x = i[1] * block_scale + block_scale // 2   # center_x를 c로 구하는 이유: x좌표는 픽셀 size이기 때문
        center_y = i[0] * block_scale + block_scale // 2
        cv2.circle(picture_map, (center_x, center_y), block_scale // 4, (0, 0, 0), 1)

    cv2.imwrite("/home/kwon/catkin_ws/src/imsogyeong_action/a.jpg", picture_map)
    rospy.spin()

if __name__ == '__main__':
    main()