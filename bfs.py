import time
from collections import deque
import math

from DroneBlocksTelloSimulator.DroneBlocksSimulatorContextManager import DroneBlocksSimulatorContextManager, SimulatedDrone


class BFS:
    def __init__(self, start, goal, buildings):
        self.start = start
        self.goal = goal
        self.buildings = buildings
        self.visited = set()  # 방문한 노드
        self.came_from = {}  # 경로 복원용
        self.queue = deque([start])  # 탐색할 위치 큐

    def is_collision(self, x, y):
        # 건물의 크기를 기준으로 충돌 여부 판단
        for (bx, by, size) in self.buildings:
            # 건물은 (bx, by)를 중심으로 size 크기를 가지므로, 불가능 영역 계산
            half_size = size / 2
            if bx - half_size <= x <= bx + half_size and by - half_size <= y <= by + half_size:
                return True
        return False

    def neighbors(self, current):
        # 8방향 이동 + 추가적인 대각선(45도 이외의 대각선) 포함
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),   # 상, 하, 좌, 우
                      (-1, -1), (1, -1), (-1, 1), (1, 1),  # 45도 대각선
                      (-2, -1), (2, -1), (-2, 1), (2, 1),  # 비정상적인 대각선 방향들
                      (-1, -2), (1, -2), (-1, 2), (1, 2)]  # 비정상적인 대각선 방향들
        neighbors = []
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            if not self.is_collision(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    def reconstruct_path(self):
        # 경로 복원
        path = []
        current = self.goal
        while current != self.start:
            path.append(current)
            current = self.came_from[current]
        path.append(self.start)
        path.reverse()
        return path

    def find_path(self):
        # BFS 탐색
        while self.queue:
            current = self.queue.popleft()

            if current == self.goal:
                # 목표에 도달하면 경로 복원
                path = self.reconstruct_path()
                return path

            for neighbor in self.neighbors(current):
                if neighbor not in self.visited:
                    self.visited.add(neighbor)
                    self.came_from[neighbor] = current
                    self.queue.append(neighbor)

        return None  # 경로가 없으면 None 반환


# https://coding-sim.droneblocks.io/
sim_key = input("SIM 연결 키? ")

start_pos = (0, 0, 10)      # X, Y, Altitude [cm]
goal_pos = (200, 200, 75)   # X, Y, Z [cm]]
buildings = [
    (100, 100, 40)
]
lift_distance = 10

bfs = BFS(start_pos[0:2], goal_pos[0:2], buildings)
paths = bfs.find_path()

raise ValueError

with DroneBlocksSimulatorContextManager(simulator_key=sim_key) as drone:
    drone: SimulatedDrone = drone   # Typing

    # 시작 시간
    start = time.time()

    # 드론 상승
    drone.takeoff()

    drone.fly_to_xyz(0, 0, goal_pos[2]+lift_distance, 'cm')

    previous_path = start_pos[0:2]
    for path in paths:
        drone.fly_to_xyz(path[0] - previous_path[0], path[1] - previous_path[1], 0, 'cm')
        previous_path = path

    drone.fly_to_xyz(0, 0, lift_distance*-1, 'cm')

    # 종료 시간
    end = time.time()

    # 경로 이동 시간
    print(f"경로 이동 시간: {round(end - start, 3)} seconds")

    # 착륙
    # 다만 경로 이동 시간 측정이므로 미필요
    # drone.land()
