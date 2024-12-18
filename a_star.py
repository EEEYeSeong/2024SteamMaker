import time
import heapq
import math

from DroneBlocksTelloSimulator.DroneBlocksSimulatorContextManager import DroneBlocksSimulatorContextManager, SimulatedDrone


class AStar:
    def __init__(self, start, goal, buildings):
        self.start = start
        self.goal = goal
        self.buildings = buildings
        self.open_list = []
        self.closed_list = set()
        self.came_from = {}
        self.g_score = {start: 0}
        self.f_score = {start: self.heuristic(start)}
        heapq.heappush(self.open_list, (self.f_score[start], start))

    def heuristic(self, current):
        # 유클리드 거리 (직선거리)를 휴리스틱으로 사용
        return math.dist(current, self.goal)

    def is_collision(self, x, y):
        # 건물의 크기를 기준으로 충돌 여부 판단
        for (bx, by, size) in self.buildings:
            # 건물은 (bx, by)를 중심으로 size 크기를 가지므로, 불가능 영역 계산
            half_size = size / 2
            if bx - half_size <= x <= bx + half_size and by - half_size <= y <= by + half_size:
                return True
        return False

    def neighbors(self, current):
        # 8방향으로 이동 (상, 하, 좌, 우, 대각선 포함)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),   # 상, 하, 좌, 우
                      (-1, -1), (1, -1), (-1, 1), (1, 1)]  # 대각선
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

    def optimize_path(self, path):
        # 최적화: 경로 중간에 불필요한 점을 제거하여 직선 경로로 만듦
        optimized_path = [path[0]]
        for i in range(2, len(path)):
            prev, curr, next_point = path[i - 2], path[i - 1], path[i]
            # 직선 여부 확인 (세 점이 일직선 상에 있으면 현재 점을 건너뛰기)
            if (next_point[1] - prev[1]) * (curr[0] - prev[0]) == (curr[1] - prev[1]) * (next_point[0] - prev[0]):
                continue
            optimized_path.append(curr)
        optimized_path.append(path[-1])
        return optimized_path

    def find_path(self):
        while self.open_list:
            _, current = heapq.heappop(self.open_list)

            if current == self.goal:
                # 목표에 도달하면 경로 복원
                path = self.reconstruct_path()
                optimized_path = self.optimize_path(path)
                return optimized_path

            self.closed_list.add(current)

            for neighbor in self.neighbors(current):
                if neighbor in self.closed_list:
                    continue

                tentative_g_score = self.g_score[current] + 1  # 이동 비용 (상하좌우, 대각선은 1)

                if neighbor not in self.g_score or tentative_g_score < self.g_score[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = tentative_g_score
                    self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor)
                    heapq.heappush(self.open_list, (self.f_score[neighbor], neighbor))

        return None  # 경로가 없으면 None 반환


# https://coding-sim.droneblocks.io/
sim_key = input("SIM 연결 키? ")

start_pos = (0, 0, 10)      # X, Y, Altitude [cm]
goal_pos = (200, 200, 75)   # X, Y, Z [cm]]
buildings = [
    (100, 100, 40)
]
lift_distance = 10


start = time.time()
astar = AStar(start_pos[0:2], goal_pos[0:2], buildings)
paths = astar.find_path()

print(time.time() - start)

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
