import time

from DroneBlocksTelloSimulator.DroneBlocksSimulatorContextManager import DroneBlocksSimulatorContextManager, SimulatedDrone

# https://coding-sim.droneblocks.io/
sim_key = input("SIM 연결 키? ")

start_pos = (0, 0, 10)      # X, Y, Altitude [cm]
goal_pos = (200, 200, 75)   # X, Y, Z [cm]
lift_distance = 10

distance = 40
with DroneBlocksSimulatorContextManager(simulator_key=sim_key) as drone:
    drone: SimulatedDrone = drone   # Typing

    # 시작 시간
    start = time.time()

    # 드론 상승
    drone.takeoff()

    drone.fly_to_xyz(goal_pos[0], goal_pos[1], goal_pos[2]+lift_distance, 'cm')
    drone.fly_to_xyz(0, 0, lift_distance*-1, 'cm')

    # 종료 시간
    end = time.time()

    # 경로 이동 시간
    print(f"경로 이동 시간: {round(end - start, 3)} seconds")

    # 착륙
    # 다만 경로 이동 시간 측정이므로 미필요
    # drone.land()
