import numpy as np

# 定义坐标范围和障碍物数量
x_range = [-1, 1]
y_range = [-1, 1]
z_range = [-1, 1]
num_obstacles = 10

# 指定点位坐标
target_points = [(0.5, 0.5, 0.5), (0.3, 0.7, 0.4), (0.8, 0.2, 0.1)]

# 生成随机点位
obstacle_positions = []
while len(obstacle_positions) < num_obstacles:
    x = np.random.uniform(*x_range)
    y = np.random.uniform(*y_range)
    z = np.random.uniform(*z_range)
    # 计算点位到每个指定点位的距离
    distances = [np.sqrt((x - p[0])**2 + (y - p[1])**2 + (z - p[2])**2) for p in target_points]
    # 确认生成的点位到每个指定点位的距离都足够远
    if all(distance > 0.05 for distance in distances):
        obstacle_positions.append((x, y, z))

print(obstacle_positions)