import numpy as np

map_size = 200
data = np.full((map_size, map_size), 255, dtype=np.uint8)

# 1. Industrial Border (Walls)
data[0:5, :] = 0; data[-5:, :] = 0; data[:, 0:5] = 0; data[:, -5:] = 0

# 2. Structural Shelving (Pillars)
# This creates a "corridor" effect
for y in [40, 160]:
    for x in range(20, 180, 40):
        data[y-4:y+4, x-10:x+10] = 0

# 3. The Central Gate (The Decision Point)
# Two long shelves that force the robot through specific openings
data[85:115, 70:75] = 0  # Left gate post
data[85:115, 125:130] = 0 # Right gate post

with open('complex_maze.pgm', 'wb') as f:
    f.write(f'P5\n{map_size} {map_size}\n255\n'.encode())
    f.write(data.tobytes())