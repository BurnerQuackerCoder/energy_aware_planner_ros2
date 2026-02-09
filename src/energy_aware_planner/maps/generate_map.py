import numpy as np

# Map dimensions 200x200 (10m x 10m at 0.05 res)
map_size = 200
data = np.full((map_size, map_size), 255, dtype=np.uint8)

# Add two horizontal walls to create 3 lanes
# Wall 1
data[65:70, 0:150] = 0   # Wall from left to 3/4 right
# Wall 2
data[130:135, 50:200] = 0 # Wall from 1/4 right to far right

# This creates:
# Lane 1 (Top): Gap is on the left
# Lane 2 (Middle): Gaps on both ends
# Lane 3 (Bottom): Gap is on the right

# Save as PGM (Portable Gray Map)
with open('complex_maze.pgm', 'wb') as f:
    f.write(f'P5\n{map_size} {map_size}\n255\n'.encode())
    f.write(data.tobytes())

print("Created complex_maze.pgm")

# Create the YAML
yaml_content = """image: complex_maze.pgm
resolution: 0.050000
origin: [-5.000000, -5.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open('complex_maze.yaml', 'w') as f:
    f.write(yaml_content)

print("Created complex_maze.yaml")