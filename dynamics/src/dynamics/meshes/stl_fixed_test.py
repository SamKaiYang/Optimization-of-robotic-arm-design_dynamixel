import numpy as np
from stl import mesh

# # 讀取 STL 檔案
stl_file = 'random_2_20.0.STL'
# mesh = mesh.Mesh.from_file(stl_file)

# # 將每個物件縮放倍數
# scale = 10.0  # 縮放倍數
# for i, _ in enumerate(mesh.vectors):
#     mesh.vectors[i] *= scale

# # 將結果保存回 STL 檔案
# output_file = 'random_2_20.0_scaled.STL'
# mesh.save(output_file)


# 指定需要更改的方向
direction = 'y' # 'x' or 'y' or 'z'
scale = 2.0 # 放大/缩小倍数

# 读取STL文件
mesh_data = mesh.Mesh.from_file(stl_file)

# 获取所有向量
vectors = mesh_data.vectors

# 根据指定方向调整向量的尺寸
if direction == 'x':
    vectors[:, :, 0] *= scale
elif direction == 'y':
    vectors[:, :, 1] *= scale
elif direction == 'z':
    vectors[:, :, 2] *= scale

# 将结果保存回STL文件
output_file = 'random_2_20.0_scaled.STL'
mesh_data.save(output_file)