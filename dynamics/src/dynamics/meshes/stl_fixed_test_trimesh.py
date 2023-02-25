import trimesh
import numpy as np
# def compute_face_normals(mesh):
#     normals = np.zeros(mesh.faces.shape)
#     for i, face in enumerate(mesh.faces):
#         v0, v1, v2 = mesh.vertices[face]
#         normal = np.cross(v1-v0, v2-v0)
#         norm = np.linalg.norm(normal)
#         if norm == 0:
#             continue
#         normal /= norm
#         normals[i] = normal
#     return normals
# 載入STL檔
'''
mesh = trimesh.load('original.STL')

# 切割機構為單獨的構件
components = mesh.split(only_watertight=False)
print(len(components))
# 選擇要更改長度的機構
component_to_modify = components[1] # 更新中間組件的位置

# 獲取機構的邊界框
bbox = component_to_modify.bounding_box.bounds

# 計算機構的長度
length = bbox[1][0] - bbox[0][0]

# 將機構的長度增加20%
new_length = length * 2.2

# 縮放機構，以匹配新的長度
scale_factor = new_length / length
# component_to_modify.apply_scale(scale_factor)
# 應用縮放因素
component_to_modify = component_to_modify  # 如果您只想更改STL檔中的某個部分，則將其設為該部分
component_to_modify.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度

# 將修改後的機構保存為新的STL檔
trimesh.exchange.export.export_mesh(component_to_modify, 'modified.STL', file_type='stl')
# # 存回 STL 模型
# mesh.export('modified.STL')
remesh = trimesh.load('modified.STL')
remesh.show()

'''

'''
import trimesh
import numpy as np

# Load the original STL file
mesh = trimesh.load('original.STL')
mesh.show()
# Find the component to scale (assume it's the first part)
component_mesh = mesh.split()[1]

# Scale the component in the X direction
scale_factor = 1
scale_matrix = np.eye(4)
scale_matrix[0, 0] = scale_factor
scaled_component_mesh = component_mesh.apply_transform(scale_matrix)
scaled_component_mesh.export('scaled_component.STL')
scaled_component_mesh.show()

# 加载原始STL
original_mesh = trimesh.load('original.STL')

# 加载需要缩放的组件
scaled_mesh = trimesh.load('scaled_component.STL')

# 在X方向上缩放
# scaled_mesh.scale([2.0, 1.0, 1.0])
scale_factor = 2
scaled_mesh.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度
# 合并原始STL和缩放后的组件
merged_mesh = trimesh.util.concatenate([original_mesh, scaled_mesh])

# 保存为新的STL文件
merged_mesh.export('merged.STL')
merged_mesh.show()
'''

import trimesh

# 加载包含三个组件的STL
mesh = trimesh.load('original.STL')

# 获取需要缩放的组件和其他组件
scaled_mesh = mesh.split()[1]  # 第二个组件
scaled_mesh.show()
# other_meshes = mesh.split()[:1] + mesh.split()[2:]  # 其他组件
# 将其他组件和缩放后的组件合并为一个网格对象
other_meshes = trimesh.util.concatenate(mesh.split()[:1] + mesh.split()[2:])
other_meshes.show()
# 将其他组件和需要缩放的组件放置在它们在全局坐标系中的正确位置
# for mesh in other_meshes:
#     mesh.apply_transform(mesh.transform)
# scaled_mesh.apply_transform(scaled_mesh.transform)

# 记录需要缩放的组件在其局部坐标系中的位置和方向
# scaled_transform = scaled_mesh.transform

# # 将需要缩放的组件从父网格对象中删除并应用到其父网格对象中
# parent_mesh = scaled_mesh.parent
# parent_mesh.apply_transform(scaled_transform)
# scaled_mesh.apply_transform(-scaled_transform)
# parent_mesh.remove_geometry(scaled_mesh)
# scaled_mesh.apply_transform(scaled_transform)

# 在X方向上缩放
# scaled_mesh.scale([2.0, 1.0, 1.0])# 在X方向上缩放
# scaled_mesh.scale([2.0, 1.0, 1.0])
scale_factor = 2
scaled_mesh.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度

# # 将缩放后的组件重新添加到其父网格对象中
# parent_mesh.add_geometry(scaled_mesh)

# 将缩放后的组件和其他组件重新组合成一个网格对象
new_mesh = trimesh.util.concatenate(scaled_mesh + other_meshes)

# 保存为新的STL文件
new_mesh.export('merged.STL')
new_mesh.show()