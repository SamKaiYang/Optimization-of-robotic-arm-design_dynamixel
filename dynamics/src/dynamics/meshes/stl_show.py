import trimesh
import numpy as np

# 加载包含三个组件的STL
mesh = trimesh.load('original.STL')
# mesh.show()
# # 获取需要缩放的组件和其他组件
# # 0 2 7 8
# # scaled_mesh = mesh.split()  # 第二个组件
# # scaled_mesh.show()

# # 切割機構為單獨的構件
# components = mesh.split()
# print(len(components))
components = mesh.split()[1]
print(components)
# # 获取第一个组件的顶点位置
# index_component = 0  # 需要获取位置的组件的索引
# vertices = mesh.vertices[mesh.geometry[index_component].vertices]

# # 计算组件的中心位置
# center = vertices.mean(axis=0)

# print(center)  # 输出组件的中心位置