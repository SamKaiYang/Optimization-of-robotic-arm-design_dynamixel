import trimesh
import numpy as np

# 加载包含三个组件的STL
mesh = trimesh.load('original_single_arm_v22_axis_2.STL')
# mesh = trimesh.load('original_single_arm_axis_2.STL')
mesh.show()
# # 获取需要缩放的组件和其他组件
# axis2 共有 10個組件 其中需要縮放3
    # base_meshes 1 5 8 9 10
    # other meshes 2 4 6 7

print("axis_2 mesh 組件量:",len(mesh.split()))
for i in range(len(mesh.split())):
    print(i)
    scaled_mesh = mesh.split()[i]
    scaled_mesh.show()

# 加载包含三个组件的STL
mesh = trimesh.load('original_single_arm_v22_axis_3.STL')
mesh.show()
# # 获取需要缩放的组件和其他组件
# axis3 共有 8個組件 其中需要縮放5
        # base_meshes 1 4 6 7 8
        # other meshes 2 3
print("axis_3 mesh 組件量:",len(mesh.split()))
for i in range(len(mesh.split())):
    scaled_mesh = mesh.split()[i]
    scaled_mesh.show()


# # 切割機構為單獨的構件
# components = mesh.split()
# print(len(components))
# components = mesh.split()[1]
# print(components)
# # 获取第一个组件的顶点位置
# index_component = 0  # 需要获取位置的组件的索引
# vertices = mesh.vertices[mesh.geometry[index_component].vertices]

# # 计算组件的中心位置
# center = vertices.mean(axis=0)

# print(center)  # 输出组件的中心位置