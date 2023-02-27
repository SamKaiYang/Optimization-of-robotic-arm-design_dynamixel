import trimesh
import numpy as np



def stl_trimesh_scaling(axis, scale):
    # 加载包含三个组件的STL
    if axis == 2:
        mesh = trimesh.load('original_single_arm_axis_2.STL')
        mesh.show()
        # 获取需要缩放的组件和其他组件
        scaled_mesh = mesh.split()[1]  # 第二个组件

        # 0 2 7 8
        # base_meshes = mesh.split()[:1]
        base_meshes = trimesh.util.concatenate(mesh.split()[3]+mesh.split()[4]+mesh.split()[5]+mesh.split()[6])
        other_meshes = trimesh.util.concatenate(mesh.split()[0]+mesh.split()[2]+mesh.split()[7]+mesh.split()[8])
        
        # 将 STL 模型沿 X 轴方向偏移 1 厘米
        x_cm = scale - 12 # 沿 X 轴方向的偏移量（单位：厘米）
        x_m = x_cm / 100.0  # 将单位从厘米转换为米
        other_meshes.apply_translation([x_m, 0, 0])

        # 在X方向上缩放
        scale_factor = scale/12

        scaled_mesh.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度
        
        offset_x = -x_cm/2*0.01 # 将单位从厘米转换为米

        scaled_mesh.apply_translation([offset_x, 0, 0])
        # 将缩放后的组件和其他组件重新组合成一个网格对象
        new_mesh = trimesh.util.concatenate(base_meshes +scaled_mesh + other_meshes)

        # 保存为新的STL文件
        new_mesh.export('merged_single_arm_axis_2.STL')
        new_mesh.show()
    elif axis == 3:
        mesh = trimesh.load('original_single_arm_axis_3.STL')
        mesh.show()
        # 获取需要缩放的组件和其他组件
        scaled_mesh = mesh.split()[3]  # 第四个组件

        # 0 2 7 8
        # base_meshes = mesh.split()[:1]
        base_meshes = trimesh.util.concatenate(mesh.split()[0]+mesh.split()[1]+mesh.split()[2]+mesh.split()[5])
        other_meshes = trimesh.util.concatenate(mesh.split()[4]+mesh.split()[6])
        
        # 将 STL 模型沿 X 轴方向偏移 1 厘米
        x_cm = scale - 12 # 沿 X 轴方向的偏移量（单位：厘米）
        x_m = x_cm / 100.0  # 将单位从厘米转换为米
        other_meshes.apply_translation([x_m, 0, 0])

        # 在X方向上缩放
        scale_factor = scale/12

        scaled_mesh.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度
        
        offset_x = -x_cm/2*0.01 # 将单位从厘米转换为米

        scaled_mesh.apply_translation([offset_x, 0, 0])
        # 将缩放后的组件和其他组件重新组合成一个网格对象
        new_mesh = trimesh.util.concatenate(base_meshes +scaled_mesh + other_meshes)

        # 保存为新的STL文件
        new_mesh.export('merged_single_arm_axis_3.STL')
        new_mesh.show()



if __name__=="__main__":
    stl_trimesh_scaling(3,18)