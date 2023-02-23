import trimesh

# 載入STL檔
mesh = trimesh.load('single_arm_v12_2_12.0.STL')

# 切割機構為單獨的構件
components = mesh.split(only_watertight=False)
print(len(components))
# 選擇要更改長度的機構
component_to_modify = components[1]

# 獲取機構的邊界框
bbox = component_to_modify.bounding_box.bounds

# 計算機構的長度
length = bbox[1][0] - bbox[0][0]

# 將機構的長度增加20%
new_length = length * 50.2

# 縮放機構，以匹配新的長度
scale_factor = new_length / length
# component_to_modify.apply_scale(scale_factor)
# 應用縮放因素
component_to_modify = component_to_modify  # 如果您只想更改STL檔中的某個部分，則將其設為該部分
component_to_modify.apply_scale([scale_factor, 1, 1])  # 只更改x軸的長度

# 將修改後的機構保存為新的STL檔
trimesh.exchange.export.export_mesh(component_to_modify, 'modified_filename.STL', file_type='stl')


import trimesh

# 載入原始STL檔
original_mesh = trimesh.load('single_arm_v12_2_12.0.STL')

# 載入修改後的STL檔
modified_mesh = trimesh.load('modified_filename.STL')

# 取得要合併回原始STL檔的部分
original_part = original_mesh.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])  # 使用適當的平面參數選擇您需要合併的部分
# modified_part = modified_mesh.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
modified_part = modified_mesh
# 將修改後的部分與原始部分對齊
modified_part.apply_transform(trimesh.transformations.superimposition_matrix(modified_part.vertices, original_part.vertices))

# 合併部分
merged_part = original_part.union(modified_part)

# 將合併後的部分與原始STL檔的其餘部分合併
final_mesh = original_mesh.difference(original_part).union(merged_part)

# 保存新的STL檔
trimesh.exchange.export.export_mesh(final_mesh, 'final_filename.stl', file_type='stl')