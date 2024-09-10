# import struct
# import numpy as np
# import open3d as o3d
# import matplotlib.pyplot as plt

# # 示例 PointCloud2 消息
# msg = {
#     'header': {
#         'stamp': {
#             'sec': 845,
#             'nanosec': 600000000
#         },
#         'frame_id': 'bluerov2/sonar'
#     },
#     'height': 1,
#     'width': 10,
#     'fields': [
#         {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
#         {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
#         {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
#         {'name': 'intensity', 'offset': 12, 'datatype': 7, 'count': 1}
#     ],
#     'is_bigendian': False,
#     'point_step': 16,
#     'row_step': 160,
#     'data': [
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 0, 63,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 128, 63,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 64, 63,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 192, 63,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 0, 64,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 64, 64,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 128, 64,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 192, 64,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 0, 65,
#         0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 64, 65,
#     ],
#     'is_dense': True
# }

# # 定义解析字段的函数
# def unpack_field(data, offset, datatype, count, is_bigendian):
#     fmt = {
#         1: 'B',   # uint8
#         2: 'H',   # uint16
#         3: 'I',   # uint32
#         4: 'b',   # int8
#         5: 'h',   # int16
#         6: 'i',   # int32
#         7: 'f',   # float32
#         8: 'd'    # float64
#     }[datatype]
    
#     endian = '>' if is_bigendian else '<'
#     unpack_fmt = endian + fmt * count
#     return struct.unpack(unpack_fmt, bytes(data[offset:offset + struct.calcsize(unpack_fmt)]))

# # 解析点云数据
# points = []
# for i in range(0, len(msg['data']), msg['point_step']):
#     point_data = msg['data'][i:i + msg['point_step']]
#     x = unpack_field(point_data, msg['fields'][0]['offset'], msg['fields'][0]['datatype'], msg['fields'][0]['count'], msg['is_bigendian'])[0]
#     y = unpack_field(point_data, msg['fields'][1]['offset'], msg['fields'][1]['datatype'], msg['fields'][1]['count'], msg['is_bigendian'])[0]
#     z = unpack_field(point_data, msg['fields'][2]['offset'], msg['fields'][2]['datatype'], msg['fields'][2]['count'], msg['is_bigendian'])[0]
#     intensity = unpack_field(point_data, msg['fields'][3]['offset'], msg['fields'][3]['datatype'], msg['fields'][3]['count'], msg['is_bigendian'])[0]
#     points.append([x, y, z, intensity])

# # 转换为 NumPy 数组
# points = np.array(points)

# # 计算每个点到原点的距离
# distances = np.sqrt(np.sum(points[:, :3]**2, axis=1))

# # 打印距离的统计信息
# print(f"Distance statistics:\n"
#       f"Min distance: {np.min(distances)}\n"
#       f"Max distance: {np.max(distances)}\n"
#       f"Mean distance: {np.mean(distances)}\n"
#       f"Median distance: {np.median(distances)}")

# # 将距离信息应用于颜色映射
# min_distance = np.min(distances)
# max_distance = np.max(distances)
# normalized_distances = (distances - min_distance) / (max_distance - min_distance + 1e-5)
# colors = plt.cm.viridis(normalized_distances)[:, :3]

# # 使用 Open3D 可视化点云数据
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points[:, :3])
# pcd.colors = o3d.utility.Vector3dVector(colors)
# o3d.visualization.draw_geometries([pcd])

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudProcessor(Node):

    def __init__(self):
        super().__init__('point_cloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/bluerov2/cloud',
            self.point_cloud_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/processed_point_cloud', 10)
        self.get_logger().info('PointCloudProcessor has been started.')

    def point_cloud_callback(self, msg):
        cloud_points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "intensity", "ring")))
        self.get_logger().info(f'Number of points received: {len(cloud_points)}')

        # 取前3590个点
        if len(cloud_points) >= 3590:
            cloud_points = cloud_points[:3590]
            structured_array = np.array(cloud_points, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32), ('ring', np.uint32)])
            self.get_logger().info(f'Shape of structured array: {structured_array.shape}')

            try:
                points = np.column_stack([structured_array['x'], structured_array['y'], structured_array['z'], structured_array['intensity'], structured_array['ring']])
                obstacles = self.analyze_point_cloud(points)
                self.publish_obstacle_cloud(obstacles)
            except ValueError as e:
                self.get_logger().error(f'Error reshaping points array: {e}')
        else:
            self.get_logger().error('Not enough points received.')

    def analyze_point_cloud(self, points):
        # 假设 z > 0.2 表示障碍物
        obstacle_threshold = 0.2
        try:
            obstacles = points[points[:, 2] > obstacle_threshold]
            obstacle_positions = obstacles[:, :3]  # 提取 x, y, z 位置
            self.get_logger().info(f'Obstacle positions: {obstacle_positions}')
            return obstacles
        except IndexError as e:
            self.get_logger().error(f'Error indexing points array: {e}')
            return np.array([])

    def publish_obstacle_cloud(self, obstacles):
        if obstacles.size == 0:
            self.get_logger().info('No obstacles found, not publishing.')
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT32, count=1)
        ]
        
        # 使用结构化数组创建点云消息
        obstacle_cloud = pc2.create_cloud(header, fields, obstacles.view(np.recarray))
        self.publisher.publish(obstacle_cloud)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_processor = PointCloudProcessor()
    rclpy.spin(point_cloud_processor)
    point_cloud_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()













