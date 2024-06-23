import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

def plot_projection(points):
    # 提取 x, y, z 坐标
    x = [point[1] for point in points]
    y = [point[2] for point in points]
    
    # 创建一个新的图形
    plt.figure()

    # 绘制投影到 xy 平面的点
    plt.scatter(x, y, c='b', marker='^', label='Projection on XY Plane')

    # 设置标签
    plt.xlabel('X Label')
    plt.ylabel('Y Label')
    plt.gca().set_aspect('equal', adjustable='box')
    # 添加图例
    plt.legend()
    # 显示图形
    plt.savefig('projection.png', dpi=300)

# 示例数据
points = []

def parse_vector(vector_str):
    # 使用正则表达式提取向量中的数字
    match = re.findall(r'-?\d*\.?\d+', vector_str)
    if match:
        # 将字符串中的数字转换为浮点数
        vector = list(map(float, match))
        return vector
    else:
        raise ValueError("输入格式不正确")
    
# 调用函数绘制图形

with open('/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/build/testlog', 'r') as file:
    lines = file.readlines()

# 每两行作为一个处理单位
for i in range(0, len(lines), 1):
    vec = parse_vector(lines[i].strip())
    print(vec)
    if abs(vec[1]) > 1 or abs(vec[2]) > 1 or abs(vec[3]) > 1:
        continue
    else:
        points.append(vec)

plot_projection(points)