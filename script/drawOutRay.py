import matplotlib.pyplot as plt
import numpy as np
import re
import random

def parse_vector(vector_str):
    # 使用正则表达式提取向量中的数字
    match = re.findall(r'-?\d*\.?\d+(?:[eE][+-]?\d+)?', vector_str)
    print(vector_str)
    print(match)
    if match:
        # 将字符串中的数字转换为浮点数
        vector = list(map(float, match))
        return vector
    else:
        raise ValueError("输入格式不正确")
    
# 定义向量对 (x, y)
vector_pairs = []

with open('/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/build/testlog', 'r') as file:
    lines = file.readlines()

# 每两行作为一个处理单位
for i in range(0, len(lines), 2):
    addr = lines[i].strip()
    Dir = lines[i+1].strip() if i+1 < len(lines) else ''
    if random.random() < 0.3:
        vector_pairs.append((parse_vector(addr),parse_vector(Dir)))
    print((parse_vector(addr),parse_vector(Dir)))

# 提取XY平面的投影
projections = [(x[1], x[2], y[1], y[2]) for x, y in vector_pairs]

# 转换为numpy数组以便于绘图
projections = np.array(projections)
print(projections)

# 创建图形
plt.figure()
plt.quiver(projections[:, 0], projections[:, 1], projections[:, 2], projections[:, 3], angles='xy', scale_units='xy', scale=1, width=0.002)
plt.xlim(-1, 1)
plt.ylim(-0.5, 0.5)
plt.xlabel('Y')
plt.ylabel('Z')
plt.title('Vectors projected onto YZ plane')
plt.grid()
plt.savefig('vector_projections_3.png', dpi=300)
