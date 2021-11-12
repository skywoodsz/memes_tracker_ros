import numpy as np


def coordinates_calculate(alpha, beta):
    # # MEMS偏转角
    # # alpha = np.pi / 6
    # alpha = 0
    # # 电机偏转角
    # # beta = np.pi / 4
    # beta = 0
    # MEMS法向量
    N1 = np.array([np.sin(alpha), -np.cos(alpha), 0])
    # 入射方向
    I1 = np.array([0, 1, 0])
    W1 = - 2 * I1.dot(N1) * N1
    # MEMS出射方向，也即电机入射方向
    R1 = I1 + W1
    # print(R1)
    # 电机法向量
    N2 = np.array([-np.cos(beta) * np.sqrt(2) / 2, np.sqrt(2) / 2, np.sin(beta) * np.sqrt(2) / 2])
    W2 = - 2 * R1.dot(N2) * N2
    # 电机出射方向
    R2 = R1 + W2
    # print(R2)
    return R2


def motor_cross(alpha, beta, d1):
    # 求解MEMS出射直线与电机镜面的交点，利用直线与平面相交求得。d1为MEMS和电机镜面中心点之间的距离
    temp = 2 * np.cos(beta) * np.cos(alpha) * np.sin(alpha) + 2 * np.cos(alpha) * np.cos(alpha) - 1
    x0 = 2 * np.cos(alpha) * np.sin(alpha) * d1 / temp
    y0 = 2 * np.cos(beta) * np.cos(alpha) * np.sin(alpha) * d1 / temp
    z0 = 0
    return np.array([x0, y0, z0])


def plane_cross(R2, start, d2):
    # 求解电机出射直线与接收端平面交点。d2为电机镜面中心与接收端距离
    temp = -(d2 + start[0]) / R2[0]
    x0 = -d2
    y0 = start[1] + R2[1] * temp
    z0 = R2[2] * temp
    return np.array([x0, y0, z0])


def main():
    # print(motor_cross(np.pi / 6, np.pi / 6, 10))
    # for i in range(100):
    #     alpha = i * np.pi / 100
    #     beta = i * np.pi / 100
    #     R2 = coordinates_calculate(alpha, beta)
    #     start = motor_cross(alpha, beta, 0.1)
    #     print(plane_cross(R2, start, 0.5))
    # print('*' * 10)
    alpha = 0.0332931
    beta = 0.1288319
    R2 = coordinates_calculate(alpha, beta)
    start = motor_cross(alpha, beta, 0.1)
    print(plane_cross(R2, start, 0.5))
#
# for i in range(30):
#     beta = i * np.pi / 100
#     coordinates_calculate(0, beta)


if __name__ == '__main__':
    main()
