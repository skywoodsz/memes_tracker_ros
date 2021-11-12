import numpy as np
from scipy.optimize import fsolve, root
from math import sin, cos


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
    temp = 2 * np.cos(beta) * np.cos(alpha) * np.sin(alpha) + 2 * np.cos(alpha) * np.cos(alpha) - 1
    x0 = 2 * np.cos(alpha) * np.sin(alpha) * d1 / temp
    y0 = 2 * np.cos(beta) * np.cos(alpha) * np.sin(alpha) * d1 / temp
    z0 = 0
    return np.array([x0, y0, z0])


def plane_cross(R2, start, d2):
    temp = -(d2 + start[0]) / R2[0]
    x0 = -d2
    y0 = start[1] + R2[1] * temp
    z0 = R2[2] * temp
    return np.array([x0, y0, z0])


def f(x):
    alpha = float(x[0])
    beta = float(x[1])
    # d1 = 0.1
    # d2 = 0.5
    # temp = 2 * cos(beta) * cos(alpha) * sin(alpha) + 2 * cos(alpha) * cos(alpha) - 1
    # x0 = 2 * cos(alpha) * sin(alpha) * d1 / temp
    # y0 = 2 * cos(beta) * cos(alpha) * sin(alpha) * d1 / temp
    # z0 = 0
    # result = [
    #
    # ]
    R2 = coordinates_calculate(alpha, beta)
    start = motor_cross(alpha, beta, 0.064)
    return plane_cross(R2, start, 0.507)[1:] - np.array([0, -0.022])


def main():
    result = fsolve(f, [0, 0])
    print(result[0] * 180 / np.pi, result[1] * 180 / np.pi)
    # result = root(f, [0, 0], method='krylov')
    # print(result[0] * 130 * 180 / np.pi, result[1] * 0.006 * 180 / np.pi + Y)
    # print('*' * 10)
    # print(motor_cross(np.pi / 6, np.pi / 6, 10))
    # for i in range(100):
    #     alpha = i * np.pi / 100
    #     beta = i * np.pi / 100
    #     R2 = coordinates_calculate(alpha, beta)
    #     start = motor_cross(alpha, beta, 0.1)
    #     print(plane_cross(R2, start, 0.5))
    # print('*' * 10)
    #
    # for i in range(30):
    #     beta = i * np.pi / 100
    #     coordinates_calculate(0, beta)


if __name__ == '__main__':
    main()
