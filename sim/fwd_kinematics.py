import matplotlib.pyplot as plt
from math import sin, cos

M_PI = 3.14159265
TRACK_WIDTH = 0.0633

def fwd(x, y, yaw, vl , vr, dt_s):
    w = (vr - vl) / TRACK_WIDTH

    x_ = x
    y_ = y
    yaw_ = yaw
    if (vl == vr):
        # going perfectly straight is a special condition
        x_ += (dt_s * (vl + vr) / 2) * cos(yaw)
        y_ += -(dt_s * (vl + vr) / 2) * sin(yaw)
    else:
        r = TRACK_WIDTH / 2 * (vl + vr) / (vr - vl)
        x_ = -cos(w * dt_s) * r * sin(yaw) + sin(w * dt_s) * r * cos(yaw) + x + r * sin(yaw)
        y_ = -sin(w * dt_s) * r * sin(yaw) - cos(w * dt_s) * r * cos(yaw) + y + r * cos(yaw)
        yaw += w * dt_s

    if yaw < -M_PI:
      yaw_ += M_PI * 2
    elif yaw >= M_PI:
      yaw_ -= M_PI * 2

    return x_, y_, yaw_


def main():
    dt = 0.02
    xs = []
    ys = []
    MAX = 0.1
    vr = MAX
    vl = MAX
    acc = 4
    x = 0
    y = 0
    yaw = 0
    for i in range(100):
        x, y, yaw = fwd(x, y, yaw, vl, vr, dt)
        print(vl)
        xs.append(x)
        ys.append(y)
        vl = MAX * (i / 50.0) ** 2

    plt.plot(xs, ys)
    plt.show()

if __name__ == '__main__':
    main()

