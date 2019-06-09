import numpy as np
from matplotlib import pyplot as plt


def main():
    tt = np.linspace(0.0, 3.0)

    v_mph = 40
    mps_per_mph = 2.24
    v_mps = v_mph * (1.0 / mps_per_mph)

    angle_rad = np.radians(10)

    vx0_mps = v_mps * np.cos(angle_rad)
    vy0_mps = v_mps * np.sin(angle_rad)

    ax_mpss = 0.0
    ay_mpss = -9.81
    pos_x_m = (vx0_mps * tt) + (ax_mpss * (tt ** 2.0) * 0.5)
    pos_y_m = (vy0_mps * tt) + (ay_mpss * (tt ** 2.0) * 0.5)

    pos_x_ft = pos_x_m * 3.28084
    pos_y_ft = pos_y_m * 3.28084
    plt.plot(pos_x_ft, pos_y_ft)
    plt.axis('square')
    plt.ylim([0.0, np.max(pos_y_ft)])
    # plt.xlim([0.0, 40.0])

    plt.xlabel('Distance (ft)')
    plt.ylabel('Height (ft)')
    plt.show()


if __name__ == '__main__':
    main()
