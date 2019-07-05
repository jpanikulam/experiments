import numpy as np
from matplotlib import pyplot as plt

data_path = "/home/jacob/repos/hover-jet/bin/ppdata"

data = np.genfromtxt(data_path, invalid_raise=False, skip_header=11, delimiter=',')
print data
data = data[data[:, 1] > 32]


def histo():
    plt.figure('Just error')
    plt.title("Angle of gravity w.r.t Fiducial X-Axis")
    plt.hist(data[:, 0], bins=50)
    plt.xlabel('Estimated Angle (rad)')
    plt.ylabel('Count')

    plt.figure('Just Data')
    plt.subplot(211)
    plt.plot(data[:, 0])
    plt.subplot(212)
    plt.plot(data[:, 1])


def plot():
    plt.figure("Error w.r.t detection count")

    plt.subplot(121)
    plt.title("Angle of gravity w.r.t Fiducial X-Axis")
    plt.hist2d(data[:, 0], data[:, 1], bins=(10, 10))
    plt.xlabel('Angle (rad)')
    plt.ylabel('Number of points Detected')

    # plt.figure("Error w.r.t average reproj error")
    plt.subplot(122)
    plt.title("Angle of gravity w.r.t Fiducial X-Axis")
    plt.hist2d(data[:, 0], data[:, 2], bins=(10, 10))
    plt.xlabel('Angle (rad)')
    plt.ylabel('Average Reprojection Error')


histo()
plot()
plt.show()
