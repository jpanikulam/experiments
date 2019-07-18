import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit

data = np.genfromtxt("/home/jacob/repos/hover-jet/bin/ppdata", delimiter=',')

plt.title('Camera Radiometric Calibration')
plt.ylabel('Camera Reported Intensity')
plt.xlabel('Exposure')
# plt.xscale('log')

nrm_d = data[:, 0] / 2048

plt.scatter(nrm_d, data[:, 1], label='0')
plt.scatter(nrm_d, data[:, 2], label='1')
plt.scatter(nrm_d, data[:, 3], label='2')
plt.scatter(nrm_d, data[:, 4], label='3')
plt.scatter(nrm_d, data[:, 5], label='4')
plt.legend()
plt.show()

umax = np.max(data[:, 0])
tt = np.linspace(np.min(data[:, 0]), umax)


exit(0)


def func(x, exponent, left_shift, scaling, xscl):
    max_x = 2048
    # max_y = scaling
    # return max_y * (np.arctan(exponent * ((np.maximum(x, 0.0001) / max_x) + left_shift)) + 1.0)

    return scaling * (np.exp(exponent * (np.log(xscl * (np.maximum(x, 0.0001) / max_x)) + left_shift)))
    # return scaling * (np.power(xscl * (x / max_x) + left_shift, exponent))


p0 = np.array([0.2, 0.2, 1.0, 0.1])
popt, pcov = curve_fit(func, data[:, 0], data[:, 1], p0)

tt = np.linspace(np.min(data[:, 0]), np.max(data[:, 0]))
plt.plot(tt, func(tt, *popt), 'r-')

print popt
# uu = np.array([0.61476368, 6.25435345, 6.45954789, 1.04694536])

for val_offset in np.linspace(0.0, 0.1, 10):
    index = 0
    offset = np.zeros_like(p0)
    offset[index] += val_offset
    p_guess = popt + offset
    plt.plot(tt, func(tt, *p_guess))

plt.show()
