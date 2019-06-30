import numpy as np
from matplotlib import pyplot as plt

data_path = "/home/jacob/repos/hover-jet/bin/ppdata"

data = np.genfromtxt(data_path)
srt = np.sort(data)

plt.hist(np.diff(srt), bins=np.linspace(0, 0.2, 50))

# plt.plot(data, 'r.')
plt.show()
