import numpy as np

delays = [3, 1, 4, 5, 1]

sig = np.random.uniform(low=0, high=2, size=100000)

sig[200:300] += 20
t = 200
for d in delays:
    t += d * 1000
    sig[t-50:t+50] += 20

with open('dat.txt', 'w') as f:
    f.write('\n'.join(map(str, sig)))

#from matplotlib import pyplot as plt

#plt.plot(sig)
#plt.show()