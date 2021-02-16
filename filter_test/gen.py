from random import randint

beeps = [1000, 2000, 3000, 5000, 7000]

val = [0 for _ in range(10000)]

for i in range(len(val)):
    val[i] = randint(0, 10)
    if min(abs(i - j) for j in beeps) < 50:
        val[i] += 30

with open('dat.txt', 'w') as f:
    f.write('\n'.join(map(str, val)))

from matplotlib import pyplot as plt

plt.plot(val)
plt.show()