import matplotlib.pyplot as plt
import math

def easeInOutSine(x):
    return -(math.cos(math.pi * x) - 1) / 2

def easeInOutCubic(x):
    if x < 0.5:
        return 4 * x * x * x
    else:
        return 1 - pow(-2 * x + 2, 3) / 2

x = []
for i in range(0,100,1):
    x.append(easeInOutSine(i/100) * 8)

max_speed = 8

y = []
for i in range(0,100,1):
    y.append(i * max_speed / 100)

plt.plot(x,y)
plt.show()
