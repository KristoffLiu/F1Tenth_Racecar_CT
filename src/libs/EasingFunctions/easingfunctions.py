import math

def easeOutCubic(x):
    return 1 - math.pow(1 - x, 3)

def easeInSine(x):
    return 1 - math.cos(x * math.pi / 2)


def easeInOutSine(x):
    return -(math.cos(math.pi * x) - 1) / 2


def easeInOutCubic(x):
    if x < 0.5:
        return 4 * x * x * x
    else:
        return 1 - pow(-2 * x + 2, 3) / 2

def easeOutSine(x):
    return math.sin(x * math.pi / 2)