import math


# 计算方位角函数
def azimuthAngle( x1, y1, x2, y2):
    angle = 0.0;
    dx = x2 - x1
    dy = y2 - y1
    if x2 == x1:
        angle = math.pi / 2.0
        if y2 == y1 :
            angle = 0.0
        elif y2 < y1 :
            angle = 3.0 * math.pi / 2.0
    elif x2 > x1 and y2 > y1:
        angle = math.atan(dx / dy)
    elif x2 > x1 and y2 < y1 :
        angle = math.pi / 2 + math.atan(-dy / dx)
    elif x2 < x1 and y2 < y1 :
        angle = math.pi + math.atan(dx / dy)
    elif x2 < x1 and y2 > y1 :
        angle = 3.0 * math.pi / 2.0 + math.atan(dy / -dx)
    return angle

def importlidar02():
    from PIL import Image
    im = Image.open("lidar02.png")

    import matplotlib.pyplot as plt

    pixels = []
    rgblist = list(im.getdata())

    width, height = im.size
    count = 0
    for index in range(len(rgblist)):
        if(rgblist[index][0]) < 10:
            x = int(index % width) / 500 - 0.55
            y = int(index // width) / 500 - 0.8
            pixels.append([x, y])

    import numpy as np 
    pts = np.asarray(pixels)
    
    final = []
    for pt in pts:
        # angle = math.atan(pt[1]/pt[0])
        # angle = azimuthAngle(0, 0, pt[0], pt[1])
        angle = math.atan2(pt[1], pt[0])
        if angle < 0.0:
            angle += math.pi * 2.0
        elif angle > math.pi * 2.0:
            angle -= math.pi * 2.0
        distance = math.sqrt(pt[1]**2 + pt[0]**2)
        final.append([angle, distance])
    np.savetxt("lidar02.csv", final, fmt="%f", delimiter=",") 

    x = []
    y = []
    for finalpt in final:
        x.append(finalpt[1] * math.cos(finalpt[0]))
        y.append(finalpt[1] * math.sin(finalpt[0]))

    plt.plot(x, y, 'oc')
    bottom, top = plt.ylim()  # return the current y-lim
    plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
    plt.grid(True)
    plt.show()
    
    

importlidar02()