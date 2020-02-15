#!/usr/bin/env python
# coding=utf8

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math


s = list()
# Data for plotting
t = np.arange(0, 5.0, 0.1)
k = np.arctan(-t)
#
# # e1
# for x in t:
#
#     offset_x = 3.0
#     offset_y = 0.5
#
#     scale_x = 1
#     scale_y = 0.4
#
#     f = math.atan((-x+offset_x)*scale_x)*scale_y+offset_y
#     # f =  1.0 if f > 1.0 else f
#     # f =  0.0 if f < 0 else f
#
#     s.append(f)


for x in t:

    offset_x = 3.0
    offset_y = 0.5

    scale_x = 3
    scale_y = 0.4

    f = math.atan((-x+offset_x)*scale_x)*scale_y+offset_y
    f =  1.0 if f > 1.0 else f
    f =  0.0 if f < 0 else f

    s.append(f)

fig, ax = plt.subplots()
ax.plot(t, s)
# ax.plot(t, k)

ax.set(xlabel='dist (m)', ylabel='efficiency (%)')
ax.grid()

fig.savefig("test.png")
plt.show()