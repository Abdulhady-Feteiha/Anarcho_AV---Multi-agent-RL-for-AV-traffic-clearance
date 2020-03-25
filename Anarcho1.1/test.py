#!/usr/bin/env python
from math import sqrt
agaccel = -0.2
emeraccel = -0.7
agspd = 15
emerspd = 10
dist = 15
a = 0.5*(agaccel-emeraccel)
b = (agspd-emerspd)
c = dist
neg = (-b - sqrt(b**2 - 4*a*c)) / (2 * a)
po =(-b + sqrt(b**2 - 4*a*c)) / (2 * a)
print(max(neg,po))
print(neg," ",po)
