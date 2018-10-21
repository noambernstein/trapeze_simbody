#!/usr/bin/env python

import math

length = 25.0
n_length = 25
width = 3.0
n_width = 3
r = 200.0
x0 = - length/2.0
y0 = - width/2.0
z0 = math.sqrt(r**2 - x0**2 - y0**2)
for i_length in range(n_length+1):
    for i_width in range(n_width+1):
        x = length * float(i_length)/float(n_length) - length/2.0
        y = width * float(i_width)/float(n_width) - width/2.0
        z = z0 - math.sqrt(r**2 - x**2 - y**2)
        print "v", x,z,y

for i_length in range(n_length):
    for i_width in range(n_width):
        i0 = i_length*(n_width+1)+i_width
        i1 = i_length*(n_width+1)+i_width+1
        i2 = (i_length+1)*(n_width+1)+i_width+1
        i3 = (i_length+1)*(n_width+1)+i_width
        print "f",i0+1,i1+1,i2+1
        print "f",i0+1,i2+1,i3+1
