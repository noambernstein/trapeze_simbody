#!/usr/bin/env python

import math

length = 10.0
n_length = 10
width = 3.0
n_width = 3
r = 100.0
z0 = None
for i_length in range(n_length+1):
    for i_width in range(n_width+1):
        x = length * float(i_length)/float(n_length) - length/2.0
        y = width * float(i_width)/float(n_width) - width/2.0
        z = math.sqrt(r**2 - x**2 - y**2)
        if z0 is None:
            z0 = z
        z = z0 - z
        print "v", x,y,z


for i_length in range(n_length):
    for i_width in range(n_width):
        i0 = i_length*(n_width+1)+i_width
        i1 = i_length*(n_width+1)+i_width+1
        i2 = (i_length+1)*(n_width+1)+i_width+1
        i3 = (i_length+1)*(n_width+1)+i_width
        print "f",i0,i1,i2
        print "f",i0,i2,i3
