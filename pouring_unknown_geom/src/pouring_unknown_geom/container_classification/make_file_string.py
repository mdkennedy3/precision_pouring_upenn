#!/usr/bin/env python
l = []
for idx in range(284):
  if idx < 100:
    l.append('S0'+str(idx))
  else:
    l.append('S'+str(idx))
print "L: ",l

