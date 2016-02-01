import numpy as np

a = np.arange(25).reshape(5,5)
d=0
c=[2,3]
print a
it = np.nditer(a, flags=['multi_index'],op_flags=['readwrite'])
while not it.finished:
	p=it.multi_index


	if (p[0]>c[0]+d or p[0]<c[0]-d) or (p[1]>c[1]+d or p[1]<c[1]-d) :
	
		it[0] = 0
		#print "%d <%s>" % (it[0], it.multi_index),
	it.iternext()
print a