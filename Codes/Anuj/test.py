# Import datasets, classifiers and performance metrics
from sknn.mlp import Classifier, Layer
import numpy as np
# The digits dataset
opt = []

f = open("feature_open_hand", "r+")
ipt_open = f.read()
f.close()
ipt_open  = ipt_open.split("\n")
for i in range(0,len(ipt_open)-1):
	ipt_open[i] = ipt_open[i].strip("[]").split(",")
	# print ipt_open
	ipt_open[i][0] = int(ipt_open[i][0])
	ipt_open[i][1] = int(ipt_open[i][1])
	opt.append(1)

f = open("feature_closed_hand", "r+")
ipt_closed = f.read()
f.close()
ipt_closed  = ipt_closed.split("\n")
for i in range(0,len(ipt_closed)-1):
	ipt_closed[i] = ipt_closed[i].strip("[]").split(",")
	ipt_closed[i][0] = int(ipt_closed[i][0])
	ipt_closed[i][1] = int(ipt_closed[i][1])
	opt.append(0)

ipt = ipt_open[:-1]+ipt_closed[:-1]
ipt = np.asarray(ipt)
opt = np.asarray(opt)
print ":"+str(len(ipt))
print len(opt)
nn = Classifier(
    layers=[
        Layer("Softmax", units=5),
        Layer("Softmax",units=2),
        Layer("Softmax")],
    learning_rate=0.05,
    n_iter=10)
nn.fit(ipt,opt)

a = np.asarray([[6,300]])
# a =a.reshape(2,-1)
op = nn.predict(a)

print op