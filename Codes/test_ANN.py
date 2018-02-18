# Import datasets, classifiers and performance metrics
from sknn.mlp import Classifier, Layer
import numpy as np
import logging
import pickle
logging.basicConfig()

# The digits dataset
opt = []
f = open("feature_open_hand", "r+")
ipt_open = f.read()
f.close()
ipt_open  = ipt_open.split("\n")

for i in range(0,len(ipt_open)-1):
	ipt_open[i] = ipt_open[i].strip("[]").split(",")
	# print ipt_open
	ipt_open[i][0] = float(ipt_open[i][0])/7
	ipt_open[i][1] = float(ipt_open[i][1])/500
	opt.append(1)

f = open("feature_closed_hand", "r+")
ipt_closed = f.read()
f.close()
ipt_closed  = ipt_closed.split("\n")

for i in range(0,len(ipt_closed)-1):
	ipt_closed[i] = ipt_closed[i].strip("[]").split(",")
	ipt_closed[i][0] = float(ipt_closed[i][0])/7
	ipt_closed[i][1] = float(ipt_closed[i][1])/500
	opt.append(0)

ipt = ipt_open[:-1]+ipt_closed[:-1]

# ipt = [[2/7,30/400],[6/7,400/400]]
# opt = [0,1]
ipt = np.asarray(ipt)
opt = np.asarray(opt)

print ":"+str(len(ipt))
print len(opt)
print opt
print ipt

nn = Classifier(
    layers=[
        Layer("Sigmoid", units=2),
        # Layer("Softmax",units=5),
        Layer("Softmax")],
    learning_rate=0.5,
    n_iter=5)

# nn.set_parameters([([[-3.75906463,  1.26411728],[-5.44439202,  0.44432455]], [ 2.63582797, -0.23474542]), 
				   # ([[ 4.32310838, -5.46097277],[-1.114463  ,  1.37638111]], [-2.13190273,  2.13190273])])
nn.fit(ipt,opt)

a = np.asarray([[2/7,30/400],[4/7,30/400],[6/7,400/400],[4/7,400/400]])
# a = np.asarray([[2,30],[4,30],[6,400],[4,400]])
# a =a.reshape(2,-1)


# params = nn.get_parameters()

# print "::NEW::"

# tnn = Classifier(
#     layers=[
#         Layer("Sigmoid", units=2),
#         # Layer("Softmax",units=5),
#         Layer("Softmax")],
#     learning_rate=0.5,
#     n_iter=5)

# tnn.set_parameters(params)

op = nn.predict(a)
pickle.dump(nn, open('nn.pkl','wb'))