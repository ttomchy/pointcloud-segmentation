import matplotlib.pyplot as plt
import numpy as np
#from mpl_toolkits.mplot3d import Axes3D
# result=[]
# with open('z-value.txt') as f:
#     for line in f:
#         result.append(map(float,line.split(',')))
#    # print(result)
# plt.plot(result)
# plt.xlabel('z-value')
# plt.ylabel('numbers')
# plt.show()

fig = plt.figure()
#ax = Axes3D(fig)
result=[]
with open('value.txt') as f:
    for line in f:
        result.append(map(float, line.split(' ')))

#print  result


xyz=np.array(result)

x=xyz[:,0]
y=xyz[:,1]
z=xyz[:,2]


plt.plot(z)
plt.show()
# x_max=np.max(x)
# x_min=np.min(x)
# print 'x_max',x_max
# print 'x_min',x_min
#
# y_max=np.max(y)
# y_min=np.min(y)
# print 'y_max',y_max
# print 'y_min',y_min
#
# z_max=np.max(z)
# z_min=np.min(z)
# print 'z_max',z_max
# print 'z_min',z_min
#
# # ax.scatter(x,y,z)
# #
# #
# # ax.set_zlabel('Z-axis')
# # ax.set_ylabel('Y-axis')
# # ax.set_xlabel('X-axis')
#
#
# plt.show()
#
