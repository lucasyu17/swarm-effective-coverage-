import numpy as np
import os
from scipy.integrate import quad,dblquad,nquad
from matplotlib import cm
import scipy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d import Axes3D
import yaml
import numpy.ma as ma

from includes import taskDef as task

f_yaml = open('includes/config.yaml')
cont = f_yaml.read()
configs = yaml.load(cont)
AgentsConfig = configs['Agents']
FieldsConfig = configs['Field']
SimuConfig = configs['Simulation']
InitConfig = SimuConfig['initialization']

# simulation = task.Task(AgentsConfig=AgentsConfig, InitializationConfig=InitConfig,
#                        FieldConfig=FieldsConfig, SimuConfig=SimuConfig)
# simulation.run()

fig = plt.figure()
ax = fig.add_subplot(111)
plt.show()

for i in range(100):
    ax.plot([i/100, i/50])
    plt.pause(0.3)


# flag = True
# if flag:
#     flag = False
# if not(flag):
#     print(1)
#
# aa = np.array([[[1,2,3],[4,5,6],[7,8,9]],[[11,12,13],[14,15,16],[17,18,19]]])
# aa = aa.reshape(2,-1)
#
# print(aa)

x = np.arange(-1, 1.1, 0.2)
y = np.arange(-1, 1.1, 0.2)
X,Y = np.meshgrid(x,y)
# Z = np.ones(X.shape)
# print(1e-3)
# print(scipy.integrate.trapz(scipy.integrate.trapz(Z, x), y))
# W = Y[Y>=0.5]
# print(Y)
# fig = plt.figure()
# ax = axes3d(fig)
# X = np.arange(-4,4,0.25)
# Y = np.arange(-4,4,0.25)



# aa = np.array([[1],[2],[3],[35],[6],[7]])
# bb = np.transpose(aa)
#
# print(bb-aa)