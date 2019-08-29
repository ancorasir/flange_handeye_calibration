from matplotlib import pyplot as pl
import numpy as np
import matplotlib.pyplot as plt

# data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/aubo"
# robot = 'aubo'
# data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-05-28/10:59:06"
# robot = 'ur5'
# data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-05-28/14:45:39"
# robot = 'ur10e'
data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-06-05/16:57:46"
robot = 'franka'
# data_path = "/home/bionicdl/kinect_simulation_data/20190730"
# robot = 'Simulated UR5'

data = np.load(data_path+"/results_photoneo.npz")
error = data["arr_0"]
xyz = data["arr_1"]
rpy = data["arr_2"]
NUM_POINTS = xyz.shape[1]

pl.clf()
pl.hold(1)
tt = np.linspace(5, NUM_POINTS+4, NUM_POINTS)

x = np.mean(xyz[:,:,0], axis=0)
error = np.std(xyz[:,:,0], axis=0)
pl.plot(tt, x, 'k', color='#CC4F1B',label="x")
pl.fill_between(tt, x-error, x+error,
    alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')

y = np.mean(xyz[:,:,1], axis=0)
error = np.std(xyz[:,:,1], axis=0)
pl.plot(tt, y, 'k', color='#1B2ACC',label="y")
pl.fill_between(tt, y-error, y+error,
    alpha=0.2, edgecolor='#1B2ACC', facecolor='#089FFF')

z = np.mean(xyz[:,:,2], axis=0)
error = np.std(xyz[:,:,2], axis=0)
pl.plot(tt, z, 'k', color='#3F7F4C',label="z")
pl.fill_between(tt, z-error, z+error,
    alpha=0.5, edgecolor='#3F7F4C', facecolor='#7EFF99',
    linewidth=0)

plt.xlabel('Number of points', fontsize=14)
plt.ylabel('Translation error (mm)', fontsize=14)
plt.xlim(5, 40)
plt.legend()

plt.savefig(robot+'_xyz',dpi=600)
########################################
pl.clf()
pl.hold(1)

r = np.mean(rpy[:,:,0], axis=0)
error = np.std(rpy[:,:,0], axis=0)
pl.plot(tt, r, 'k', color='#CC4F1B', label='Roll')
pl.fill_between(tt, r-error, r+error,
    alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')

p = np.mean(rpy[:,:,1], axis=0)
error = np.std(rpy[:,:,1], axis=0)
pl.plot(tt, p, 'k', color='#1B2ACC', label='Pitch')
pl.fill_between(tt, p-error, p+error,
    alpha=0.2, edgecolor='#1B2ACC', facecolor='#089FFF')

y = np.mean(rpy[:,:,2], axis=0)
error = np.std(rpy[:,:,2], axis=0)
pl.plot(tt, y, 'k', color='#3F7F4C', label='Yaw')
pl.fill_between(tt, y-error, y+error,
    alpha=0.5, edgecolor='#3F7F4C', facecolor='#7EFF99',
    linewidth=0)

plt.xlabel('Number of points', fontsize=14)
plt.ylabel('Rotation error (degree)', fontsize=14)
plt.xlim(5, 40)
# plt.ylim(-0.15, 0.15)
plt.legend()

plt.savefig(robot+'_rpy',dpi=600)
