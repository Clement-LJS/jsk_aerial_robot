from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

P=Path(__file__).parent
s=np.load(P/'series.npz');base=1793513197.256713
fig,ax=plt.subplots(7,1,sharex=True,figsize=(13,15),layout='constrained')
pid=s['pid'];od=s['uav/cog/odom'];imu=s['imu'];pwm=s['pwm'];joint=s['joints'];mc=s['mocap/pose']
for axis in range(3):
    ax[0].plot(od[:,0]-base,od[:,3+axis],label='XYZ'[axis])
    ax[0].plot(pid[:,0]-base,pid[:,2+8*axis],ls='--',alpha=.6)
ax[0].set_ylabel('CoG position / target (m)');ax[0].legend(ncol=3)
for axis in range(3):
    ax[1].plot(od[:,0]-base,np.degrees(od[:,6+axis]),label=('roll','pitch','yaw')[axis])
    ax[1].plot(pid[:,0]-base,np.degrees(pid[:,2+8*(axis+3)]),ls='--',alpha=.7)
ax[1].set_ylabel('CoG attitude / target (deg)');ax[1].legend(ncol=3)
for i in range(3):ax[2].plot(imu[:,0]-base,imu[:,5+i],label=('gx','gy','gz')[i],lw=.8)
ax[2].set_ylabel('Gyro (rad/s)');ax[2].legend(ncol=3)
for i in range(4):ax[3].plot(pwm[:,0]-base,pwm[:,1+i],label=str(i+1),lw=.8)
ax[3].set_ylabel('Motor PWM equivalent');ax[3].legend(ncol=4)
for i in range(4):ax[4].plot(joint[:,0]-base,np.degrees(joint[:,1+i]),label=str(i+1),lw=.8)
ax[4].set_ylabel('Gimbal feedback (deg)');ax[4].legend(ncol=4)
ax[5].plot(mc[1:,0]-base,np.diff(mc[:,0])*1000,label='Bag arrival')
ax[5].plot(mc[1:,0]-base,np.diff(mc[:,1])*1000,label='Header time',alpha=.7)
ax[5].set_ylabel('Mocap interval (ms)');ax[5].legend(ncol=2)
v=s['voltage'];ax[6].plot(v[:,0]-base,v[:,1]);ax[6].set_ylabel('Voltage (V)')
marks=[(14.5367,'Takeoff'),(26.537,'Hover'),(44,'Z up'),(47.145,'Z down'),(50.359,'Yaw'),(67.73,'Land'),(69.934,'Halt')]
for a in ax:
    a.grid(alpha=.2)
    for t,label in marks:a.axvline(t,c='grey',ls='--',lw=.6)
for t,label in marks:ax[0].text(t,1.02,label,rotation=65,fontsize=8,transform=ax[0].get_xaxis_transform())
ax[-1].set_xlabel('Seconds after 15:06:37.257 JST (November 1)')
fig.suptitle('Real flight: measured response, commands and timing (dashed targets)')
fig.savefig(P/'flight_overview.png',dpi=140)
for a in ax:a.set_xlim(48,72)
fig.savefig(P/'flight_late.png',dpi=140)
print(P/'flight_overview.png')
