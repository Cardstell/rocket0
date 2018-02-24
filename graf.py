import matplotlib.pyplot as plt
import time

md = dict()
for i in range(10):
	md[i] = []
with open('rdata.txt','r') as f:
	cnt = 0
	for l in f:
		if (cnt%2==0):
			for i in range(len(l.split())):
				md[i].append(float(l.split()[i]))
		cnt += 1
#plt.figure(1)
#plt.subplot(321)
#plt.plot(md[0], md[1], label='Height')
#plt.legend()
#plt.subplot(323)
#plt.plot(md[0], md[2], label='Speed')
#plt.legend()
#plt.subplot(325)
#plt.plot(md[0], md[3], label='Acceleration')
#plt.legend()
#plt.subplot(324)
#plt.plot(md[0], md[4], label='Thrust')
#plt.legend()
#plt.subplot(322)
#plt.plot(md[0], md[5], label='Mass')
#plt.legend()
#plt.subplot(326)
#plt.plot(md[0], md[6], label='Aero')
#plt.legend()
#plt.show()
kxr = 0.05
lns = [None]*6
axs = [None]*6
plt.ion()
fig = plt.figure(1)
axs[0] = fig.add_subplot(321)
lns[0], = axs[0].plot(md[0][:1],md[1][:1],label='Height')
axs[0].legend()
axs[1] = fig.add_subplot(323)
lns[1], = axs[1].plot(md[0][:1],md[2][:1],label='Speed')
axs[1].legend()
axs[2] = fig.add_subplot(325)
lns[2], = axs[2].plot(md[0][:1],md[3][:1],label='Acceleration')
axs[2].legend()
axs[3] = fig.add_subplot(322)
lns[3], = axs[3].plot(md[0][:1],md[4][:1],label='Thrust')
axs[3].legend()
axs[4] = fig.add_subplot(324)
lns[4], = axs[4].plot(md[0][:1],md[5][:1],label='Mass')
axs[4].legend()
axs[5] = fig.add_subplot(326)
lns[5], = axs[5].plot(md[0][:1],md[6][:1],label='Aero')
axs[5].legend()
limits = [None]*6
for i in range(6):
	limits[i] = [md[i][0], md[i][0]]
for i in range(2,len(md[0])):
	for j in range(6):
		lns[j].set_xdata(md[0][:i])
		lns[j].set_ydata(md[j+1][:i])
		mnx = 0;mxx = md[0][i-1]
		limits[j][0] = min(limits[j][0], md[j+1][i-1])
		limits[j][1] = max(limits[j][1], md[j+1][i-1])
		xrng = abs(mxx - mnx)
		yrng = abs(limits[j][1] - limits[j][0])
		axs[j].set_ylim([limits[j][0]-yrng*kxr, limits[j][1]+yrng*kxr])
		axs[j].set_xlim([mnx - xrng*kxr, mxx + xrng*kxr])
		
	fig.canvas.draw()
	fig.canvas.flush_events()
	#time.sleep(0.0)