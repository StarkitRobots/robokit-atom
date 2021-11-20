import sys, os

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if sys.version != '3.4.0': 
    current_work_directory += '/'

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
sys.path.append( current_work_directory + 'Soccer/Vision/')
sys.path.append( current_work_directory + 'Soccer/Localisation/')
sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')

import cv2
import numpy as np
import random, json 
import matplotlib.pyplot as plt

with open(current_work_directory + "Soccer/log/deviation.json", "r") as f:
    data1 = json.loads(f.read())
#pf_avg, pf_cur, ab_avg, ab_cur = [],[],[],[]
#for i in range(len(data1)):
#    pf_avg.append(data1[i]['average pf'])
#    pf_cur.append(data1[i]['current pf'])
#    ab_avg.append(data1[i]['average ab'])
#    ab_cur.append(data1[i]['current ab'])

pf_avg, pf_cur = [],[]
for i in range(len(data1)):
    pf_avg.append(data1[i]['average pf'])
    pf_cur.append(data1[i]['current pf'])


rng = np.arange(len(data1))
fig,ax = plt.subplots(figsize=(10,6))
plt.plot(rng, pf_avg, label = 'PF average')
plt.plot(rng, pf_cur, label = 'PF current')
#plt.plot(rng, ab_avg, label = 'AB average')
#plt.plot(rng, ab_cur, label = 'AB current')

ax.legend(loc='upper left')
ax.grid(True)
plt.show()

#print(a)