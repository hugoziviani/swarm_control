import os
import threading
import time
import subprocess

process = list()
for p in range(0,16):
    process.append(subprocess.Popen(args=[ "python3", "src/MyFirstComp.py", "etc/config"+str(p)]))

time.sleep(120)

for p in process:
    p.kill()
# threading._start_new_thread(os.system('python3 src/MyFirstComp.py etc/config0'),"(process0)",)
# threading._start_new_thread(os.system('python3 src/MyFirstComp.py etc/config1'),"(process1)",)
# threading._start_new_thread(os.system('python3 src/MyFirstComp.py etc/config2'),"(process2)",)






# a = np.array([3,5,2,5,3,4,3,4,1,5,5,5,3,3,2,3,3,2])
# print(len(a))
# print("total", a.sum())
# print("A (urgencia): ", (a[0]+a[2]+a[5]+a[8]+a[11]+a[14])/a.sum()  *100, "%")
# print("B (importante): ", (a[3]+a[6]+a[9]+a[10]+a[13]+a[16])/a.sum() *100, "%")
# print("C (circunstancia): ", (a[1]+a[4]+a[7]+a[12]+a[15]+a[17])/a.sum() *100, "%")