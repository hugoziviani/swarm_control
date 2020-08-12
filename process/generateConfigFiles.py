import os
newfile = "/home/hz/robocomp/components/autonomous/etc/config"


os.system("rm -rf ../etc/*") #limpa a pasta
port = 10004
comp = 11000
doble_count = 0
for count in range(0,10):
    content = [
    "RobotId = " + str(count) + "\n"
    "DifferentialRobotProxy = differentialrobot:tcp -h 192.168.0.6 -p " + str(port + doble_count) + "\n",
    "LaserProxy = laser:tcp -h 192.168.0.6 -p " + str(port + doble_count + 1) + "\n",
    "Ice.Warn.Connections=0\n",
    "Ice.Trace.Network=0\n",
    "Ice.Trace.Protocol=0\n"
    ]
    with open(newfile+str(count), 'w') as outfile:
        outfile.writelines(content)
    
    doble_count = (doble_count + 2)