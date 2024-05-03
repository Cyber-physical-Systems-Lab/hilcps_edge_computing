from pymycobot import MyCobotSocket
# Use port 9000 by default
# Where "192.168.10.22" is the IP of the robot arm
mc = MyCobotSocket("192.168.0.223",9001)

#After the connections is normal, the robot arm can be controlled.
res = mc.get_angles()
print(res)
mc.send_angles([0,0,0,0,0,0],20)