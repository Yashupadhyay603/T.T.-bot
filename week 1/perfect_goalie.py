import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np
import math
def simple():
    x = np.random.randint(3, 9) / 10
    y = np.random.randint(-4, 4) / 10
    puck_pos = [x, y, 0.32]
    puck_ori = p.getQuaternionFromEuler([0, 0, 0])
    puck = p.loadURDF("puck(1).urdf", puck_pos, puck_ori)
    vel = np.random.randint(-20, 0) / 10
    print("given velocity", vel)
    p.resetBaseVelocity(puck, [vel, 0, 0])
    return puck
def notsosimple():
    x = np.random.randint(3, 9) / 10
    y = np.random.randint(-4, 4) / 10
    puck_pos = [x, y, 0.32]
    puck_ori = p.getQuaternionFromEuler([0, 0, 0])
    puck = p.loadURDF("puck(1).urdf", puck_pos, puck_ori)
    vel_x = np.random.randint(-40, -30) / 10
    vel_y = np.random.randint(-30, 30) / 10
    vel = math.sqrt(vel_x * vel_x + vel_y * vel_y)
    p.resetBaseVelocity(puck, [vel_x, vel_y, 0])
    print("given velocity", vel)
    return puck
def getcentre(image):
    center = ()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 5, 255)
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    for c in contours:
        if cv2.contourArea(c) < 400:
            x,y,w,h = cv2.boundingRect(c)
            M = cv2.moments(c)
            if w != 0:
                center = ((M["m10"]/M["m00"]),(M["m01"]/M["m00"]))

    return center
def cal_pos_defend(position1,position2):
    y=position1[1]+(position2[1]-position1[1])/(position2[0]-position1[0])*(40-position1[0])
    while(y<67 and y>186):
        if(y<67):
            dif=68-y
            y=68+dif
        elif(y>186):
            dif=y-186
            y=186-dif
    if(y<125):
        y=(-0.5)/(125-66)*(y-125)
    elif y>125:
        y = (0.5) / (125 - 185) * (y - 125)
    else:
        y=0
    return y
def cal_pos_attack(pos):
    y2=np.random.randint(120,150)
    x2=234
    m=0
    if(x2!=pos[0]):
        m=(y2-pos[1])/(x2-pos[0])
        sin = m / math.sqrt(m * m + 1)
        cos = 1 / math.sqrt(m * m + 1)
        y = pos[1] + 14 * sin
        x = pos[0] + 70* cos
        if (y < 125):
            y = (-0.5) / (125 - 66) * (y - 125)
        elif y > 125:
            y = (0.5) / (125 - 185) * (y - 125)

        x = (0.9 / (234 - 128)) * (x - 128)
        return x, y
PhysicsClent = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
plane = p.loadURDF("plane.urdf")
table_pos = [0, 0, 0]
table_ori = p.getQuaternionFromEuler([0, 0, 0])
table = p.loadURDF("table(1).urdf", table_pos, table_ori)
hitbot_pos=[-0.45,0,0.26]
hitbot_ori=p.getQuaternionFromEuler([0,0,0])
hitbot=p.loadURDF("hitbot.urdf",hitbot_pos,hitbot_ori)
p.changeDynamics(table, 0, restitution=1)
p.changeDynamics(table, 1, restitution=1)
p.changeDynamics(table, 2, restitution=1)
p.changeDynamics(table, 3, restitution=1)
p.changeDynamics(table, 4, restitution=1)
p.changeDynamics(table, 5, restitution=1)
p.createConstraint(plane, -1, table, -1, p.JOINT_FIXED, [0, 0, 0], table_pos, [0, 0, 0])
p.createConstraint(table,-1,hitbot,-1, p.JOINT_FIXED,[0,0,0],hitbot_pos,[0,0,0],hitbot_ori)
time.sleep(1)
count = 0
puck = notsosimple()
p.changeDynamics(puck, -1, restitution=1)
pos=[]
for i in range(10000):
    p.setRealTimeSimulation(1)
    p.setTimeStep(1.0/240.0)
    view = p.computeViewMatrix([0, 0, 1.4], [0, 0, 0.5], [0, 1, 0])
    projection = p.computeProjectionMatrixFOV(90, 1, 0.1, 3.1)
    images = p.getCameraImage(256, 256, viewMatrix=view, projectionMatrix=projection)
    rgb = images[2]
    rgb = np.array(rgb)
    rgb = np.reshape(rgb, (256, 256, 4))
    rgb = np.uint8(rgb)
    image = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    pos1 = getcentre(image)
    pos.append(pos1)
    x_pos = -0.8
    y_pos = 0
    mv = 1
    if len(pos)==2 :
        if pos[0][0]>pos[1][0] :
            x_pos=-0.8
            y_pos = cal_pos_defend(pos[0],pos[1])
            mv=1
        elif pos1[0]<60 :
            x_pos,y_pos=cal_pos_attack(pos[1])
            mv=3
        p.setJointMotorControl2(hitbot, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=x_pos,
                                maxVelocity=mv)
        p.setJointMotorControl2(hitbot, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=y_pos,
                                maxVelocity=mv)
        pos.clear()
p.disconnect()
