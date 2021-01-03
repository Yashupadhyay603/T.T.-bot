import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt


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
    vel_x = np.random.randint(-60, -50) / 10
    vel_y = np.random.randint(-20, 20) / 10
    vel = math.sqrt(vel_x * vel_x + vel_y * vel_y)
    p.resetBaseVelocity(puck, [vel_x, vel_y, 0])
    print("given velocity", vel)
    return puck


def getcentre(image):
    center = ()
    centorg = None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 5, 255)
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    for c in contours:
        if cv2.contourArea(c) < 400:
            x,y,w,h = cv2.boundingRect(c)
            M = cv2.moments(c)
            if w != 0:
                center = ((M["m10"]/M["m00"]),(M["m01"]/M["m00"]))
                # cv2.circle(image, center, 0.2, (0, 255, 0), 2)
                # cv2.imshow("image", image)
                # cv2.waitKey(0)
                centorg = ((center[0] - (image.shape[1] / 2)), (center[1] - (image.shape[
                    0] / 2)))  # shifting the origin to centre of table and then locating centre of puck
    return center


def calculatevelocity(i, image, puck):
    velac = 0
    angle = 0
    velocity = 0
    centre = getcentre(image)
    if centre!=None:
        centrepoints.append(centre)  # append all centres in a list
    timestamps.append(time.time())
    # time at which the centre position was recorded appended into this list
    if i % 10 == 0 and i !=0:
        framecheck = i-10
        totaldistance = 0
        angle = math.degrees(math.atan2((centrepoints[i-8][1] - centrepoints[i-10][1]), centrepoints[i-5][0] - centrepoints[i-10][1]))  # angle from the x-axis
        # print(centrec)
        for j in range(framecheck, i):
            dx = centrepoints[j + 1][0] - centrepoints[j][0]  # difference in x-coord of the two centre positions at i and i-2
            dy = centrepoints[j + 1][1] - centrepoints[j][1]# difference in y-coord of the two centre positions at i and i-2
            distance = math.sqrt(abs(dx) * abs(dx) + abs(dy) * abs(dy))  # distance btw the two centres
            totaldistance = totaldistance + distance

            # print(i," ",j," ",framecheck, " ", distance, " " , totaldistance, " ", angle)
        totaltime = abs(timestamps[i] - timestamps[framecheck])
        velocity = (totaldistance / totaltime) / pixelpermetric  # speed calc using speed=distance/time
        # print("Calculated velocity", velocity)
        # print("angle =", angle)
        velac = p.getBaseVelocity(puck)
        # abs_vel = math.sqrt((velac[0][0]**2)+(velac[0][1]**2))
        # print("actual velocity", velac)
    if velocity>0:
        return velocity, centre, angle  #, velac
    else:
        return None


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
        y=(-0.5)/(125-67)*(y-125)
    elif y>125:
        y = (0.5) / (125 - 185) * (y - 125)
    else:
        y=0
    return y









# setting physics client
PhysicsClent = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# setting gravity
p.setGravity(0, 0, -10)

pixelpermetric = 100 / 2.5

# setting plane
plane = p.loadURDF("plane.urdf")

# setting table
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



# creating constraints for table
p.createConstraint(plane, -1, table, -1, p.JOINT_FIXED, [0, 0, 0], table_pos, [0, 0, 0])
p.createConstraint(table,-1,hitbot,-1, p.JOINT_FIXED,[0,0,0],hitbot_pos,[0,0,0],hitbot_ori)
p.setJointMotorControl2(hitbot, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=-.8, maxVelocity=1)


time.sleep(1)
count = 0
puck = notsosimple()
p.changeDynamics(puck, -1, restitution=1)
# notsosimple()
ALPHA = 1
centrepoints = []
timestamps = []
pos=[]
for i in range(10000):
    p.setRealTimeSimulation(1)
    p.setTimeStep(1.0/240.0)
    # setting camera
    view = p.computeViewMatrix([0, 0, 1.4], [0, 0, 0.5], [0, 1, 0])
    projection = p.computeProjectionMatrixFOV(90, 1, 0.1, 3.1)
    images = p.getCameraImage(256, 256, viewMatrix=view, projectionMatrix=projection)

    rgb = images[2]
    rgb = np.array(rgb)
    rgb = np.reshape(rgb, (256, 256, 4))
    # plt.imshow( rgb)
    # plt.show()


    rgb = np.uint8(rgb)


    image = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

    #pos,ori=p.getBasePositionAndOrientation(puck)

    #if(math.fabs(math.tan(ori[2]))>0):
    pos1 = getcentre(image)
    pos.append(pos1)
    # print(pos1)
    print(pos)
    if len(pos)==2:

        y_pos = cal_pos_defend(pos[0],pos[1])

            # print(y_pos)
        p.setJointMotorControl2(hitbot, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=y_pos, maxVelocity=1)
        pos.clear()

    # time.sleep(0.1)
    # print(centre[1])
    # p.setJointMotorControl2(hitbot, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=centre[1],
    #                         maxVelocity=1.5)
    #     #p.setJointMotorControl2(hitbot, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=y_pos, maxVelocity=2)


p.disconnect()
