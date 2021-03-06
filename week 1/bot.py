import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np
import math


def getcentre(image):
    center = ()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 10, 255)
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    for c in contours:
        if (cv2.contourArea(c) < 700):
            (x, y), r = cv2.minEnclosingCircle(c)
            r = int(r)
            if r != 0:
                center = (int(x), int(y))
                cv2.circle(image, center, r, (0, 255, 0), 2)
                centorg = ((center[0] - image.shape[1] / 2), (center[1] - image.shape[
                    0] / 2))  # shifting the origin to centre of table and then locating centre of puck
    return centorg


# setting physics client
PhysicsClent = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# setting gravity
p.setGravity(0, 0, -10)

# setting plane
plane = p.loadURDF("plane.urdf")

# setting table
table_pos = [0, 0, 0]
table_ori = p.getQuaternionFromEuler([0, 0, 0])
table = p.loadURDF("table(1).urdf", table_pos, table_ori)

# setting puck
puck_pos = [-0.6, 0.3, 0.3]
puck_ori = p.getQuaternionFromEuler([0, 0, 0])
puck = p.loadURDF("puck(1).urdf", puck_pos, puck_ori)
# hitbot

hitbot_pos=[0.41,0,0.31]
hitbot_ori=p.getQuaternionFromEuler([0,0,3.14])
hitbot=p.loadURDF("hitbot.urdf",hitbot_pos,hitbot_ori)


# setting kuka bot
kuka = p.loadURDF("model.urdf", [-1.0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

# creating constraints for table
p.createConstraint(plane, -1, table, -1, p.JOINT_FIXED, [0, 0, 0], table_pos, [0, 0, 0])
p.createConstraint(plane,-1,hitbot,-1, p.JOINT_FIXED,[0,0,0],hitbot_pos,[0,0,0],hitbot_ori)

ALPHA = 1
centrepoints = []
timestamps = []
for i in range(10000):
    p.stepSimulation()

    puck_pos, puck_ori = p.getBasePositionAndOrientation(puck)

    force = ALPHA * (np.array([2.05, -3.0, puck_pos[2]]) - np.array(puck_pos))
    p.applyExternalForce(objectUniqueId=puck, linkIndex=-1,
                         forceObj=force, posObj=puck_pos, flags=p.WORLD_FRAME)

    # setting camera
    view = p.computeViewMatrix([0, 0, 1.4], [0, 0, 0.5], [0, 1, 0])
    projection = p.computeProjectionMatrixFOV(90, 1, 0.1, 3.1)

    # getting images
    images = p.getCameraImage(512, 512, viewMatrix=view, projectionMatrix=projection)
    rgb=images[2]
    rgb= np.array(rgb)
    rgb= np.reshape(rgb,(512,512,4))
    rgb= np.uint8(rgb)
    image= cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)
    centrec = getcentre(image)

    centrepoints.append(centrec)  # append all centres in a list
    timestamps.append(time.time())  # time at which the centre position was recorded appended into this list
    if (i % 2 == 0 and i != 0):
        print(centrec)
        dx = centrepoints[i - 2][0] - centrepoints[i][
            0]  # difference in x-coord of the two centre positions at i and i-2
        dy = centrepoints[i - 2][1] - centrepoints[i][
            1]  # difference in y-coord of the two centre positions at i and i-2
        distance = math.sqrt(abs(dx) * abs(dx) + abs(dy) * abs(dy))  # distance btw the two centres
        print(distance)
        velocity = distance / abs(timestamps[i] - timestamps[i - 2])  # speed calc using speed=distance/time
        print(velocity)
        angle = math.degrees(math.atan2(dy, dx))  # angle from the x-axis
        print("angle =", angle)
    time.sleep(1. / 1000.)

p.disconnect()
