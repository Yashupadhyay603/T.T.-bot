import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np
import math


def simple():
    x=np.random.randint(3,9)/10
    y=np.random.randint(-4,4)/10
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
    vel_x = np.random.randint(-20, 0) / 10
    vel_y = np.random.randint(-20, 0) / 10
    vel = math.sqrt(vel_x*vel_x + vel_y*vel_y)
    p.resetBaseVelocity(puck, [vel_x, vel_y, 0])
    print("given velocity", vel)
    return puck


def getcentre(image):
    center = ()
    centorg = None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 10, 255)
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    for c in contours:
        if cv2.contourArea(c) < 700:
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

pixelpermetric = float(106 / 1.9)


# setting plane
plane = p.loadURDF("plane.urdf")

# setting table
table_pos = [0, 0, 0]
table_ori = p.getQuaternionFromEuler([0, 0, 0])
table = p.loadURDF("table(1).urdf", table_pos, table_ori)

p.changeDynamics(table,0,restitution=1)
p.changeDynamics(table,1,restitution=1)
p.changeDynamics(table,2,restitution=1)
p.changeDynamics(table,3,restitution=1)
p.changeDynamics(table,4,restitution=1)
p.changeDynamics(table,5,restitution=1)

# setting kuka bot
kuka = p.loadURDF("model.urdf", [-1.0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

# creating constraints for table
p.createConstraint(plane, -1, table, -1, p.JOINT_FIXED, [0, 0, 0], table_pos, [0, 0, 0])

time.sleep(1)
count = 0
puck = notsosimple()
p.changeDynamics(puck, -1, restitution=1)
# notsosimple()
ALPHA = 1
centrepoints = []
timestamps = []
for i in range(10000):
    p.setRealTimeSimulation(1)
    p.setTimeStep(1/100)
    # p.stepSimulation()

    # #puck_pos, puck_ori = p.getBasePositionAndOrientation(puck)
    #
    #
    # setting camera
    view = p.computeViewMatrix([0, 0, 1.4], [0, 0, 0.5], [0, 1, 0])

    projection = p.computeProjectionMatrixFOV(90, 1, 0.1, 3.1)
    # p.stepSimulation()
    #
    # # getting images
    images = p.getCameraImage(112, 112, viewMatrix=view, projectionMatrix=projection)
    # p.stepSimulation()
    rgb = images[2]
    rgb = np.array(rgb)
    # #p.stepSimulation()
    rgb = np.reshape(rgb, (112, 112, 4))
    rgb = np.uint8(rgb)
    # p.stepSimulation()
    image = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    # cv2.imshow("image0", image)
    # cv2.waitKey(0)
    # p.stepSimulation()
    centrec = getcentre(image)
    # p.stepSimulation()
    # #
    centrepoints.append(centrec)  # append all centres in a list
    timestamps.append(time.time())
    # time at which the centre position was recorded appended into this list
    if i % 10 == 0 and i != 0 and count <= 1:
        count = count + 1
        # print(centrec)
        dx = centrepoints[i - 10][0] - centrepoints[i][
            0]  # difference in x-coord of the two centre positions at i and i-2
        dy = centrepoints[i - 10][1] - centrepoints[i][
            1]  # difference in y-coord of the two centre positions at i and i-2
        distance = math.sqrt(abs(dx) * abs(dx) + abs(dy) * abs(dy))  # distance btw the two centres
        # print(distance)
        velocity = (distance / abs(
            timestamps[i] - timestamps[i - 10])) / pixelpermetric  # speed calc using speed=distance/time
        print("Calculated velocity",velocity)
        angle = math.degrees(math.atan2(dy, dx))  # angle from the x-axis
        print("angle =", angle)
        velac = p.getBaseVelocity(puck)
        abs_vel = math.sqrt((velac[0][0]**2)+(velac[0][1]**2))
        print("absolute speed", abs_vel)

    # time.sleep(1)

p.disconnect()
