# importing libraries
import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

def getcentre(image):
    center = ()
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 5, 255)
    contours = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]
    for c in contours:
        if(cv2.contourArea(c)<500):
            (x,y),r = cv2.minEnclosingCircle(c)
            r = int(r)
            if r!=0:
                center = (int(x),int(y))
                cv2.circle(image,center,r,(0,255,0),2)
    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return center
    

# setting physics client
PhysicsClent=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# setting gravity
p.setGravity(0,0,-10)

# setting plane
plane=p.loadURDF("plane.urdf")

# setting table
table_pos=[0,0,0]
table_ori=p.getQuaternionFromEuler([0,0,0])
table=p.loadURDF("table(1).urdf",table_pos,table_ori)

# setting puck
puck_pos=[-0.6,0.0,0.3]
puck_ori=p.getQuaternionFromEuler([0,0,0])
puck=p.loadURDF("puck(1).urdf",puck_pos,puck_ori)

# setting kuka bot
kuka=p.loadURDF("model.urdf",[-1.0,0,0],p.getQuaternionFromEuler([0,0,0]))

# creating constraints for table
p.createConstraint(plane,-1,table,-1, p.JOINT_FIXED,[0,0,0],table_pos,[0,0,0])
ALPHA = 100

for i in range(10000):
    p.stepSimulation()

    puck_pos, puck_ori = p.getBasePositionAndOrientation(puck)

    force = ALPHA * (np.array([0, 0, 0.3]) - np.array(puck_pos))
    p.applyExternalForce(objectUniqueId=puck, linkIndex=-1,
                         forceObj=force, posObj=puck_pos, flags=p.WORLD_FRAME)

    # setting camera
    view = p.computeViewMatrix([0,0,1.4],[0,0,0.5],[0,1,0])
    projection= p.computeProjectionMatrixFOV(90,1,0.1,3.1)

    # getting images
    images=p.getCameraImage(512,512,viewMatrix=view,projectionMatrix=projection)
    image=images[2]
    centrec = getcentre(image)
    print(centrec)
    time.sleep(1./1000.)
p.disconnect()
