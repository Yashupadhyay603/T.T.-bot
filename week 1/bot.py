# importing libraries
import pybullet as p
import pybullet_data
import time
import cv2

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
puck_pos=[-0.2,0,0.5]
puck_ori=p.getQuaternionFromEuler([0,0,0])
puck=p.loadURDF("puck(1).urdf",puck_pos,puck_ori)

# setting kuka bot
kuka=p.loadURDF("model.urdf",[-1.0,0,0],p.getQuaternionFromEuler([0,0,0]))

# creating constraints for table
p.createConstraint(plane,-1,table,-1, p.JOINT_FIXED,[0,0,0],table_pos,[0,0,0])


for i in range(10000):
    p.stepSimulation()

    # setting camera
    view = p.computeViewMatrix([0,0,1.4],[0,0,0.5],[0,1,0])
    projection= p.computeProjectionMatrixFOV(90,1,0.1,3.1)

    # getting images
    images=p.getCameraImage(512,512,viewMatrix=view,projectionMatrix=projection)
    image=images[2]
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    cv2.imshow("image",gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    time.sleep(1./240.)

p.disconnect()
