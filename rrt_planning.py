#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped,Pose,PoseStamped
from move_base_msgs.msg import MoveBaseAction ,MoveBaseGoal
from actionlib_msgs.msg import GoalID,GoalStatus
from std_msgs.msg import Bool
import random
from nav_msgs.msg import MapMetaData
import matplotlib.pyplot as plt
import numpy as np

class getgoalpoints(object):
    def __init__(self, initial_position):
        self.step=1000
        self.threshold_to_goal=5
        self.goalReached=False
        self.path=[]
        self.parent=[]
        self.initial_position=initial_position
        # self.target_position=target_position
        self.point_data=rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
        self.mapdimension=rospy.Subscriber("/map_metadata",MapMetaData,self.mapCallback)

    def callback(self,points):

        position=points.pose.position
        orientation=points.pose.orientation

        self.target_position= position.x , position.y, orientation.z
        print("Init: "+str(self.initial_position))  
        print("Goal: "+str(self.target_position))
        print("")
        
        
        self.make_path()
        # self.initial_position=self.target_position    #after reaching Goal, make goal location as initial position of robot
        # self.make_path()

    def mapCallback(self, map):
        self.MapW=map.width
        self.MapH=map.height
        # print(self.MapW, self.MapH)

    def make_path(self):
        
        Xinit,Yinit= self.initial_position[0], self.initial_position[1] 

        self.NodeInit=(Xinit,Yinit)

        self.path.append(self.NodeInit)                         # adding robot initial point as starting point of the path
        # self.parent.append(self.NodeInit)

        # self.newNode=self.initial_position
        while(self.goalReached==False):
            self.randomPoint =randomPoints(self)                # searching for nearest randon points
            # print(self.randomPoint)

            num_node_in_tree=len(self.path)
            # print(num_node_in_tree)
            D=[]
            for i in range(0,num_node_in_tree):
                node=self.path[i]
                self.distance=dist_Btw_Points(self,node)
                # print(distance)
                D.append(self.distance)
            minDistace=min(D)                                   # Compares distances from random node to avialable tree nodes and
                                                                # gives min Distaces of random node

            edge_tobe_extend=np.argmin(D)                       # gives index of min Distance 
                                                                # from this i can decide which node to be extened to random node
            # print(minDistace,edge_tobe_extend)

            newPoint=extendEdge(self,edge_tobe_extend)

        #---------------checking newPoint reached goal Or not------------

            (self.Gx2,self.Gy2)=self.target_position[0],self.target_position[1]
            (self.Nx1,self.Ny1)=newPoint[0],newPoint[1]

            Gpx=(float(self.Nx1)-float(self.Gx2))**2
            Gpy=(float(self.Ny1)-float(self.Gy2))**2
            Gdist=(Gpx+Gpy)**(0.5)
            
            # print(dist)
            if Gdist==self.threshold_to_goal or Gdist<self.threshold_to_goal:   
                self.goalReached=True
                print("Inside IF")
                print(self.path)

            self.path.append(newPoint)
            print(self.path)
        print(self.path)



    # print(self.target_position[0])
        # print(self.initial_position[0])
        # print(a)
        # if (self.target_position[0]>self.initial_position[0] and self.target_position[1]>self.initial_position[1] ):
        #     newpointX=self.initial_position[0]+a
        #     newpointY=self.initial_position[1]+a
        #     newPoint.append((newpointX,newpointY))

        # elif(self.target_position[0]<self.initial_position[0] and self.target_position[1]<self.initial_position[1] ):
        #     newpointX=self.initial_position[0]-a
        #     newpointY=self.initial_position[1]-a
        #     newPoint.append((newpointX,newpointY))

        # elif(self.target_position[0]>self.initial_position[0] and self.target_position[1]<self.initial_position[1] ):
        #     newpointX=self.initial_position[0]+a
        #     newpointY=self.initial_position[1]-a
        #     newPoint.append((newpointX,newpointY))
        
        # elif(self.target_position[0]<self.initial_position[0] and self.target_position[1]>self.initial_position[1] ):
        #     newpointX=self.initial_position[0]-a
        #     newpointY=self.initial_position[1]+a
        #     newPoint.append((newpointX,newpointY))

        # else:
        #     print("Unknown")
        # print(newPoint)
    # pass
        

def randomPoints(self):

    radius = 5
    rangeX = (0, self.MapW)
    rangeY = (0, self.MapH)
    qty = 3  # number of random points
    deltas = set()
    for x in range(-radius, radius+1):
        for y in range(-radius, radius+1):   # making points inside defined radius circle 5*5 (AREA OF CIRCLE)
            if x*x + y*y <= radius*radius:   #varifying points inside circle
                deltas.add((x,y))

    randPoints = []
    excluded = set()
    i = 0
    while i<qty:
        x = random.randrange(*rangeX)
        y = random.randrange(*rangeY)
        if (x,y) in excluded: continue
        randPoints.append((x,y))
        i += 1
        excluded.update((x+dx, y+dy) for (dx,dy) in deltas)
    # print (randPoints)   # prints all random points generated
    
    return min(randPoints)  # returning nearest random points



def dist_Btw_Points(self,node):
    # (x1,y1)=self.newNode[0],self.newNode[1]
    (self.x1,self.y1)=node[0],node[1]
    (self.x2,self.y2)=self.randomPoint[0],self.randomPoint[1]
    # print("Printing extracted x y co orinates")
    # print(x1,y1,x2,y2)
    px=(float(self.x1)-float(self.x2))**2
    py=(float(self.y1)-float(self.y2))**2
    dist=(px+py)**(0.5)
    return dist
    

def extendEdge(self,edge_tobe_extend):
    # print("Printing From EDGE")
    # print(self.path[0])

    newPointX=self.x1-(((self.x1-self.x2)/self.distance)*self.step)
    newPointY=self.y1-(((self.y1-self.y2)/self.distance)*self.step)
    newPoint=(newPointX,newPointY)
    

    # print(edge_tobe_extend)                         #tring for RRT* 
    # self.parent.append(self.path[edge_tobe_extend])

    # print(newPoint)
    return newPoint
  
   


def main():
    rospy.init_node("Testing", anonymous=True)
    initial_position=0,0,0
    gp=getgoalpoints(initial_position)
    # print(gp.x  ,gp.y)
    
    try: 
        rospy.spin()
        if rospy.is_shutdown:
                print("----------------------Try once Again-------------------------")
            
                print("")
                print("Dont give up !!!")
    except KeyboardInterrupt:
        print("Something went wrong!!!")

if __name__ == '__main__':
    main()
   
    
    