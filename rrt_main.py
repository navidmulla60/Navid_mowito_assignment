from random import sample
import pygame
from pygame.draw import circle, line
from rrt_base import RRtGraph, RRTMap
from pygame.locals import *
import sys

def main():
    dimensions=(600,1000)
    start=(20,50)
    goal=(810,510)
    obsdim=30           
    obsnum=50           # defining number of obstacle


    pygame.init()
   

    iteration=0

    

    map=RRTMap(start,goal,dimensions,obsdim,obsnum)
    graph=RRtGraph(start,goal,dimensions,obsdim,obsnum)

    obstacles=graph.makeObs()
    map.drawMap(obstacles)

    # while(True):
    #     x,y=graph.sample_envir()
    #     n=graph.number_of_nodes()
    #     graph.add_node(n,x,y)
    #     graph.add_edge(n-1,n)
    #     x1,y1=graph.x[n],graph.y[n]
    #     x2,y2=graph.x[n-1],graph.y[n-1]

    #     if(graph.isFree()):
    #         pygame.draw.circle(map.map, map.Red, (graph.x[n],graph.y[n]), map.nodeRad, map.nodeThickness)
    #         if not graph.crossObstacle(x1,x2,y1,y2):
    #             pygame.draw.line(map.map, map.Blue, (x1,y1), (x2,y2), map.edgeThickness)


        # pygame.display.update()
        # pygame.event.clear()
        # pygame.event.wait(0)

    while(not graph.path_to_goal()):
        if iteration % 10 == 0:
            X, Y, Parent= graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1],Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1],Y[-1]),(X[Parent[-1]],Y[Parent[-1]]), map.edgeThickness)
        else:
            X,Y,Parent=graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1],Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1],Y[-1]),(X[Parent[-1]],Y[Parent[-1]]), map.edgeThickness)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration+=1

    map.drawPath(graph.getpathcoord())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':   
    result=False
    try:                    
        main()
        result=True
    except:
        result=False
    