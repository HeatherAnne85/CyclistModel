# -*- coding: utf-8 -*-
"""
Created on Thu Apr 21 16:16:46 2022

@author: heath
"""

import os, sys
import numpy as np
from math import *
from trafficintelligence import moving
import gc
from shapely.geometry import *

if 'SUMO_HOME' in os.environ:
     tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
     sys.path.append(tools)
else:
     sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "C:/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui.exe"
sumoCmd = [sumoBinary, "-c", "C:/Users/heath/Desktop/material/My Tools/projects/CyclistModel/CyclistModel/data/test_strecke.sumocfg"]
import traci


'''
NOTES
- Each controlled bicycle has given set of interactors (check in each timestep for distance)


'''

class Interacter(object):
    def __init__(self,P,N,ID,Poly):
        self.id = ID
        self.type = traci.vehicle.getTypeID(self.id)
        self.P = P
        self.N = N
        self.Poly = Poly
        
class Bicycle(object):
    def __init__(self, id, G, N, P, L, W, exitP):
        self.id = id
        self.wish = min(max(np.random.normal(loc=5.3, scale=1.4, size=1),3.0),10)
        self.Tv = min(max(np.random.normal(loc=4, scale=1, size=1),3.0),5)
        self.Rv = min(max(np.random.normal(loc=2, scale=0.1, size=1),1),3)
        self.gv = min(max(np.random.normal(loc=1, scale=0.1, size=1),0.97),1.05)
        self.safety = min(max(np.random.normal(loc=0.25, scale=0.1, size=1),0.2),0.3)      
        self.G = G
        self.N = N
        self.P = P
        self.Poly = getPoly(self.N.angle,self.P,traci.vehicle.getLength(self.id),traci.vehicle.getWidth(self.id),traci.vehicle.getTypeID(self.id))
        self.Traj = []
        self.L = L
        self.W = W
        self.t = 0
        self.exitP = exitP
        traci.vehicle.setColor(id, (5,5,5,100) )   

class Interactor_set(object):
    def __init__(self, all_road_users):
        self.all_road_users = all_road_users
        self.distance_matrix()
        
    def distance_matrix(self):
        '''
        Creates a symmetrical matrix of the distances between all road users
        '''
        mat = np.zeros((len(self.all_road_users),len(self.all_road_users)))
        
        for row in range(0,len(self.all_road_users)-1):
            m = self.all_road_users[row]
            P1 = Point(traci.vehicle.getPosition(m)[0],traci.vehicle.getPosition(m)[1])
            for col in range(row+1, len(self.all_road_users)):
                n = self.all_road_users[col]
                print(m,n)
                P2 = Point(traci.vehicle.getPosition(n)[0],traci.vehicle.getPosition(n)[1])
                mat[row,col] = P1.distance(P2) 
                
        self.mat = mat + mat.T - np.diag(np.diag(mat))   
        
        



def getInfo(inters):
    #get position, norm/angle, and polygon for all interacting road users in list inters
    #return dictionary 
    inters_dict = {}
    for veh in inters:
        if veh not in controlled.keys():
            Pos=Point(traci.vehicle.getPosition(veh)[0],traci.vehicle.getPosition(veh)[1])
            N = moving.NormAngle(traci.vehicle.getSpeed(veh),(pi/2)-(traci.vehicle.getAngle(veh)*pi/180)) 
            Poly = getPoly(N.angle,Pos,traci.vehicle.getLength(veh),traci.vehicle.getWidth(veh),traci.vehicle.getTypeID(veh))
            if veh in inters_dict.keys():
                inters_dict[veh].P = Pos
                inters_dict[veh].N = N
                inters_dict[veh].Poly = Poly
            else:
                inters_dict[veh] = Interacter(Pos,N,veh,Poly)

    return inters_dict


def getInfoObs(Ob_list):
    obstacles_dict = {}
    for f in range(len(Ob_list)):
        obstacles_dict[str(f)] = Obstacles('obs',Ob_list[f])
    return obstacles_dict
 
    



class Obstacles(object):
    def __init__(self,ID,Po):
        self.id = ID
        self.N = moving.NormAngle(0.1,0)
        self.Poly = Po.buffer(0.01)
        

def getPoly(A,P,L,W,Type):   
    #returns diamond for Type='bicycle' and rectangle for Type='car'
    Line=LineString([(P.x-L*np.cos(A),P.y-L*np.sin(A)),(P.x,P.y)])
    right=Line.parallel_offset(0.5*W,'right')
    left=Line.parallel_offset(0.5*W,'left')
    return MultiPoint([Line.coords[0],list(right.centroid.coords)[0],list(right.coords)[1],list(left.centroid.coords)[0]]).convex_hull
       

 
    

def findDirection(bike):
    P = bike.G.interpolate(bike.G.project(bike.P)+bike.wish)
    direction = atan2(P.y-bike.P.y,P.x-bike.P.x)
    return direction


def getGuideline(xmin,xmax,ymin,ymax):
    y = min(max(np.random.normal(loc=ymax-0.5, scale=0.3, size=1),ymin+0.4),ymax-0.4)
    return LineString([(xmax,y),(xmin,y)])


def getInters(ID,RUs,controlled,Obstacles):
    inter_list=[]
    ego=controlled[ID]
    check_ego = 0
    if ego.N.norm == 0:
        ego.N = moving.NormAngle(0.1,ego.N.angle)
        check_ego = 1
    for group in [RUs,controlled,Obstacles]:  #
        for name,inter in group.items():  
            if inter.Poly.intersects(ego.P.buffer(10))==True and ego.id!=name: 
                inter_pt = inter.Poly.boundary.interpolate(inter.Poly.boundary.project(ego.P))      
                u_bq = moving.Point(inter_pt.x-ego.P.x,inter_pt.y-ego.P.y)
                if inter.N.norm == 0:
                    inter.N = moving.NormAngle(0.1,inter.N.angle)
                jk = moving.Point.cosine(u_bq,ego.N.getPoint())
                try:
                    theta = acos(jk)
                except:
                    theta = acos(jk+0.005) 
                if theta == 0:
                    theta = -0.0001
                side = moving.Point.cross(ego.N.getPoint(),u_bq)/(u_bq.norm2()*ego.N.norm*sin(theta))
                similarity = moving.Point.cosine(inter.N.getPoint(),ego.N.getPoint())
                ahead = moving.Point.cosine(u_bq,ego.N.getPoint())
                perp = moving.NormAngle(ego.N.norm, ego.N.angle+side*pi/2).getPoint()
                v_comp = ego.N.getPoint().__mul__(1/ego.N.norm)
                a_comp = perp.__mul__(1/perp.norm2())
                parallel_dist = moving.Point.dot(u_bq,ego.N.getPoint())/ego.N.norm
                perp_dist = moving.Point.dot(u_bq,perp)/ego.N.norm
                delta_V = ego.N.norm - inter.N.norm
                if ahead < 0 and inter.id!='obs':
                    continue
                if group == Obstacles:
                    similarity = 0
                inter_list.append([similarity,u_bq.norm2(),inter.id,ahead,u_bq,v_comp,a_comp, parallel_dist, perp_dist, delta_V])
    if check_ego == 1:
        ego.N = moving.NormAngle(0,ego.N.angle)
    return sorted(inter_list, key=lambda x: x[2]) 


def findAction(bike,multi,direction,signal):  
    wish = moving.NormAngle(bike.wish,direction).getPoint()
    acc_sum= moving.Point(0,0)
    acc_obs = moving.Point(0,0)
    Ap = 2
    for interacter in multi:
        similarity,distance,ID,ahead, u_bq, v_comp, a_comp, parallel_dist, perp_dist, delta_V = interacter
        if ID != 'obs':
            D_star = parallel_dist+perp_dist*10+bike.gv*similarity
            acc_sum = acc_sum.__add__(u_bq.__mul__(np.exp(-D_star/bike.Rv))) 
        if ID == 'obs':
            distance = distance - 1
            if distance < 0 or distance < bike.safety:
                d = 1       
            elif distance < bike.safety + bike.W/2:
                ds = bike.safety + bike.W/2
                d = -(distance-ds)/ds
            else:
                d = 0
            acc_obs = acc_obs.__add__(a_comp.__mul__(d))

    acc = wish.__sub__(bike.N.getPoint()).divide(bike.Tv).__sub__(acc_sum.__mul__(Ap)).__sub__(acc_obs.__mul__(Ap))
    
    if bike.N.norm < 0.5:
        nor = moving.NormAngle(0,0).fromPoint(acc)
        nor.angle = nor.angle * bike.N.norm**2        
        acc = nor.getPoint()
    return acc.x, acc.y


Ob_list = [LineString([(-100,59),(80,59)]),
           LineString([(-100,63),(80,63)])]

traci.start(sumoCmd)
clock = 0
step=5

controlled = {}
leaving = []

obstacles = getInfoObs(Ob_list)

sig_pos = moving.Point(-70,60)

while clock < 10000:
    
    inters = getInfo(list(set(traci.vehicle.getIDList())|set(controlled.keys())))   #get information from all not controlled road users
    Matrix = Interactor_set(traci.vehicle.getIDList()) 
    
    for m in [x for x in traci.vehicle.getIDList() if x not in controlled.keys()]:  
        if traci.vehicle.getVehicleClass(m)=='bicycle':
            P=Point(traci.vehicle.getPosition(m)[0],traci.vehicle.getPosition(m)[1])
            if P.distance(Point(75,60)) < 3:
                Guideline = getGuideline(-70,75,60,62)       
                A_b=(pi/2)-(traci.vehicle.getAngle(m)*pi/180)
                N = moving.NormAngle(traci.vehicle.getSpeed(m),A_b) 
                bike = Bicycle(m,Guideline,N,P,traci.vehicle.getLength(m),traci.vehicle.getWidth(m),Point(-65,60))
                controlled[m]=bike
        
    for ID,n in controlled.items():
        
        signal = None
        
        if len(traci.vehicle.getNextTLS(n.id)) > 0:
            if traci.vehicle.getNextTLS(n.id)[0][3] == 'r':
                signal = traci.vehicle.getNextTLS(n.id)[0][2]
        
        if n.P.distance(n.exitP)<4:
            leaving.append(n)
        
        direction = findDirection(n)
        
        multi=getInters(ID,inters,controlled,obstacles)    
            
        a,h=findAction(n,multi,direction,signal)
            
        V = n.N.getPoint() 
        V = moving.Point(V.x+a,V.y+h)
        n.P = Point(n.P.x + V.x/step,n.P.y + V.y/step)
        n.N = moving.NormAngle(0,0).fromPoint(V)
        
        n.Poly = getPoly(n.N.angle,n.P,traci.vehicle.getLength(ID),traci.vehicle.getWidth(ID),traci.vehicle.getTypeID(ID))
        
        traci.vehicle.moveToXY(n.id, '', 0, n.P.x, n.P.y, (pi/2-n.N.angle)*180/pi, keepRoute=2)
        traci.vehicle.setSpeed(n.id,n.N.norm)   
        
    for leaver in leaving[:]:
        try:
            del controlled[leaver.id]
            traci.vehicle.setColor(leaver.id, (5,5,255,100) )
        except:
            continue
      
    traci.simulationStep()
    clock+= 1
    gc.collect()

traci.close()
