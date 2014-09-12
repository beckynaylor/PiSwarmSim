#    Pi-Swarm Simulator is a simple graphical simulation environment for the Pi-Swarm robots
#    Copyright (C) 2014 Becky Naylor, Jon Timmis, University of York

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#import external libraries
import os, random, sys, math, itertools
from framework import *

#import simulator classes
from arena import *

class ProxSensor:
        #Each sensor on a robot has a unique ID, a field of view calculated using the radius and sensor apeture and a position in degrees around the edge of the robot
        #For the distance sensing we also need the robot position and rotation (robottransform)
        def __init__(self, sensid, robotRadius, sensorRange, sensApeture, position, robottransform):
            
            #Sensor properties
            self.proxid = sensid
            self.contactIR = False  #If another robot is in this sensor's range then true 
            self.contactObs = False     #If an obstacle is in this sensor's range then true
            self.contactedRobots = []   #List of robots in the sensor's range
            self.commsRangeRobots = []  #Subtle difference from the above list - if another robot's sensor is in range of this sensor then they can communicate
            self.contactedObstacles = [] #List of obstacles in the sensor's range
            self.prev_dist = {}
            
            #The distance is measured using a ray cast along the centre line of the sensor - set this up
            self.sensorCoords = (0,0)
            self.sensorExtentCoords = (0,0)
            self.raylistInternal = []
            self.raylistExternal = []
            
            #Each sensor on a robot has a unique ID, a field of view calculated using the radius and sensor apeture and a position in degrees around the edge of the robot
            self.robotRadius = robotRadius
            self.radpos = math.radians(position)
            self.robottransform = robottransform
            
            #calculate extent of the sensor
            self.radius = robotRadius + sensorRange
            
            #Use 10 points around the edge of the cone, start in the centre of the robot
            #TODO: Move IR sensors start coordinate to edge of robot, not the centre
            num_vertices = 10
            vertices = [(0,0)] 
            
            #Calculate cone of perception - has length set by radius and arc by sensApeture
            for i in range(0,num_vertices+1):
                #i/num_vert calculates current vertex position, the -0.5 centres the cone about the position
                #position rotates the cone to the correct angle
                angle = (((float(i)/num_vertices)-0.5)*(math.radians(sensApeture)) - math.radians(position)) 
                #Get the angles of the two edges of the sensor
                if i == 0:
                    self.leftangle = angle
                if i == num_vertices:
                    self.rightangle = angle
                
                vertices.append((-(self.radius * math.cos(angle)), -(self.radius * math.sin(angle))))

            self.leftcoord = vertices[1]
            self.rightcoord = vertices[num_vertices+1]
            
            #Return fixture with correct polygon shape
            self.sensorshape=b2PolygonShape(vertices=vertices)
            self.sensorfixture = b2FixtureDef(shape=self.sensorshape, isSensor=True, userData=self)
        
        #Notify sensor of the contacted robot on begincontact, pass in robot's fixture
        def contactRobot(self, hitRobot):
            self.contactedRobots.append(hitRobot)
            self.prev_dist[hitRobot.userData.robotid] = -1
        
        #Notify sensor no longer in contact with robot on endcontact, pass in robot's fixture
        def removeContactRobot(self, robot):
            #The value should only be in the list once, but remove all instances for safety
            self.contactedRobots = [x for x in self.contactedRobots if x.userData.robotid != robot.robotid]
            
        #Notify sensor of the contacted obstacle on begincontact, pass in obstacle's fixture
        def contactObstacle(self, hitObstacle):
            self.contactedObstacles.append(hitObstacle)
        
        #Notify sensor no longer in contact with obstacle on endcontact, pass in obstacle's fixture
        def removeContactObstacle(self, obstacle):

            if isinstance(obstacle, Room):
                #Room is a special case
                self.contactedObstacles = [x for x in self.contactedObstacles if not(isinstance(x.userData, Room))]
            else:    
                #As the list contains room and obstacles, we can't use list comprehension to test it (room doesn't have .obstacleid)
                newContactedObs = []
                
                for x in self.contactedObstacles:
                    if isinstance(x.userData, Room):
                        newContactedObs.append(x)
                    else:
                        if x.userData.obstacleid != obstacle.obstacleid:
                            newContactedObs.append(x)
                
                self.contactedObstacles = newContactedObs            
        
        #Initialise the distance monitor while contact is made with the sensor
        def setupDistMonitor(self):
            
            #Clear previous values
            self.rayinputList = []
            self.rayoutputList = []
            self.hitList = []
            
            #Calculate remaining coordinates of the three rays (left, right and centre of sensor)
            self.sensorCoords = ((-self.robotRadius * math.cos(self.radpos), -self.robotRadius * math.sin(self.radpos)))
            leftInCoord = ((-self.robotRadius * math.cos(self.leftangle), -self.robotRadius * math.sin(self.leftangle)))
            rightInCoord = ((-self.robotRadius * math.cos(self.rightangle), -self.robotRadius * math.sin(self.rightangle)))
            self.sensorExtentCoords = ((-self.radius * math.cos(self.radpos), -self.radius * math.sin(self.radpos)))
            
            self.raylistInternal = [leftInCoord, self.sensorCoords ,rightInCoord]
            self.raylistExternal = [self.leftcoord, self.sensorExtentCoords, self.rightcoord]
            
            #Check the distance to each robot the sensor can see - real sensor would just return the closest, but we need to work out which is closest
            for contactedRobot in self.contactedRobots:
        
                hitshape = contactedRobot
        
                rayinput = []
                rayoutput = []
                hit = []
                
                for rayNo in range(0,len(self.raylistInternal)):
           
                    #Rotate the coordinates
                    p1 = b2Mul(self.robottransform.R, self.raylistInternal[rayNo])
                    p2 = b2Mul(self.robottransform.R, self.raylistExternal[rayNo])

                    #Get sensor raytrace positions
                    pos = self.robottransform.position                
                    p1 = (p1[0]+pos[0], p1[1]+pos[1])
                    p2 = (p2[0]+pos[0], p2[1]+pos[1])
            
                    #Construct raycast
                    rayinput.append(b2RayCastInput(p1=p1, p2=p2, maxFraction=1.0))
                    rayoutput.append(b2RayCastOutput())
                    hit.append(hitshape.RayCast(rayoutput[rayNo], rayinput[rayNo], 0))
                
                self.rayinputList.append(rayinput)
                self.rayoutputList.append(rayoutput)
                self.hitList.append(hit)
        
        #Return the distance between the sensor and an object 'hitshape'
        ####NOTE: Please use this function with caution. It works in most situations, but may respond unusually sometimes.
        def calcDist(self):
            
            #print "Num robots in sensor range", len(self.contactedRobots)
            
            #TODO: make this more general - so distance to obstacles can be detected too. 
            
            #Start new raycast from the robot's current position 
            self.setupDistMonitor()
            
            averageDistList = []
            
            #Check the distance to each robot the sensor can see - real sensor would just return the closest, but we need to work out which is closest
            for index in range(0,len(self.contactedRobots)):
            
                contactedRobot = self.contactedRobots[index]
                robotid = contactedRobot.userData.robotid
            
                distlist = []
                #print self.robottransform
            
                #check each of the rays for a hit
                for hitNo in range(0,len(self.hitList[index])):
                
                    #print "ray " + str(hitNo+1) + " " + str(self.hit[hitNo])
                
                    #Check for the point of intersection, this will provide the distance
                    if self.hitList[index][hitNo]:
                        hit_point = self.rayinputList[index][hitNo].p1 + self.rayoutputList[index][hitNo].fraction * (self.rayinputList[index][hitNo].p2 - self.rayinputList[index][hitNo].p1)
                        #print hit_point
            
                        #Calculate distance from sensor to obstacle
                        dist_to_obst = math.sqrt(math.pow(self.rayinputList[index][hitNo].p1[0]-hit_point[0], 2) + math.pow(self.rayinputList[index][hitNo].p1[1]-hit_point[1],2))
                        #print dist_to_obst
                
                        distlist.append(dist_to_obst)
            
                #If only one ray has hit then return that value
                if len(distlist) == 1:
                    averageDist = distlist[0]    
                #If more than one have hit then calculate the average
                elif len(distlist) > 1:
                    distsum = 0
                
                    for dist in distlist:
                        distsum = distsum + dist
                
                    averageDist = distsum/len(distlist)
                #None have hit
                else:
                    #The obstacle ray has not hit
                    #this is either because the robot has left the rays but not the sensor area or because the ray starts within the robot (very close)
                    #Check for overlap or use the previous distance to guess whether the robot is close or far away
                
                    overlapping = b2TestOverlap(self.sensorfixture.shape, 0, contactedRobot.shape, 0, self.robottransform, contactedRobot.body.transform);
                    if overlapping == True:
                        averageDist = 0
                    else:
                    
                        raylen = math.sqrt(math.pow(self.raylistInternal[0][0]-self.raylistExternal[0][0], 2) + math.pow(self.raylistInternal[0][1]-self.raylistExternal[0][1],2))
                
                        #If this is the first call we have no information, assume it is close because this is safest
                        if self.prev_dist[robotid] == -1:
                           averageDist = 0
                        #If the previous distance was small then assume we have hit the start point of the ray
                        elif self.prev_dist[robotid] <= raylen/5:
                            averageDist = 0
                        #If the previous distance was large assume we have moved out of range (set distance to range of sensor)
                        else:
                            averageDist = raylen

                averageDistList.append(averageDist)

            #Update previous distance - use a dictionary rather than index because a robot may have left the sensor range
            self.prev_dist = {}
            
            for index in range(0,len(self.contactedRobots)):
                robot = self.contactedRobots[index]
                self.prev_dist[robot.userData.robotid] = averageDistList[index]
            
            #get the closest distance
            nearestDist = min(averageDistList)
            self.nearestRobot = self.contactedRobots[averageDistList.index(nearestDist)]
            
            return(nearestDist)
