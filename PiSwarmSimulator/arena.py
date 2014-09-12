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

#All arena element classes are in this file
#import external libraries
import os, random, sys, math, itertools, operator, datetime, re, cProfile
from framework import *

#import simulator classes
from robot import *
from proxSensor import *

#Room perimeter polygon, currently there should be just one of these
class Room():
    def __init__(self, world, xsize, ysize):
        
        self.xsize = xsize
        self.ysize = ysize

        #TODO: make centre relative to the screen, not hardcoded
        #Centre the room in the screen
        self.centrePoint = (0,ysize/2)
        self.walls = world.CreateBody(position=self.centrePoint, userData=self)
        
        #List of corner positions to create edges
        self.corners = [ (-xsize/2,-ysize/2),
                                (-xsize/2,ysize/2),
                                (xsize/2,ysize/2),
                                (xsize/2,-ysize/2),
                                (-xsize/2,-ysize/2) ]
        
        #Make vertices
        self.walls.CreateEdgeChain(self.corners)
        
#Arena obstacles, provide the world to add them to. Can also provide a list of protected areas (of type b2body)
class Obstacle:
    def __init__(self, world, obstacleid, room, protectedAreaList=0):
		
        self.obstacleid =  obstacleid
        self.shape = ""
        
        #Pick random size
        obs_size = random.uniform(0.5,1.5)
		
        #Dice roll to decide object shape
        diceroll = random.randint(0,2)
		
        roomx = room.xsize
        roomy = room.ysize
        
        #square
        if diceroll == 0:
            self.shape = "square"
            obs_y_size = obs_size
            obstacle=b2PolygonShape(box=(obs_size, obs_size))
            self.size = obs_size
        #rectangle
        elif diceroll == 1:
            self.shape = "rectangle"
            #generate y side too for rectangle
            obs_y_size = random.uniform(0.5,1.5)
            obstacle=b2PolygonShape(box=(obs_size, obs_y_size))
            self.size = (obs_size, obs_y_size)
        #circle
        elif diceroll == 2:
            self.shape = "circle"
            obs_size = obs_size*2
            obs_y_size = obs_size
            obstacle=b2CircleShape(radius=(obs_size))
            self.size = obs_size
		
        positionAccepted = False        
        while positionAccepted == False:
        
            #Pick random co-ordinates
            (xpos, ypos) = (random.uniform(-(float(roomx)/2)+obs_size,(float(roomx)/2)-obs_size), random.uniform(0+obs_y_size,roomy-obs_y_size))
        
            self.fixtures = b2FixtureDef(shape=obstacle, density=1, friction=0.3)
            self.body = world.CreateStaticBody(position=(xpos,ypos), fixtures=self.fixtures, userData=self)

            #Check there are no protected areas e.g. powersockets at this point 
            if protectedAreaList != 0:
                positionAccepted = True
                for protArea in protectedAreaList:
                    overlapping = b2TestOverlap(self.fixtures.shape, 0, protArea.fixtures.shape, 0, self.body.transform, protArea.body.transform);
                
                    #If the shape overlaps a protected area then we need to generate new coordinates
                    if overlapping == True:
                        positionAccepted = False
            
            #Destroy old shape before creating a new one
            if positionAccepted == False:
                world.DestroyBody(self.body)
        
#Floor area where the robots recharge, specified size (x,y) and position (x,y)
class PowerStrip:
    def __init__(self, world, powerid, room, position="none", size="none"):
		
        roomx = room.xsize
        roomy = room.ysize
        
        if size == "none":
            size = (1.5,1.5)
        
        if position == "none":
            #position = ((roomx/2)-size[0],roomy-size[1])
            position = (-(roomx/2)+size[0],roomy-size[1])
        
        #with size (3,2) and room (40,40) xpos = -17 and ypos = 2 is bottom left
        #with size (3,2) and room (40,40) xpos = -17 and ypos = 38 is top left
        #with size (3,2) and room (40,40) xpos = 17 and ypos = 38 is top right
		
        self.powerid = powerid
        self.size = size
        
        powerstrip=b2PolygonShape(box=self.size)
        self.fixtures = b2FixtureDef(shape=powerstrip, density=0, friction=0, isSensor=True, userData=self)
        self.body = world.CreateStaticBody(position=position, fixtures=self.fixtures)

#Floor tile of specified size (x,y) and position (x,y)
class FloorTile:
    def __init__(self, world, position, size):
        
        self.contacted = False
        
        #pygame seems to double the expected size, so (4.0,4.0) has 4.0 above centre point and 4.0 below - so halve it.
        size = (size[0]/2,size[1]/2) 
        
        floortile=b2PolygonShape(box=size)
        self.fixtures = b2FixtureDef(shape=floortile, density=0, friction=0, isSensor=True, userData=self)
        self.body = world.CreateStaticBody(position=position, fixtures=self.fixtures)
