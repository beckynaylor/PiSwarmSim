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

#Robot agent class, provide the pygame world and room 
#specify robot diameter and IR sensor range in cm 
#Provide current ticklength (fraction of a second) and the unique ID of this robot

#import external libraries
import os, random, sys, math, itertools
from framework import *

#import simulator classes
from proxSensor import *
from arena import *

class Robot:
    def __init__(self, world, room, diameter, IRSensRange, ticklength, robotid, logfiles=False, logpath='', paramDict={}):
        
        #Use room values to inform the x and y pos
        roomx = room.xsize
        roomy = room.ysize
        self.ticklen = ticklength
        self.logfiles = logfiles
        
        #Robot physical properties
        self.robotid = robotid #Set in hardware ID tag
        self.size = diameter
        self.IRSensRange = IRSensRange
        
        #Robot movement properties
        self.currentSpeed = 0
        self.topSpeed = 3
        
        self.heading = (0,0)        #Turn to coordinate
        self.headingAngle = 0       #Turn to this angle (in radians)
        self.prevpos = (0,0)        #to store vals from previous timestep - allows us to check if a robot is stuck
        self.prevheading = 0
        self.generalheadingAngle = 0 #This is the overall angle (in radians) to direct the robot e.g. to power, above is used for local interactions eg flocking
        self.headingAngleAchieved = True #Flag to tell robot to turn
        self.relativeToRobot = True     #Flag to state whether the headingAngle is relative to the robot (true) or the world (false)
        
        self.timer = 0
        self.waittime = random.randint(240,2400) #Timesteps to wait before carrying out an action
        self.stucktimer = 0         #Number of timesteps there has been no movement
        self.freedtimer = 0         #Number of timesteps there has been movement
        
        self.reverse = False
        self.turning = False
        self.direction = "straight" #Control robot direction while driving, can be 'straight', 'left' or 'right'
        self.contactObs = False
        self.stuck = False
        
        self.task = "explore" #The current robot task, will either be recharge or explore
        
        #Robot power properties
        self.fullBattery = 2400     #total battery capacity in coulombs - 2400 lasts for 2hrs 
        
        if 'startBatteryLevel' in paramDict:
            self.batterylevel = (self.fullBattery/100)*paramDict['startBatteryLevel']
        else:    
            self.batterylevel = self.fullBattery
        #self.batterylevel = random.randint(600,1200)
        self.rechargeThresh = 40     #Threshold as percentage - must be lower than exploreThresh
        self.exploreThresh = 70    #Threshold as percentage - must be higher than rechargeThresh
        self.recharging = False
        self.chargeRate = 0.5
        self.useRate = 0.33333
        self.totalUsedPower = 0
        self.totalDrawnPower = 0
        
        self.neighbours = []    #Robots within IR sensor range
        
        if logfiles == True:
            self.logfile = open(logpath + '/robotdata' + str(self.robotid) + '.log', 'w')
            self.logfile.write('timestep, position, heading, state, battery level out of ' + str(self.fullBattery) + 'neighbours\n')
        
        (xpos, ypos) = (random.uniform(-(float(roomx)/2)+(diameter/2),(float(roomx)/2)-(diameter/2)), random.uniform(0+(diameter/2),roomy-(diameter/2)))
        #Choose heading at random
        start_heading = math.radians(random.randint(0,359))
                
        #Construct body in the world
        robotshape=b2CircleShape(radius=(diameter/2))
        self.fixtures = b2FixtureDef(shape=robotshape, density=1, friction=0.3, userData=self) #pass the robot pointer to both fixture and body for use in collisions
        self.body = world.CreateDynamicBody(position=(xpos,ypos), angle=start_heading, fixtures=self.fixtures, linearDamping=5, angularDamping=5, userData=self)
    
        #Set up IR sensors at correct positions round the robot
        IRsensID = 0
        IRposList = [15, 50, 90, 154, 206, 270, 310, 345]
        self.IRSensList = []
        for IRpos in IRposList:
            IRsens = ProxSensor(IRsensID, diameter/2, self.IRSensRange, 30, IRpos, self.body.transform)
            self.IRSensList.append(IRsens)
            self.body.CreateFixture(IRsens.sensorfixture)
            IRsensID += 1 
    
	#Move robot in correct direction
    def drive(self, clock):
        
        if self.logfiles == True:
            outputstring = [clock, self.body.position, self.normaliseAngle(self.body.transform.R.angle), self.task, self.batterylevel, self.neighbours, "\n"]
            outputstring = " ".join([str(x) for x in outputstring])
            self.logfile.write(outputstring)
        
        #If the battery still has energy then move
        if self.batterylevel > 0:

            #if self.stuck == True:
            #    print "robot", self.robotid, "turning", self.turning, "direction", self.direction, "heading achieved", self.headingAchieved()

            #Apply a small force to the centre of the robot
            #TODO: Make realistic force for motor to deliver at each time step
            if self.reverse == True:
                self.driveBackward()
            elif self.turning == True:
                
                if self.headingAngleAchieved == False:
                    self.turnToHeading()
                else:
                    #The robot should no longer be stuck if it has achieved its heading
                    if self.stuck == True:
                        self.stuck = False
                    
                    #Turn the robot in correct direction without driving
                    if self.direction == "left":
                        self.turnAntiClockwise()
                    elif self.direction == "right":
                        self.turnClockwise()
                    else:
                        print "error - turn but no direction specified"
                
            else:
                #Turn the robot in correct direction while driving
                if self.direction == "left":
                    self.driveForwardLeft()
                elif self.direction == "right":
                    self.driveForwardRight()
                else:
                    self.driveForward()
        
            #Clamp the velocity vector to the robots top speed
            (xLinVel, yLinVel) = self.body.linearVelocity
            #Calculate the current speed from the linear velocity
            self.currentSpeed = math.sqrt((xLinVel**2)+(yLinVel**2))
            
            #Calculate new vector in same direction, but clipped to top speed
            if self.currentSpeed > self.topSpeed:
                self.body.linearVelocity = (((xLinVel*self.topSpeed)/self.currentSpeed),((yLinVel*self.topSpeed)/self.currentSpeed))
                self.currentSpeed = self.topSpeed
        
            #Reduce battery level
            self.batterylevel -= (self.useRate * self.ticklen)
            self.totalUsedPower += (self.useRate * self.ticklen)
            
            if self.batterylevel < 0:
                self.batterylevel = 0
            
        else:
            #This isn't a good way to do this - world friction should stop movement
            #body.setLinearDamping (ish)
            self.body.linearVelocity = (0,0)
            self.body.angularVelocity = 0
   
        offset = 0.001
        
        #Check whether robot is stuck by testing whether position and heading are different to the last timestep (within a tolerance range)        
        if self.compareHeading(self.prevheading, offset) and (self.prevpos == self.body.position):
            self.stucktimer += 1
            self.freedtimer = 0
        else:
            self.stucktimer = 0
            self.freedtimer += 1
            
        #print self.stucktimer
            
        #Update previous heading and position - must use new b2Vec2 not == self.body.position so it uses value not reference
        self.prevheading = self.normaliseAngle(self.body.transform.R.angle)
        self.prevpos = b2Vec2(self.body.position[0], self.body.position[1])
    
    #Specify a new heading in degrees, relative is a flag to determine whether to turn to orientation within world or compared to current heading   
    def changeHeading(self, desiredHeading, relativeToRobot=True):
        print "change heading to :", desiredHeading
        self.relativeToRobot = relativeToRobot
        self.changeHeadingRad(math.radians(desiredHeading), relativeToRobot)
    
    #Specify a new heading in radians, relative is a flag to determine whether to turn to orientation within world or compared to current heading
    def changeHeadingRad(self, desiredHeading, relativeToRobot=True):
        self.relativeToRobot = relativeToRobot
        self.turning = True
        
        #Get co-ordinates of desired heading
        x = -(math.cos(desiredHeading))
        y = -(math.sin(desiredHeading))
        
        #Set own parameters to the desired heading
        self.heading = b2Mul(self.body.transform.R,(x,y))
        self.headingAngle = self.normaliseAngle(desiredHeading)
        self.headingAngleAchieved = False
        
        #print "turn to angle", math.degrees(self.headingAngle)
        
        self.turnToHeading()
        
    def recharge(self):
        #If the battery is not 100% then charge it
        if self.batterylevel < self.fullBattery:
            self.batterylevel += (self.chargeRate * self.ticklen)
            self.totalDrawnPower += (self.chargeRate * self.ticklen)
    
    def driveForward(self):
        #Equal forward power from both wheels
        self.driveRightWheelForward()
        self.driveLeftWheelForward()
            
    def driveBackward(self):
        #Equal backward power from both wheels
        self.driveRightWheelBackward()
        self.driveLeftWheelBackward()
    
    #Turn while still moving
    def driveForwardLeft(self):
        self.driveRightWheelForward(0)
        self.driveLeftWheelForward(0.25)
        
    def driveForwardRight(self):
        self.driveRightWheelForward(0.25)
        self.driveLeftWheelForward(0)
    
    #See if the current heading matches the specified one 
    def compareHeading(self, angle, offset=2):
        #Get current heading
        heading = self.normaliseAngle(self.body.transform.R.angle)
       
        #convert to radians
        radOffset = math.radians(offset)
        
        lowestPermissable = self.normaliseAngle(angle - radOffset)
        highestPermissable = self.normaliseAngle(angle + radOffset)
        
        #Special case for crossing 0/359 degrees
        if lowestPermissable > highestPermissable:
            
            if (heading >= lowestPermissable) or (heading <= highestPermissable):
                headingMatch = True
            else:
                headingMatch = False
        
        else:
            #Is the heading within 'offset' of the specified angle
            if (lowestPermissable <= heading <= highestPermissable):
                headingMatch = True
            else:
                headingMatch = False
        
        return headingMatch
        
    #Check whether the requested heading is met, if so then set flags to stop turning, if not set flags to keep turning
    #TODO: Could utilise compareHeading above
    def headingAchieved(self):
        
        permittedOffset = 1
        radOffset = math.radians(permittedOffset)
       
        #Get the current robot heading
        if self.relativeToRobot == True:
            heading = b2Mul(self.body.transform.R,(-1.0,0.0))
        else:
            heading = self.normaliseAngle(self.body.transform.R.angle)
        
        desiredHeading = self.heading
        desiredHeadingAngle = self.headingAngle
        lowestPermissable = self.normaliseAngle(desiredHeadingAngle - radOffset)
        highestPermissable = self.normaliseAngle(desiredHeadingAngle + radOffset)

        #Check heading against desired heading
        if (self.relativeToRobot == True) and not((desiredHeading[0] - radOffset) <= heading[0] <= (desiredHeading[0] + radOffset)) \
            and not((desiredHeading[1] - radOffset) <= heading[1] <= (desiredHeading[1] + radOffset)):
            self.headingAngleAchieved = False
        #If relative to the world
        elif (self.relativeToRobot == False):
            #Special case for crossing 0/359 degrees 
            if (lowestPermissable > highestPermissable): 
                if not((heading >= lowestPermissable) or (heading <= highestPermissable)):
                    self.headingAngleAchieved = False
                else:
                    self.headingAngleAchieved = True
            else:
                #Check if the heading is within the allowable range 
                if not(lowestPermissable <= heading <= highestPermissable):
                    self.headingAngleAchieved = False
                else:
                    self.headingAngleAchieved = True
        else:
            self.headingAngleAchieved = True
        
        if self.headingAngleAchieved == True:
            #If it is achieved then clear the heading values
            self.heading = (0,0)
            self.headingAngle = 0
            self.turning = False
            self.direction = "straight"
        
        return self.headingAngleAchieved
    
    #Turn to specified heading 
    def turnToHeading(self):
    
        #Check if we have reached the desired heading - if not continue to turn
        if self.headingAchieved() == False:
            
            #print "heading angle", math.degrees(self.headingAngle)
            
            #Set direction to turn the shortest way
            #When relative to robot, simply >=180 turn right and <180 turn left
            if self.relativeToRobot == True:
                if self.headingAngle >= math.pi or self.headingAngle <= 0:
                    self.direction = "left"
                    self.turnAntiClockwise()  
                else:
                    self.direction = "right"
                    self.turnClockwise()
                    
            #When relative to the world it is the difference between current and desired headings
            else:
                heading = self.normaliseAngle(self.body.transform.R.angle)
                
                if (self.normaliseAngle(self.headingAngle - heading) >= math.pi) or (self.normaliseAngle(self.headingAngle - heading) <= 0):    
                    self.direction = "right"
                    self.turnClockwise()
                else:
                    self.direction = "left"
                    self.turnAntiClockwise()
                    
    #Drive in given direction  
    def driveToHeading(self, headingAngle):
        
        #If the direction within 'argument2' degrees isn't right then adjust heading
        directionCorrect = self.compareHeading(headingAngle, 5)
        
        if directionCorrect == False:
            
            #Set direction to turn the shortest way
            #When relative to robot, simply >=180 turn right and <180 turn left
            if self.relativeToRobot == True:
                if headingAngle >= math.pi or headingAngle <= 0:    
                    self.direction = "right"
                else:
                    self.direction = "left"
                    
            #When relative to the world it is the difference between current and desired headings
            else:
                heading = self.body.transform.R.angle
                
                if (self.normaliseAngle(headingAngle - heading) >= math.pi) or (self.normaliseAngle(headingAngle - heading) <= 0):    
                    self.direction = "right"
                else:
                    self.direction = "left"
        else:
            self.direction = "straight"

    #Go to place in world using GPS - pass in a coordinate to head to, return angle in degrees
    def angleToCoord(self, (x2,y2)):
        #Get current position
        (x1,y1) = self.body.position
        #Calculate the angle of the vector between current position and desired position
        angleToCoord = self.normaliseAngle(math.atan2((y1-y2),(x1-x2)))
        
        return angleToCoord
    
    def turnClockwise(self):
        self.driveRightWheelForward(0.15)
        self.driveLeftWheelBackward(0.15)
        
    def turnAntiClockwise(self):
        self.driveLeftWheelForward(0.15)
        self.driveRightWheelBackward(0.15)
    
    #Functions to control each wheel independently - allowing movement and turning of the robot
    def driveRightWheelForward(self, speed=5):
        #Get robot heading vector from the perspective of the right wheel
        #Right wheel position is (0,-1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(-1.0,-1.0))
        
        #adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        #Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, -1.0))
        self.body.ApplyForce(f, p, True)
        
    def driveLeftWheelForward(self, speed=5):
        #Get robot heading vector from the perspective of the left wheel
        #Left wheel position is (0,1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(-1.0,1.0))
        
        #adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        #Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, 1.0))
        self.body.ApplyForce(f, p, True)
            
    def driveRightWheelBackward(self, speed=1):
        #Get robot reverse heading vector from the perspective of the right wheel
        #Right wheel position is (0,-1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(1.0,-1.0))
        
        #adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        #Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, -1.0))
        self.body.ApplyForce(f, p, True)
        
    def driveLeftWheelBackward(self, speed=1):
        #Get robot reverse heading vector from the perspective of the left wheel
        #Left wheel position is (0,1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(1.0,1.0))
        
        #adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        #Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, 1.0))
        self.body.ApplyForce(f, p, True)
        
    #Return side of robot that the sensor is on
    def sensSide(self, sensorid):
        
        if sensorid <=3:
            side = "right"
        else:
            side = "left"
        
        return side
    
    #Return a list of the sensors that are not in contact with an obstacle
    def freeSides(self):
        nonContactList = [x.proxid for x in self.IRSensList if x.contactObs == False] 
        return nonContactList
        
    #Return a list of the sensors that are not in contact with an obstacle
    def contactedSides(self):
        contactList = [x.proxid for x in self.IRSensList if x.contactObs == True] 
        return contactList
        
    #Issues where angle is negative or greater than 2pi radians (to decide whether to turn left or right)
    #Therefore normalise to a value between 0 and 2pi
    def normaliseAngle(self, angle):
        
        if angle < 0:
            angle = (2*math.pi) + angle 
        elif angle > (2*math.pi):
            angle = angle - 2*math.pi
        return angle
    
    #Set general heading towards a paticular world object
    def headToBody(self, worldObject):
        self.generalheadingAngle = self.normaliseAngle(self.angleToCoord(worldObject.body.position))
    
    def goRecharge(self, powerstrips):
        print "Robot", self.robotid, ": I am heading to recharge"
        self.task = "recharge"
        #TODO: head to nearest powersocket not just the first one
        
        #Set the main heading of the robot towards the powersocket
        #self.generalheadingAngle = self.normaliseAngle(self.angleToCoord(powerstrips[0].body.position))
        self.headToBody(powerstrips[0])
            
        #If not turning due to something else (e.g. obstacle) then turn to powerstrip
        if self.turning == False:
            self.changeHeadingRad(self.generalheadingAngle, False)
        
    def explore(self):
        print "I am exploring"
        self.task = "explore"
        self.waittime = random.randint(2400,4800)  

        print self.direction
        print self.headingAngleAchieved

        #If we were heading for the power supply this is no longer important so clear it
        #side note: this will also clear heading if turning from wall in recharge area, but the timer will reinitialise stuck behaviour so it's ok
        if self.headingAngleAchieved == False:
            self.headingAngleAchieved = True
        #Allow robot to start exploring again
        self.turning = False
        self.direction = "straight"
