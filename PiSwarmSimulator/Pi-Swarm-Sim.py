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
import os, random, sys, math, itertools, operator, datetime, re, cProfile, ast, shutil
from framework import *

#import simulator classes
from robot import *
from proxSensor import *
from arena import *

#Initialise and run the simulation
class runSim(Framework):
    name="Pi-Swarm Simulator"
    
    def __init__(self):
        super(runSim, self).__init__()

        if self.settings.experiment == True:
            expfile = 'experiment_settings.txt'
            paramFile = open(expfile, 'r')
            expName = paramFile.readline().strip()
            print expName
            dictStr = paramFile.readline()
            print dictStr
            paramFile.close()
            parameterList = ast.literal_eval(dictStr)
        else:
            expName = ''
        
        #Seed rng, use system time converted to int so it can easily be stored and rerun
        self.starttime = datetime.datetime.now()
        timestamp = str(self.starttime)
        seed = re.sub("\D", "", timestamp)
        
        #Alternatively use the same seed so obstacles, robots and headings are the same
        #seed = 8
        
        random.seed(seed)
        
        #Construct log files and directory
        if self.settings.logfiles == True:
            self.path = 'log-files-' + expName
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            
            settingsDict = {'seed': seed}
            settingslogfile = open(self.path + '/settings.log', 'w')
            self.simulationlogfile = open(self.path + '/simulation.log', 'w')
            self.simulationlogfile.write("timestep, number of unexplored tiles, % unexplored, total power use \n")
            
            if self.settings.experiment == True:
                shutil.copyfile(expfile, self.path + '/' + expfile)
            
        else:
            self.path = ''
        
        #These are used for debugging
        self.settopause = False
        self.begins = 0
        self.ends = 0
                
        #Define simulation timing
        self.ticklength = 0.25 #Proportion of a second that each timestep is
        self.clock = 0
        
        #Define world parameters
        self.world.gravity = (0.0, 0.0)
        self.unitsize = 10.0 #Number of cm one simulation unit represents
        
        #list to allow robots to print when they run out of charge
        self.finishedlist = []
    
        #Define simulation values - these can be altered to suit your experiment
        num_robots = 10
        num_obstacles = 50
        room_x_size = 800 #size in cm
        room_y_size = 600
        numXTiles = 40
        numYTiles = 40
        self.powerlimit = 48000
        
        #Find additional values
        self.numFloortiles = numXTiles* numYTiles
        if self.settings.experiment == True:
            num_robots = len(parameterList)

        #These match physical hardware settings
        robot_size = 9.5 #size in cm
        IRSensRange = 10
        
        if self.settings.logfiles == True:
            settingsDict['ticklength'] = self.ticklength
            settingsDict['unitsize'] = self.unitsize
            settingsDict['num_robots'] = num_robots
            settingsDict['robot_size'] = robot_size
            settingsDict['IRSensRange'] = IRSensRange
            
            settingsDict['room_x_size'] = room_x_size
            settingsDict['room_y_size'] = room_y_size
            settingsDict['numXTiles'] = numXTiles
            settingsDict['numYTiles'] = numYTiles
            
            settingsDict['powerlimit'] = self.powerlimit
            
        self.theroom = Room(self.world, self.calcSimSize(room_x_size), self.calcSimSize(room_y_size))
        
        print self.calcSimSize(robot_size)
        
        #Generate robots
        self.robotlist = []
        for x in range(num_robots):
            #Need to stop them being placed overlapping
            if self.settings.experiment == False:
                currentRobot = Robot(self.world, self.theroom, self.calcSimSize(robot_size), self.calcSimSize(IRSensRange), self.ticklength, x, self.settings.logfiles, self.path)
            else:
                currentRobot = Robot(self.world, self.theroom, self.calcSimSize(robot_size), self.calcSimSize(IRSensRange), self.ticklength, x, self.settings.logfiles, self.path, parameterList[x])
            self.robotlist.append(currentRobot)
		
        #Generate powerstrips	
        self.powerstripslist = []
        powerstripSettings = []
        for x in range(1):
            currentPowerstrip = PowerStrip(self.world, x, self.theroom)
            self.powerstripslist.append(currentPowerstrip)   
    
            #construct dictionary for logfile
            if self.settings.logfiles == True:
                #You could use the values passed in above, but returning vals from the powerstrip provides a sanity check that the values are right
                powerstipDict = {}
                powerstipDict['id'] = currentPowerstrip.powerid
                powerstipDict['size'] = currentPowerstrip.size
                powerstipDict['position'] = currentPowerstrip.body.position 
                powerstripSettings.append(powerstipDict)
        
        #Generate obstacles
        self.obstaclelist = []
        obstacleSettings = []
        for x in range(num_obstacles):
            currentObstacle = Obstacle(self.world, x, self.theroom, self.powerstripslist)
            self.obstaclelist.append(currentObstacle)
            
            #construct dictionary for logfile
            if self.settings.logfiles == True:
                obstacleDict = {}
                obstacleDict['id'] = currentObstacle.obstacleid
                obstacleDict['shape'] = currentObstacle.shape
                obstacleDict['size'] = currentObstacle.size
                obstacleDict['position'] = currentObstacle.body.position 
                obstacleSettings.append(obstacleDict)
            
        #Generate floor area
        self.floortilelist = []
        #Determine the size of each tile using the number of tiles and room size
        xsize = self.theroom.xsize/numXTiles
        ysize = self.theroom.ysize/numYTiles        
        
        #Construct all of the tiles
        for currentx in range(0, numXTiles):
            for currenty in range(0, numYTiles):
                xpos = ((xsize/2) -(self.theroom.xsize/2)) +  (currentx*xsize)
                ypos = (ysize/2) + (currenty*ysize)
                currentTile = FloorTile(self.world, (xpos, ypos), (xsize, ysize))
                self.floortilelist.append(currentTile)
        
        #Use a different seed so the random walk is different each time - write this to file so it can be replicated 
        timestamp2 = str(datetime.datetime.now())
        inttime2 = re.sub("\D", "", timestamp2)
        random.seed(inttime2)
                
        #add to dictionary and write out logfile
        if self.settings.logfiles == True:
            settingsDict['powerstrips'] = powerstripSettings
            settingsDict['obstacles'] = obstacleSettings
            settingsDict['seed2'] = inttime2
            settingslogfile.write(str(settingsDict))
            settingslogfile.close()
    
    #Carry out these actions at each timestep    
    def Step(self, settings):
        super(runSim, self).Step(settings)
		
        self.Print("Time: %f s" % (self.ticklength*self.clock), (255,255,255))
        
        #Update power statistics
        totalusedpower = 0
        totaldrawnpower = 0
        for therobot in self.robotlist:
            totalusedpower += therobot.totalUsedPower
            totaldrawnpower += therobot.totalDrawnPower
        
        self.Print("Remaining power: %f s" % (self.powerlimit - totaldrawnpower), (255,255,255))
         
        #Output to log files   
        if self.settings.logfiles:
            percentageUnexplored = (float(len(self.floortilelist))/self.numFloortiles)*100
            #Need total energy used too
            outputstring = [self.clock, len(self.floortilelist), percentageUnexplored, totalusedpower, "\n"]
            outputstring = " ".join([str(x) for x in outputstring])
            self.simulationlogfile.write(outputstring)
        
        #Update robot behaviour
        for therobot in self.robotlist:
            
            if (therobot.batterylevel <= 0) and not(therobot.robotid in self.finishedlist) :
                endstr = ("Robot: " +  str(therobot.robotid) + " ran out of charge at Time: " + str(self.ticklength*self.clock) + " s")
                print endstr            
                self.finishedlist.append(therobot.robotid)
                if self.settings.logfiles == True:
                    finishfile = open(self.path + "/finish-times.log", "a")
                    finishfile.write(endstr + "\n")
                    finishfile.close()
            
            #If the battery falls below a given level and the current state is not recharge then set to recharge
            if therobot.batterylevel <= (therobot.rechargeThresh *(therobot.fullBattery/100)) and therobot.task != "recharge":
                therobot.goRecharge(self.powerstripslist)
                #self.settopause = True
                
            #After a randomised time change the heading of the robots
            if therobot.task == "explore":
                
                #Check if the robot is stuck for 120 seconds
                if (therobot.stucktimer >= 120/self.ticklength):
                    print "Robot", therobot.robotid, "is stuck!"
                    therobot.stuck = True
                    #TODO: need to loop at 0, 7
                    #find the best side to turn to - currently only consecutive 
                    most_free_side = []

                    freeSides = therobot.freeSides()
                    #Make sure the list is in order                    
                    freeSides.sort()
                    
                    for k, group in itertools.groupby(enumerate(freeSides), lambda (i,x):i-x):
                        consecutive_group = map(operator.itemgetter(1), group)
                        if len(consecutive_group) > len(most_free_side):
                            most_free_side = consecutive_group

                    mid_index = int(math.floor(float(len(most_free_side))/2))
                    
                    #If there are no free sides robot is completely blocked, can't do anything
                    if len(most_free_side) != 0:
                    
                        sensorid = most_free_side[mid_index]
                        print "turn to sensor", sensorid
                        #Otherwise tell robot to turn to the middle sensor
                        print math.degrees(therobot.IRSensList[sensorid].radpos)
                        therobot.changeHeadingRad(therobot.IRSensList[sensorid].radpos)
                
                #The robot is no longer stuck, but has not achieved the 'desired heading' while turning (so stuck flag is still on)
                if (therobot.freedtimer >= 120/self.ticklength):
                    therobot.stuck = False
                
                if (therobot.timer == therobot.waittime):
                    #TODO: Don't change while doing wall avoidance
                    therobot.changeHeading(random.randint(0,359))
                
                    #Set new wait time and reset timer
                    therobot.waittime = random.randint(2400,4800)
                    therobot.timer = 0
            #Check the robot is still heading towards the powersocket if it hasn't run out of power
            elif therobot.task == "recharge" and therobot.batterylevel > 0 :
                #If the robot isn't turning (e.g. because of an obstacle) 
                if therobot.turning == False and therobot.timer >= therobot.waittime:
                    
                    #Update the general heading
                    #TODO: Update to nearest powerstrip
                    therobot.headToBody(self.powerstripslist[0])
                    
                    #print "general angle", therobot.generalheadingAngle
                    #We wish to head to world coordinates
                    therobot.relativeToRobot = False
                    therobot.driveToHeading(therobot.generalheadingAngle)
            
            #Check the distance to set the behaviour 
            for sensor in therobot.IRSensList:
                if sensor.contactIR == True:
                    distToRobot = self.calccmSize(sensor.calcDist())
                    
                    ####This section is used to test the output of sensor.calcDist()
                    #As stated this function should be checked before use, because it is not fully tested and may return unexpected results in certain situations
                    #if distToRobot == -10.0 or distToRobot == 0:
                        #self.settings.pause = True
                        #self.settopause = True
                        
                        #print "robot " + str(therobot.robotid) + " sensor " + str(sensor.proxid) + " dist is " + str(distToRobot)
                        
                        #for rayNo in range(0,len(sensor.raylistInternal)):
                    
                            #Rotate the coordinates
                        #    p1 = b2Mul(therobot.body.transform.R, sensor.raylistInternal[rayNo])
                        #    p2 = b2Mul(therobot.body.transform.R, sensor.raylistExternal[rayNo])
                            
                            #Get sensor raytrace positions
                        #    pos = therobot.body.position                
                        #    p1 = (p1[0]+pos[0], p1[1]+pos[1])  # p1+pos ??
                        #    p2 = (p2[0]+pos[0], p2[1]+pos[1])
                
                            #Convert the positions to screen co-ords
                        #    p1 = self.renderer.to_screen(p1)
                        #    p2 = self.renderer.to_screen(p2)
                            
                        #    print (p1, p2)
                    
                    #if self.settings.pause == False:
                        #print "robot " + str(therobot.robotid) + " sensor " + str(sensor.proxid) + " dist is " + str(distToRobot)
                    
                    self.setFlockingBehaviour(distToRobot, sensor, therobot, sensor.nearestRobot)
        
            #Drive robots
            therobot.drive(self.clock)
            
            #Check whether to recharge
            if therobot.recharging == True:
                therobot.recharge()
                #If the robot is in the recharge state and has recharged sufficiently then set back to explore
                if therobot.task == "recharge" and therobot.batterylevel >= therobot.exploreThresh*(therobot.fullBattery/100):
                    therobot.explore()
            
            #Inc time to next heading change
            therobot.timer += 1
        
        #End the simulation when all robots have run out of charge
        #Or when the powersources run out
        #TODO: this is currently shared between all powersources, make it a property of each source
        if (len(self.finishedlist) == len(self.robotlist)) or (totaldrawnpower >= self.powerlimit):
            print "total duration", (datetime.datetime.now()-self.starttime)
            self.QuitPygame()
        
        #Delete unnecessary floortiles (ones that have been contacted already)
        for floortile in self.floortilelist:
            if floortile.contacted == True:
                self.world.DestroyBody(floortile.body)
                self.floortilelist.remove(floortile)
                 
        self.clock += 1

    def Draw(self):
        super(runSim, self).Draw()
        #Update the texture polygons of each 'body' element 
        
        #It is important that the order is correct - e.g. draw robots on top of powerstrips
        #Colour the floortiles depending on contact        
        self.colourShapes(self.floortilelist, b2Color(0.5,0.5,0.5))
                
        #Colour the powerstrips red        
        self.colourShapes(self.powerstripslist, b2Color(1.0,0.0,0.0))
        #Colour the obstacles white        
        self.colourShapes(self.obstaclelist, b2Color(1.5,1.5,1.5))
        
        #Colour robot sensors
        for robot in self.robotlist:
            sum_robots = 0
            on_sensors = []
            
            for sensor in robot.IRSensList: 
                
                sum_robots += len(sensor.contactedRobots)
                
                shape = sensor.sensorshape
                if sensor.contactIR == True:
                    self.colourPolygon(robot.body.transform, shape, b2Color(1.2,1.2,1.2))
                    on_sensors.append(sensor.proxid)
                elif sensor.contactObs == True:
                    self.colourPolygon(robot.body.transform, shape, b2Color(1.2,0,0))
                else:
                    self.colourPolygon(robot.body.transform, shape, b2Color(1.0,0.5,1.2)) 
                    
            #if sum_robots > 0:
            #    print "robot ", robot.robotid, " has " , sum_robots, " in contact  ", on_sensors
        
        #Colour the robots blue in explore and orange in recharge state       
        self.colourShapes([x for x in self.robotlist if x.task ==  "explore"], b2Color(0.0,1.5,1.5))
        self.colourShapes([x for x in self.robotlist if x.task ==  "recharge"], b2Color(1.5,1.0,0.0))
        self.colourShapes([x for x in self.robotlist if x.stuck ==  True], b2Color(1.5,1.5,1.5))
        
        #Print robot IDs
        for  robot in self.robotlist:
            #Get the position of the robots
            (xpos, ypos) = robot.body.position
            #Convert the position to screen co-ords
            (xpos, ypos) = self.renderer.to_screen((xpos, ypos))

            self.DrawStringAt(xpos-1, ypos-1, str(robot.robotid), color=(0.0,0.0,0.0))
            #Print battery level graphic
            self.DrawStringAt(xpos+5, ypos+5, str(round((robot.batterylevel/robot.fullBattery)*100, 1)), color=(255.0,255.0,255.0))
        
        #For debugging print the raytrace positions
#        for robot in self.robotlist:
#            for sens in robot.IRSensList:

#                if sens.proxid == 0 or sens.proxid == 7 or sens.proxid == 6 or sens.proxid == 1:
                    
#                    if len(sens.raylistInternal) != 3 and len(sens.raylistInternal) != 0:
#                        print "Something is wrong, there are", len(sens.raylistInternal), "rays"
                    
#                    for rayNo in range(0,len(sens.raylistInternal)):
                    
                        #Rotate the coordinates
#                        p1 = b2Mul(robot.body.transform.R, sens.raylistInternal[rayNo])
#                        p2 = b2Mul(robot.body.transform.R, sens.raylistExternal[rayNo])

                        #Get sensor raytrace positions
#                        pos = robot.body.position                
#                        p1 = (p1[0]+pos[0], p1[1]+pos[1])  # p1+pos ??
#                        p2 = (p2[0]+pos[0], p2[1]+pos[1])
                
                        #Convert the positions to screen co-ords
#                        p1 = self.renderer.to_screen(p1)
#                        p2 = self.renderer.to_screen(p2)
                
#                        self.renderer.DrawSegment(p1, p2, b2Color(1.0,0.0,0.0))
        
            #Draw a line between the robot and power source
#            self.renderer.DrawSegment(self.renderer.to_screen(robot.body.position), self.renderer.to_screen(self.powerstripslist[0].body.position), b2Color(1.0,0.0,0.0))
                        
        #redraw the room
        centre = self.theroom.centrePoint 
        room_vertices = [self.renderer.to_screen((centre[0]+x[0], centre[1]+x[1])) for x in self.theroom.corners]
        self.renderer.DrawPolygon(room_vertices,b2Color(1.0,1.0,1.0) )
        
        if self.settopause == True:    
            self.settings.pause = True

    #Add colours to simulation objects
    def colourShapes(self, object_array, colour):
        
        #If a single object is provided then make it a list
        if not(isinstance(object_array, list)):
            object_array = [object_array]    
        
        for shapeToColour in (object_array):
        
            #Get the shape position from body
            shape = shapeToColour.fixtures.shape
        
            #If the shape is a polygon use polygon functions
            if isinstance(shape, b2PolygonShape):
                self.colourPolygon(shapeToColour.body.transform, shape.vertices, colour) 
        
            #If the shape is a circle use circle functions
            if isinstance(shape, b2CircleShape):
                
                #Offset by xpos and ypos
                position_vect = shapeToColour.body.position
                
                transform = shapeToColour.body.transform
                
                #Calculate the axis line on the robot (this is the method used in b2world.cpp)
                axis = b2Mul(transform.R, (1.0,0.0))
                
                #Convert the position to screen co-ords
                position_vect = self.renderer.to_screen(position_vect)
            
                #Draw circle
                self.renderer.DrawSolidCircle(position_vect, shape.radius, axis, colour)
    
    #Draw solid polygon of given shape and colour at (x,y) shape pos            
    def colourPolygon(self, shapepos, vertices, colour):
        #Offset by xpos and ypos and angle
        position_vect = [(shapepos*v) for v in vertices]
            
        #Convert the position to screen co-ords
        position_vect = map(self.renderer.to_screen, position_vect)
            
        #Draw polygon
        self.renderer.DrawSolidPolygon(position_vect, colour)

#TODO: make robot have to be inside area not just touching it
#Check for contact between fixtures
    def BeginContact(self, contact):
        super(runSim, self).BeginContact(contact)
        
        self.begins += 1
        #print "begin contacts ", self.begins
        
        #Get the two objects that have collided
        fixtureA = contact.fixtureA.userData
        fixtureB = contact.fixtureB.userData
        bodyA = contact.fixtureA.body.userData 
        bodyB = contact.fixtureB.body.userData
        
        #If a robot enters the powerstrip then recharge it
        if (isinstance(fixtureA, Robot) and isinstance(fixtureB, PowerStrip)):
            bodyA.recharging = True
            print "Robot " + str(bodyA.robotid) + ": Beep boop. I am recharging."
            
        if isinstance(fixtureB, Robot) and isinstance(fixtureA, PowerStrip):
            bodyB.recharging = True
            print "Robot " + str(bodyB.robotid) + ": Beep boop. I am recharging."
        
        #If a robot senses over a floor area mark the area as contacted   
        if (isinstance(fixtureA, ProxSensor) and isinstance(fixtureB, FloorTile)):
            fixtureB.contacted = True
                    
        if (isinstance(fixtureB, ProxSensor) and isinstance(fixtureA, FloorTile)):
            fixtureA.contacted = True
        
        #If a robot is heading for a wall or an obstacle then turn around 
        if isinstance(fixtureA, ProxSensor) and (isinstance(bodyB, Obstacle) or isinstance(bodyB, Room)):
            fixtureA.contactObs = True
            #Check the front sensors. 
            if (fixtureA.proxid == 0 or fixtureA.proxid == 1 or fixtureA.proxid == 6 or fixtureA.proxid == 7):
                
                #Inform sensor of contact 
                fixtureA.contactObstacle(contact.fixtureB.body) 
                
                #If the right sensor contacts the wall then turn left, otherwise right (also stop robot keeping switching direction)
                if (fixtureA.proxid == 6 or fixtureA.proxid == 7) and bodyA.direction != "left":
                    bodyA.direction = "right"
                    #print "go right"
                if (fixtureA.proxid == 0 or fixtureA.proxid == 1) and bodyA.direction != "right":
                    bodyA.direction = "left"
                    #print "go left"
                
                #print "eek an obstacle, retreat"    
                bodyA.turning = True
                bodyA.contactObs = True

                #TODO: make the distance monitoring work for obstacles too.
                
        elif isinstance(fixtureB, ProxSensor) and (isinstance(bodyA, Obstacle) or isinstance(bodyA, Room)):
            fixtureB.contactObs = True
            
            #Inform sensor of contact 
            fixtureB.contactObstacle(contact.fixtureA.body) 
            
            #Check the front sensors
            if (fixtureB.proxid == 0 or fixtureB.proxid == 1 or fixtureB.proxid == 6 or fixtureB.proxid == 7):
                #If the right sensor contacts the wall then turn left, otherwise right (also stop robot keeping switching direction)
                if (fixtureB.proxid == 6 or fixtureB.proxid == 7) and bodyB.direction != "left":
                    bodyB.direction = "right"
                    #print "go right"
                if (fixtureB.proxid == 0 or fixtureB.proxid == 1) and bodyB.direction != "right":
                    bodyB.direction = "left"
                    #print "go left"
                #print "eek an obstacle, retreat"
                bodyB.turning = True
                bodyB.contactObs = True
                
                #TODO: make the distance monitoring work for obstacles too.
        
        #TODO: communications                
        #If two robots' sensors intersect then they are in communication range
        if isinstance(fixtureA, ProxSensor) and isinstance(fixtureB, ProxSensor):
            #Add each to neighbours list for the other - if not already there
            if not( bodyB.robotid in bodyA.neighbours):
                bodyA.neighbours.append(bodyB.robotid)
                fixtureA.commsRangeRobots.append(bodyB.robotid) #Need to know for sensor too, when removing on end contact
            if not( bodyA.robotid in bodyB.neighbours):
                bodyB.neighbours.append(bodyA.robotid)
                fixtureB.commsRangeRobots.append(bodyA.robotid) #Need to know for sensor too, when removing on end contact
        
        #If a robot enters another robot's sensor range then calculate distance
        if (isinstance(fixtureA, Robot) and isinstance(fixtureB, ProxSensor)) or (isinstance(fixtureA, ProxSensor) and isinstance(fixtureB, Robot)):
            
            #When fixtureA is the robot doing the sensing
            if (isinstance(fixtureA, ProxSensor)):
                #Prioritise the wall avoidance 
                if bodyA.contactObs == False:
                
                    #Don't check back sensors
                    if fixtureA.proxid != 4 and fixtureA.proxid != 3 and fixtureA.proxid != 2 and fixtureA.proxid != 5:
                        
                        #Inform sensor of contact 
                        fixtureA.contactRobot(contact.fixtureB)      
                
                        #Check the distance to set the behaviour 
                        distA = self.calccmSize(fixtureA.calcDist())
                        #print "Begin: robot", str(bodyA.robotid), "sensor", fixtureA.proxid, "senses", str(fixtureB.robotid), "dist", str(distA)
                        
                        #Set the contact flag
                        fixtureA.contactIR = True
                
                        self.setFlockingBehaviour(distA, fixtureA, bodyA, fixtureA.nearestRobot)
            
            #When fixtureB is the robot doing the sensing
            if (isinstance(fixtureB, ProxSensor)):
            
                #Prioritise the wall avoidance 
                if bodyB.contactObs == False:
                
                    #Don't check back sensors
                    if fixtureB.proxid != 4 and fixtureB.proxid != 3 and fixtureB.proxid != 2 and fixtureB.proxid != 5:
                        
                        #Inform sensor of contact 
                        fixtureB.contactRobot(contact.fixtureA)
                        
                        #Check the distance to set the behaviour 
                        distB = self.calccmSize(fixtureB.calcDist())
                        #print "Begin: robot", str(bodyB.robotid), "sensor", fixtureB.proxid, "senses", str(fixtureA.robotid), "dist", str(distB)
                        
                        #Set the contact flag
                        fixtureB.contactIR = True
                
                        self.setFlockingBehaviour(distB, fixtureB, bodyB, fixtureB.nearestRobot)
        
    def EndContact(self, contact):
        super(runSim, self).EndContact(contact)
        
        self.ends += 1
        #print "end contacts ", self.ends
        
        #Get the two objects that have stopped colliding
        fixtureA = contact.fixtureA.userData
        fixtureB = contact.fixtureB.userData
        bodyA = contact.fixtureA.body.userData 
        bodyB = contact.fixtureB.body.userData
        
        #If a robot leaves the powerstrip then stop recharging
        if isinstance(fixtureA, Robot) and isinstance(fixtureB, PowerStrip):
            bodyA.recharging = False
            print "Robot " + str(bodyA.robotid) + ": I have stopped recharging."
            
        if isinstance(fixtureB, Robot) and isinstance(fixtureA, PowerStrip):
            bodyB.recharging = False
            print "Robot " + str(bodyB.robotid) + ": I have stopped recharging."
        
        #TODO: stop communication when out of sensor range
        if isinstance(fixtureA, ProxSensor) and isinstance(fixtureB, ProxSensor):
            
            #Remove from the sensor's contactable robots
            if bodyB.robotid in fixtureA.commsRangeRobots:
                fixtureA.commsRangeRobots.remove(bodyB.robotid)
                
                #Check whether the robot is still in comms range of another sensor
                stillInContact = False
                for sens in bodyA.IRSensList:
                    #Check other sensors in robotA for robotB id
                    if bodyB.robotid in sens.commsRangeRobots:
                        stillInContact = True
            
                if stillInContact == False:
                    bodyA.neighbours.remove(bodyB.robotid)
                
            if bodyA.robotid in fixtureB.commsRangeRobots:
                fixtureB.commsRangeRobots.remove(bodyA.robotid)
                                
                #Check whether the robot is still in comms range of another sensor
                stillInContact = False
                for sens in bodyB.IRSensList:
                    #Check other sensors in robotA for robotB id
                    if bodyA.robotid in sens.commsRangeRobots:
                        stillInContact = True
            
                if stillInContact == False:
                    bodyB.neighbours.remove(bodyA.robotid)
                
        #If a robot is stops heading for a wall or an obstacle then stop turning
        if isinstance(fixtureA, ProxSensor) and (isinstance(bodyB, Obstacle) or isinstance(bodyB, Room) or isinstance(fixtureB, Robot)):
            #Sensor is no longer in contact with robot
            if isinstance(fixtureB, Robot):
                #print bodyA.robotid, "no longer in contact with ", fixtureB.robotid
                
                fixtureA.removeContactRobot(fixtureB)
                if len(fixtureA.contactedRobots) == 0:
                    #print "contactIR set to false for sensor ", fixtureA.proxid
                    fixtureA.contactIR = False
                #If the robot isn't also turning because of an obstacle and isn't stuck then stop turning
                if bodyA.contactObs == False and bodyA.stuck == False:
                    bodyA.turning = False
                    bodyA.direction = "straight"
                    #If recharging and encountered robot, wait before turning to powersource 
                    if bodyA.task == "recharge":
                        bodyA.waittime = 30
                        bodyA.timer = 0
            else:
                #Sensor is no longer in contact with this obstacle or wall
                fixtureA.removeContactObstacle(bodyB)
               
                #If list is empty then set sensor contact obstacle flag to false
                if len(fixtureA.contactedObstacles) == 0:
                    fixtureA.contactObs = False
            
            #Check the front sensors. If turning away and contact stops with the second sensor then the obstacle is clear. Unless the robot is stuck.
            #The second or clause handles tunnels - if sensors 0 and 7 are free, but 6 and 1 are blocked then drive forwards
            if (((fixtureA.proxid == 1 and bodyA.direction == "left") or (fixtureA.proxid == 6 and bodyA.direction == "right")) \
                or ((0 in bodyA.freeSides()) and (7 in bodyA.freeSides()) and (6 in bodyA.contactedSides()) and (1 in bodyA.contactedSides()))) \
                and bodyA.stuck == False :
                #There is an exception to above method if the robot turns the opposite way to expected - e.g. due to another robot
                bodyA.turning = False
                bodyA.direction = "straight"
                bodyA.contactObs = False
                
                #If recharging and stopped encountering obstacle, reset wait timer 
                if bodyA.task == "recharge":
                    bodyA.waittime = 30
                    bodyA.timer = 0
                
        elif isinstance(fixtureB, ProxSensor) and (isinstance(bodyA, Obstacle) or isinstance(bodyA, Room) or isinstance(fixtureA, Robot)):
            #Sensor is no longer in contact with robot
            if isinstance(fixtureA, Robot):
                #print bodyB.robotid, "no longer in contact with ", fixtureA.robotid
                
                fixtureB.removeContactRobot(bodyA)
                if len(fixtureB.contactedRobots) == 0:
                #print "contactIR set to false for sensor", fixtureB.proxid
                    fixtureB.contactIR = False
                
                #If the robot isn't also turning because of an obstacle and isn't stuck then stop turning
                if bodyB.contactObs == False and bodyB.stuck == False:
                    bodyB.turning = False
                    bodyB.direction = "straight"
                    #If recharging and stopped encountering obstacle, reset wait timer 
                    if bodyB.task == "recharge":
                        bodyB.waittime = 30
                        bodyB.timer = 0
            else:
                #Sensor is no longer in contact with obstacle or wall
                fixtureB.removeContactObstacle(bodyA)
                
                #If list is empty then set sensor contact obstacle flag to false
                if len(fixtureB.contactedObstacles) == 0:
                    fixtureB.contactObs = False
                
            #Check the front sensors.
            #The second or clause handles tunnels - if sensors 0 and 7 are free, but 6 and 1 are blocked then drive forwards
            if(((fixtureB.proxid == 1 and bodyB.direction == "left") or (fixtureB.proxid == 6 and bodyB.direction == "right")) \
                or ((0 in bodyB.freeSides()) and (7 in bodyB.freeSides()) and (6 in bodyB.contactedSides()) and (1 in bodyB.contactedSides()))) \
                and bodyB.stuck == False:                
                #There is an exception to above method if the robot turns the opposite way to expected - e.g. due to another robot
                    
                bodyB.turning = False
                bodyB.direction = "straight"
                bodyB.contactObs = False
                
                #If recharging and encountered obstacle, wait before turning to powersource 
                if bodyB.task == "recharge":
                    bodyB.waittime = 30
                    bodyB.timer = 0
    
    #From the real world size in cm calculate the size in simulation units
    def calcSimSize(self, sizeInCm):        
        sizeInUnits = (1/float(self.unitsize))*sizeInCm
        return sizeInUnits

    #From the size in simulation units calculate real world size in cm
    def calccmSize(self, sizeinUnits):
        sizeIncm = sizeinUnits*float(self.unitsize)
        return sizeIncm
    
    #TODO: use the sensor range, rather than hardcoded
    #TODO: average the behaviour rather than using just the nearest robot
    #Use the distance to choose the type of behaviour - distance provided in cm
    #Provide the id of the sensor that the detection occured, a pointer to the robot this sensor is on and a pointer to the robot that has been sensed
    def setFlockingBehaviour(self, distance, sensor, robot, sensedRobot):
        
        sensor_side = robot.sensSide(sensor.proxid)
        
        #if really too close then stop moving and turn away
        if distance <= 2:

            robot.turning = True
            if sensor_side == "right":
                #Set flag to send robot left
                robot.direction = "left"
            elif sensor_side == "left":
                #Set flag to send robot left
                robot.direction = "right"
        
        ###This part implements a simple flocking algorithm, it is difficult with a small sensor range so needs improvement

        #if too close then avoid
#        if distance <= 4:
            
#            if sensor_side == "right":
                #Set flag to send robot left
#                robot.direction = "left"
#            elif sensor_side == "left":
                #Set flag to send robot left
#                robot.direction = "right"
            
        #if middle distance then align 
#        elif distance <= 4:
        
            #Get the heading of the observed robot and turn to match it 
#            otherRobotHeading = sensedRobot.body.transform.R
            
#            robot.changeHeadingRad(otherRobotHeading.angle)
            
        #if too far away then go towards
#        else:
            
            #Find the angle that the sensor is at (eg 15 degrees) and drive in that direction
#            robot.changeHeadingRad(sensor.radpos)

#Run simulation
if __name__ == '__main__': main(runSim)

