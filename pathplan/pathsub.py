import heapq
import serial #for sending stuff to odometry
import time
import numpy as np

#subscriber stuff

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid, #data type
            '/global_costmap/costmap',
            self.listener_callback,
            10)
        self.ser = serial.Serial('/dev/ttyACM0')
        time.sleep(2)
        self.ser.close()
        self.subscription  # prevent unused variable warning
        
    def squarize(self,msg):
        height = int(msg.info.height)
        width = int(msg.info.width)
        xoffset = 0
        yoffset = 0
        j=0
        k=0
        
        print(height)
        print(width)
       
	
	
        if height > width:
                yoffset = (height-width)/2
                print(len(msg.data))
                if yoffset == int(yoffset):
               	    Oned_sqr = msg.data[int(yoffset)*width-1:len(msg.data)-(int(yoffset)*width)-1]
               	else:
               	    Oned_sqr = msg.data[(int(yoffset+1))*width-1:len(msg.data)-(int(yoffset))*width-1]
                print(len(Oned_sqr))
        if width > height:
                if xoffset == int(xoffset):
                    Oned_sqr = msg.data[int(xoffset*height-1):int(len(msg.data)-xoffset*height-1)]
                else:
                    Oned_sqr = msg.data[int((xoffset-1)*height-1):int(len(msg.data)-xoffset*height-1)]
                xoffset:int = int((width-height)/2)
                Oned_sqr = msg.data[int(xoffset*height-1):int(len(msg.data)-xoffset*height-1)]
     
        minVal = min(width,height)
        matrix = np.array(Oned_sqr).reshape(minVal,minVal)
        #print(matrix)
        return matrix
        	
        '''for i in range(int(height-yoffset)):
                 matrix[i][0:width] = msg.data[int(((i+yoffset)*width)+xoffset):int(((i+1)*width))]
                 #print(msg.data[int(((i+yoffset)*width)+xoffset):int(((i+1)*width))])
        return matrix
	'''	
    	

    def listener_callback(self, msg):
        #print(msg.data)
        area = self.squarize(msg)
        print(len(area),len(area[0]))
        
        goal = [int(len(area) - 1), int(len(area)/2)]
 
        """
        Right now, goal is just made to be in corner, we can reset where-ever it needs to be
        """
        xpos = msg.info.origin.position.x + msg.info.width/2
        ypos = msg.info.origin.position.y + msg.info.height/2
        #print(len(area), len(area[0]))
        #print(area)
        self.ser.open()

        robot = Search(goal, area,[int(xpos),int(ypos)]) #
        #print("here")
        dirList = robot.searchA()
        #print('dirList')
        print(int(xpos), int(ypos))
        print(dirList)
        for item in dirList[1:3]:    #first value in every path isn't used
    	        self.ser.write(item[0].encode("utf-8"))
    	        time.sleep(1.65)    #needs this amount of time between commands or else it skips
    	        print("cum")
    	        self.ser.write(item[0].encode("utf-8"))
    	        time.sleep(1.65)    #needs this amount of time between commands or else it skips
    	        print("cum1")
    	        #line = self.ser.readline().decode('utf-8').rstrip()
        print('here')
        self.ser.close()
        
        
        

# Path Planning stuff


class PriorityQueue:
    """
    Priority Queue built off of heap datatype
    """

    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority): #adds item to priority queue w/
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self): #removes item from priority queue
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        """
        If item already in priority queue with higher priority, update its priority and rebuild the heap.
        If item already in priority queue with equal or lower priority, do nothing.
        If item not in priority queue, do the same thing as self.push.
        """
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)
            
class Search():
    """
    Things to pass on in this:
    - LIDAR map(grid)
    - goal position on grid
        
    Things to get from it:
    List of actions which is a list of integers:
        'x x.x x.xxx'
        First one is either 0,1,2 (0- Forward, 1- Right, 2-Left)
        x.x is speed 0-1 (can be negative for a reverse motion)
        x.xxx is angle (only used for left and right)
    """

    def __init__(self, goal, grid, start): #goal is [int, int], grid is 2D Lidar map
        #declares attributes of search object
        self.goal = goal
        self.grid = grid
        self.start = start

    def getStart(self):
        return self.start
    

    def getSuccessors(self, position):
        #successors is list of potential positions to move to, along with the direction moved
        successors = []
        y = position[0]
        x = position[1]
        yhi = [[y + 13, x], ["AA 99 00", 1], "y", 1.0, [y, x]]
        ylow = [[y - 13, x], ["AA 99 00", -1], "y", 1.0, [y, x]]
        xhi = [[y, x + 13], ["AA 99 00", 1], "x", 1.0, [y, x]]
        xlow = [[y, x - 13], ["AA 99 00", -1], "x", 1.0, [y, x]]
        if y + 1 != len(self.grid) and y - 1 > 0: #prevents exceeding range
            successors.append(yhi)
        if y != 0 and y - 1 != 0:
            successors.append(ylow)
        if x + 1 != len(self.grid[1]) and x - 1 > 0:
            successors.append(xhi)
        if x != 0 and y - 1 != 0:
            successors.append(xlow)
        return successors

    
    def checkGoal(self, position):    #checks if goal state has been reached
        if position == self.goal:
            return True
        return False
    
    def computeTurn(self, lastAxis, actions, action): #makes the Robit TUUUUUUURRRRRRN##
        #making this very simple for now
        """
        if lastAxis == "y":
            return [1, 1, 90] #turn right if going to +x
        return [2, 1, 90]#turn left if going to +y
        """
        # trying to make it not simple, but not implementing 180° turns yet
        lastAction = actions[-1]
        #print(actions)
        
        if lastAction[1] != action[1] and lastAxis == "y": #turning from +y to -x or -y to +x
            return "CC 99 50"
        elif lastAction[1] != action[1] and lastAxis == "x": #turning +x to -y or -x to +y
            return "BB 99 50"
        elif lastAxis == "y": #turning +y to +x or -y to -x
            return "BB 99 50"
        return "CC 99 50"   #turning +x to +y or -x to -y
        
                    

    def calcDist(self): 
        #will be implemented when encoders are added
        util.raiseNotDefined()

    def calcManhattan(self, position): #manhattan position/heuristsic
        pos = position
        goal = self.goal
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def searchA(self): #visited is 2D array of visited coordinates
        """cost is cost of current path
        ACTUAL SEARCH ALGORITHM
        returns series of directions, currently speed 1 should move 1 imaginary unit on the grid
        """
        if self.checkGoal(self.start):
            return
        visited = []
        queue = PriorityQueue()
        lastAxis = "y"
        position = self.start
        node = [position, [["DD 00 00", 1]], lastAxis, 0.0, [position]] #makes node for starting position, starting movement is a stop
        queue.push(node, 0.0)    #add start node to queue
        while not queue.isEmpty():
            position, actions, lastAxis, cost, coord = queue.pop() #removes top of queue
            #print(position)
            if position[0] < len(self.grid) and position[1] < len(self.grid) and position[0] > 0 and position[1] > 0:
                #print(position)
                if position not in visited and self.grid[position[0]][position[1]] < 99 : #if node is visited, move onto next in queue
                    visited.append(position)
                    if self.checkGoal(position):
                        #print(position) #for demo purposes, prints goal node when reached
                        #print()
                        coord += [position]
                        #print(coord)#prints coordinates ordered in path
                        #actions.pop()
                        actions = actions + [["DD 00 00", 1]]
                        actions[0] = coord[2]
                        return actions
                    else:
                        """n's in successors are different from nodes, only one action is passed through a successor.
                        Nodes in the queue have a list of actions leading up to their position from start 
                        """
                        for n in self.getSuccessors(position): #goes through each possible successor
                            if lastAxis != n[2]:
                                #print(n[2], lastAxis)
                                tmp = actions + [[self.computeTurn(lastAxis, actions, n[1]), n[1][1]]] + [n[1]]
                            else:
                                tmp = actions + [n[1]]
                            tmp2 = coord + [n[4]]
                            h = self.calcManhattan(n[0])
                            #print(coord[len(coord)-1][0])
                            #print(self.grid[1][2])
                            cost += 1 + self.grid[coord[len(coord)-1][0]][coord[len(coord)-1][1]] #gets weight of node, and updates path cost
                            node = (n[0], tmp, n[2], cost, tmp2) #new node w/ actions and cost
                            queue.push(node, cost + h) #pushes new node onto stack
                            #print(actions)
        return actions  #returns list of actions



"""
# y axis along each rows(positive down), x axis is along each column(positive to right)
# grid has obstruction in 3rd row(y axis), and is 15x15
#MAKE GOAL AT BOTTOM ie HIGH Y AXIS
"""

#main for subscriber stuff
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#goal = [13, 7] #goal is +y direction from start, same x value as start
"""grid = [
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]"""
#robot = Search(goal, grid)
#dir_list = robot.searchA()
#print (dir_list)

"""
#if code works, should route left around obstruction
"""

'''
for item in dirList[1:]:    #first value in every path isn't used
    print(item)
    ser.write(item[0].encode("utf-8"))
    time.sleep(1.65)    #needs this amount of time between commands or else it skips
    line = ser.readline().decode('utf-8').rstrip()
    
    #tried doing a while loop to wait for finished command and that just didn't fucking work 
    #while ser.in_waiting:  # Or: while ser.inWaiting():
    #    print(ser.readline().decode('utf-8').rstrip())
    
    #print(line)
    #print(item)
'''
                      
