import heapq #for search, so I can put in a priority queue
import serial #for sending stuff to odometry


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

    def __init__(self, goal, grid): #goal is [int, int], grid is 2D Lidar map
        #declares attributes of search object
        self.goal = goal
        self.grid = grid
        self.start = [int(len(grid)/2), int(len(grid[0])/2)] #starting position is always center of map

    def getStart(self):
        return self.start
    

    def getSuccessors(self, position):
        #successors is list of potential positions to move to, along with the direction moved
        successors = []
        y = position[0]
        x = position[1]
        yhi = [[y + 1, x], [0, 1, 0], "y", 1.0, [y, x]]
        ylow = [[y - 1, x], [0, -1, 0], "y", 1.0, [y, x]]
        xhi = [[y, x + 1], [0, 1, 0], "x", 1.0, [y, x]]
        xlow = [[y, x - 1], [0, -1, 0], "x", 1.0, [y, x]]
        if y + 1 != len(grid): #prevents exceeding range
            successors.append(yhi)
        if y != 0:
            successors.append(ylow)
        if x + 1 != len(grid[1]):
            successors.append(xhi)
        if x != 0:
            successors.append(xlow)
        return successors

    
    def checkGoal(self, position):    #checks if goal state has been reached
        if position == goal:
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
        
        if lastAction[1] != action[1] and lastAxis == "y": #turning from +y to -x or -y to +x
            return "B 99 50"
        elif lastAction[1] != action[1] and lastAxis == "x": #turning +x to -y or -x to +y
            return "C 99 50"
        elif lastAxis == "y": #turning +y to +x or -y to -x
            return "C 99 50"
        return "B 99 50"   #turning +x to +y or -x to -y
        
                    

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
        node = [position, ["D 00 00"], lastAxis, 0.0, [position]] #makes node for starting position, starting movement is a stop
        queue.push(node, 0.0)    #add start node to queue
        while not queue.isEmpty():
            position, actions, lastAxis, cost, coord = queue.pop() #removes top of queue
            if position not in visited and grid[position[0]][position[1]] != 1: #if node is visited, move onto next in queue
                visited.append(position)
                if self.checkGoal(position):
                    print(position) #for demo purposes, prints goal node when reached
                    #print()
                    print(coord)#prints coordinates ordered in path
                    actions.pop()
                    return actions
                else:
                    """n's in successors are different from nodes, only one action is passed through a successor.
                    Nodes in the queue have a list of actions leading up to their position from start 
                    """
                    for n in self.getSuccessors(position): #goes through each possible successor
                        if lastAxis != n[2]:
                            tmp = actions + [self.computeTurn(lastAxis, actions, n[1])] + ["A 99 00"]
                        else:
                            tmp = actions + ["A 99 00"]
                        tmp2 = coord + [n[4]]
                        h = self.calcManhattan(n[0])
                        #print(coord[len(coord)-1][0])
                        #print(self.grid[1][2])
                        cost += self.grid[coord[len(coord)-1][0]][coord[len(coord)-1][1]] #gets weight of node, and updates path cost
                        node = (n[0], tmp, n[2], cost, tmp2) #new node w/ actions and cost
                        queue.push(node, cost + h) #pushes new node onto stack
                        #print(actions)
        return actions  #returns list of actions



"""
# y axis along each rows(positive down), x axis is along each column(positive to right)
# grid has obstruction in 3rd row(y axis), and is 15x15
#MAKE GOAL AT BOTTOM ie HIGH Y AXIS
"""
goal = [13, 7] #goal is +y direction from start, same x value as start
grid = [
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
]
robot = Search(goal, grid)
dir_list = robot.searchA()
print (dir_list)
ser = serial.Serial('/dev/cu.usbmodem14201')  # open serial port (top left port of my laptop)
print(ser.name)         # check which port was really used
ser.write(b'hello')     # write a string
ser.close()   
"""
#if code works, should route left around obstruction
"""

                        
