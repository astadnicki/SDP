import heapq
import serial #for sending stuff to odometry

#subscriber stuff

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray, #get data type from John
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info(% msg.data)
        goal = [len(msg.data) - 1, len(msg.data[0])/2]
        """
        Right now, goal is just made to be in corner, we can reset where-ever it needs to be
        """
        robot = Search(goal, msg.data) #
        dir_list = robot.searchA()
        send = str(dir_list[0][0]) + " " + str(dir_list[1][0]) + " " + str(dir_list[2][0]) #convert direction to string
        ser = serial.Serial("""Whatever serial port we can send to""")
        ser.write(send) #writes first instruction from path planning to odometry
        

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
        yhi = [[y + 1, x], ["a", 1, 0], "y", 1.0, [y, x]]
        ylow = [[y - 1, x], ["a", -1, 0], "y", 1.0, [y, x]]
        xhi = [[y, x + 1], ["a", 1, 0], "x", 1.0, [y, x]]
        xlow = [[y, x - 1], ["a", -1, 0], "x", 1.0, [y, x]]
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
            return ["b", 1, 90]
        elif lastAction[1] != action[1] and lastAxis == "x": #turning +x to -y or -x to +y
            return ["c", 1, 90] 
        elif lastAxis == "y": #turning +y to +x or -y to -x
            return ["c", 1, 90]
        return ["b", 1, 90]   #turning +x to +y or -x to -y
        
                    

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
        node = [position, [["a", 0, 0]], lastAxis, 0.0, [position]] #makes node for starting position
        queue.push(node, 0.0)    #add start node to queue
        while not queue.isEmpty():
            position, actions, lastAxis, cost, coord = queue.pop() #removes top of queue
            if position not in visited and grid[position[0]][position[1]] != 1: #if node is visited, move onto next in queue
                visited.append(position)
                if self.checkGoal(position):
                    #print(position) #for demo purposes, prints goal node when reached
                    #print()
                    #print(coord)#prints coordinates ordered in path
                    actions.pop()
                    return actions
                else:
                    """n's in successors are different from nodes, only one action is passed through a successor.
                    Nodes in the queue have a list of actions leading up to their position from start 
                    """
                    for n in self.getSuccessors(position): #goes through each possible successor
                        if lastAxis != n[2]:
                            tmp = actions + [self.computeTurn(lastAxis, actions, n[1])] + [["a", 1, 0]]
                        else:
                            tmp = actions + [["a", 1, 0]]
                        tmp2 = coord + [n[4]]
                        h = self.calcManhattan(n[0])
                        cost += 1   #updates path cost
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
robot = Search(goal, grid)
dir_list = robot.searchA()
#print (dir_list)

"""
#if code works, should route left around obstruction
"""

                        
