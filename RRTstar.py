import numpy as np
# import matplotlib
# matplotlib.use('tkAgg')
import matplotlib.pyplot as plt
import random
import time

from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif' 
plt.rcParams['font.size'] = 10


class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                      #X location
        self.locationY = locationY                      #Y location
        self.children = []                              #children list
        self.parent = None                              #parent node reference
        self.cost = 0

class RRTalgo():
    def __init__(self,start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0],start[1])  #root position
        self.goal = treeNode(goal[0], goal[1])         #goal position                    
        self.nearestNode = None                        #nearest node
        self.iterations = numIterations                #number of iterations to run
        self.grid = grid                               #the map
        self.step_size = stepSize                            #lenght of each branch    
        self.path_distance = 0                         #total path distance    
        self.nearestDist = 10000                       #distance to nearest node
        self.numWaypoints = 0                          #number of waypoints
        self.Waypoints = []                            #the waypoints
        
        self.nearestNodes = []
        self.allNodes = {0:treeNode(start[0],start[1])}
        self.lineDict = {}
        self.finish = False

    #add the point to the nearest node and add goal when reached
    def addChild(self, locationX, locationY, idx):
        tempNode = treeNode(locationX, locationY)
        self.nearestNode.children.append(tempNode)
        tempNode.parent = self.nearestNode 
        tempNode.cost = self.nearestNode.cost + self.distance(self.nearestNode, [locationX,locationY])
        self.allNodes[idx] = tempNode
        # if self.distance(self.goal, point) <= self.step_size:  
        #     self.goal.parent = tempNode
        #     return self.goal
        return tempNode
    
    #sample a random point within grid limits
    def sampleAPoint(self):
        if np.random.rand() < 0.5:
            point = np.array([self.goal.locationX,self.goal.locationY]) 
        else: 
            x = random.randint(1, grid.shape[1]-1)
            y = random.randint(1, grid.shape[0]-1)
            point = np.array([x, y])
        return point

    #steer a distance stepsize from start to end location
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.step_size * self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]-1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0]-1
        return point
    
    #check if obstacle lies between the start node and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        # return False
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0]) 
        for i in range(int(round(self.distance(locationStart,locationEnd),0))):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            if np.int64(round(testPoint[1])) >= grid.shape[0] or np.int64(round(testPoint[0])) >= grid.shape[1]:
                return True
            if self.grid[np.int64(round(testPoint[1])), np.int64(round(testPoint[0]))] == 1:
                return True
        return False

    #find unit vector between 2 points which from a vector
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = 0
        if np.linalg.norm(v) != 0:
            u_hat = v/np.linalg.norm(v)
        return u_hat

    #find the nearest node from a given unconnected point (Euclidean distance)
    def findNearest(self, root, point):
        if not root: 
            return
        # if self.distance(self.goal, point) > self.step_size and self.distance(self.goal, [root.locationX, root.locationY]) >a self.step_size:
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:  
            self.findNearest(child, point) 
        pass

    #find euclidean distance between  a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return round(dist,0)
    
    #check if the goal has been reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.step_size:
            self.finish = True
            return True
        pass

    #reset nearestNode and nearest Distance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        self.nearestNodes = []

    #trace the pathfrom goal to start
    def retraceRRTPath(self, goal):
        # print("type is ", type(self.randomTree.locationX), goal.parent.locationX)
        if goal.locationX == self.randomTree.locationX:
            return
        # print(self.numWaypoints)
        self.numWaypoints += 1
        currentPont = np.array([goal.locationX, goal.locationY])
        # print(currentPont)
        self.Waypoints.insert(0,currentPont)
        self.path_distance += self.step_size
        # print(goal.parent)
        self.retraceRRTPath(goal.parent)

    def Nears(self, root, point, radius):
        if not root: 
            return
        dist = self.distance(root, point)
        if dist <= radius:
            self.nearestNodes.append(root) 
        for child in root.children:  
            self.Nears(child, point, radius) 
        pass

    def chooseParent(self,newNode):
        if len(self.nearestNodes) == 0:
            return newNode
        dlist = []
        for Node in self.nearestNodes: 
            if not self.isInObstacle(Node, newNode):
                dlist.append(Node.cost + self.distance(Node, newNode))
            else:
                dlist.append(float("inf"))
        mincost = min(dlist) 
        if mincost != float("inf"):
            self.nearestNode = self.nearestNodes[dlist.index(mincost)] 
    
    def findInDict(self, dedsired_val):
        for key, value in self.allNodes.items():
            if value == dedsired_val:
                return key
        print("ERROR")

    def reWire(self, newNode):
        newNode_XY = [newNode.locationX, newNode.locationY]
        for Node in self.nearestNodes:
            temp_cost = newNode.cost + self.distance(Node, newNode_XY)
            if temp_cost < Node.cost:
                if not self.isInObstacle(Node, newNode_XY):
                    # print(1)
                    # print(self.lineDict)
                    line_key = (round(Node.parent.locationX, 0), round(Node.parent.locationY, 0), round(Node.locationX, 0), round(Node.locationY, 0))
                    if line_key in self.lineDict: 
                        line = self.lineDict.pop(line_key)
                        line[0].remove()
                        # print(1)

                    # plt.plot([Node.parent.locationX, Node.locationX], [Node.parent.locationY, Node.locationY], 'ko', linestyle="-", linewidth=1.5)
                    idx_node = self.findInDict(Node) 
                    self.allNodes[idx_node].parent = newNode
                    self.allNodes[idx_node].cost = temp_cost 
                    self.lineDict[(round(Node.parent.locationX, 0), round(Node.parent.locationY, 0), round(Node.locationX, 0), round(Node.locationY, 0))] = plt.plot([Node.parent.locationX, Node.locationX], [Node.parent.locationY, Node.locationY], 'bo', markersize = 3, linestyle="-", linewidth=0.5)
                    # plt.pause(1)
#------------------------------------------------------
def stand_rectangle_obs(rows, cols, grid):
    x = random.randint(1, cols-1)
    y = random.randint(1, rows-1)
    for i in range(500):
        for j in range(50):
            if x+i >= cols or y+j >= rows:
                break
            grid[x+i][y+j] = 1
    return grid

def lie_rectangle_obs(rows, cols, grid):
    x = random.randint(1, cols-1)
    y = random.randint(1, rows-1)
    for i in range(500):
        for j in range(50):
            if x+j >= cols or y+i >= rows:
                break
            grid[x+j][y+i] = 1
    return grid

#------------------------------------------------------
rows, cols = 150, 150
grid = np.zeros((rows, cols))   

# for i in range(5):
#     grid = stand_rectangle_obs(rows, cols, grid)
#     grid = lie_rectangle_obs(rows, cols, grid)

# for i in range(120):
#     for j in range(5):
#         grid[i+30][j+100] = 1

# for i in range(80):
#     for j in range(5):
#         grid[i+70][j+60] = 1

# for i in range(80):
#     for j in range(5):
#         grid[i+50][j+20] = 1

# for i in range(20):
#     for j in range(5):
#         grid[i][j+100] = 1

# for i in range(5):
#     for j in range(80):
#         grid[i+50][j+20] = 1


# for i in range(20):
#     for j in range(80):
#         grid[i+50][j+35] = 1 
# for i in range(80):
#     for j in range(20):
#         grid[i+50][j+35] = 1 
# for i in range(80):
#     for j in range(20):
#         grid[i+50][j+105] = 1 

start = np.array([82.0, 120.0])
# start = np.array([100.0, 100.0]) 
goal = np.array([grid.shape[1]-22.0, grid.shape[0]-30.0]) 

grid[int(start[0])][int(start[1])] = 0
grid[int(goal[0])][int(goal[1])] = 0

numIterations = 400
stepSize = 10
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='r', fill = False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'bo')
plt.plot(goal[0], goal[1], 'ro')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

rrt = RRTalgo(start, goal, numIterations, grid, stepSize)

nodes = [treeNode(start[0],start[1])]

start_time = time.time()

for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("Iteration: ",i)
    point = rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if bool == False:
        rrt.Nears(rrt.randomTree, new, rrt.step_size * 4)
        rrt.chooseParent(new)   
        newNode = rrt.addChild(new[0], new[1], i)
        if i %100 == 0:
            plt.pause(0.1)   
        rrt.lineDict[(round(rrt.nearestNode.locationX,0),round(rrt.nearestNode.locationY,0), round(new[0],0),round(new[1],0))] = plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'bo', markersize=3, linestyle="-", linewidth=0.5)
        rrt.reWire(newNode) 
        # plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'bo', linestyle="-", linewidth=0.5)
        
        # print(rrt.lineDict)
        if (rrt.goalFound(new)):
            # print(new[0],new[1])
            # rrt.Nears(rrt.randomTree, [rrt.goal.locationX, rrt.goal.locationY], rrt.step_size)
            # rrt.chooseParent([rrt.goal.locationX, rrt.goal.locationY])
            # print(rrt.nearestNode.cost)
            print("Goal found"+str(i))
            # plt.pause(.1)
            # break
        
end_time = time.time()
code_time = round(end_time - start_time, 2)

print(f"Time: {code_time} seconds")


rrt.resetNearestValues()
rrt.Nears(rrt.randomTree, [rrt.goal.locationX, rrt.goal.locationY], rrt.step_size)
rrt.chooseParent([rrt.goal.locationX, rrt.goal.locationY])
rrt.goal.parent = rrt.nearestNode
dist_to_goal = rrt.distance(rrt.nearestNode, [rrt.goal.locationX,rrt.goal.locationY])
rrt.goal.cost = rrt.nearestNode.cost + dist_to_goal

print([rrt.nearestNode.locationX,  rrt.nearestNode.locationY], rrt.goal.cost, dist_to_goal)
rrt.retraceRRTPath(rrt.goal)


rrt.Waypoints.insert(0,start)
print("Number of waypoints: ", rrt.numWaypoints)
# print("Path distance: ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints) 

for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]],[rrt.Waypoints[i][1],rrt.Waypoints[i+1][1],], 'ro', markersize = 4,  linestyle="-", linewidth=1.5)
    # plt.plot(rrt.Waypoints[i][0], rrt.Waypoints[i][1], 'ro')
    plt.pause(0.1)


plt.imshow(grid, cmap='binary')
# plt.colorbar()  # Add a colorbar for reference (optional)
# plt.savefig('RRTstar.png', bbox_inches='tight', pad_inches=0)
plt.show()
# quit()