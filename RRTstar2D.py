import numpy as np
# import matplotlib
# matplotlib.use('tkAgg')
import matplotlib.pyplot as plt
import random
from shapely.geometry import Polygon

from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif' 
plt.rcParams['font.size'] = 10

import random

class treeNode():
    def __init__(self, locationX, locationY, angle):
        self.locationX = locationX                      #X location
        self.locationY = locationY                      #Y location
        self.angle = angle
        self.children = []                              #children list
        self.parent = None                              #parent node reference
        self.cost = 0

class RRTalgo():
    def __init__(self,start, goal, numIterations, grid, stepSize, polygon, Obstacles, smooth):
        self.randomTree = treeNode(start[0], start[1], start[2]) #root position
        self.goal = treeNode(goal[0], goal[1], goal[2])            #goal position                
        self.nearestNode = None                        #nearest node
        self.iterations = numIterations                #number of iterations to run
        self.grid = grid                               #the map
        self.step_size = stepSize                            #lenght of each branch    
        self.path_distance = 0                         #total path distance    
        self.nearestDist = 10000                       #distance to nearest node
        self.numWaypoints = 0                          #number of waypoints
        self.Waypoints = []                            #the waypoints

        self.nearestNodes = []
        self.allNodes = {0:treeNode(start[0],start[1],start[2])}
        self.lineDict = {}
        self.finish = False

        self.polygon = polygon  
        self.Obstacles = Obstacles  
        self.smooth = smooth

    #add the point to the nearest node and add goal when reached
    def addChild(self, locationX, locationY, angle, idx):
        tempNode = treeNode(locationX, locationY, angle)
        self.nearestNode.children.append(tempNode)
        tempNode.parent = self.nearestNode 
        tempNode.cost = self.nearestNode.cost + self.distance(self.nearestNode, [locationX,locationY,angle])
        self.allNodes[idx] = tempNode
        if self.finish:  
            self.goal.parent = tempNode
            return self.goal
        return tempNode

    #sample a random point within grid limits
    def sampleAPoint(self):
        random_number = random.random()
        if random_number < 0.5:
            point = np.array([self.goal.locationX,self.goal.locationY,self.goal.angle])
            print(1)
        else:
            x = random.randint(1, self.grid.shape[1]-1)
            y = random.randint(1, self.grid.shape[0]-1)
            angle = random.randint(1, 360)
            point = np.array([x, y, angle])
        return point

    #steer a distance stepsize from start to end location
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.step_size * self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1], locationEnd[2]])
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]-1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0]-1
        return point
    
    #check if obstacle lies between the start node and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        return False
        startX, startY, startAngle = locationStart.locationX, locationStart.locationY, locationStart.angle
        endX, endY, endAngle = locationEnd[0], locationEnd[1], locationEnd[2]
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.step_size):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            if np.int64(round(testPoint[1])) >= grid.shape[0] or np.int64(round(testPoint[0])) >= grid.shape[1]:
                return True
            if self.grid[np.int64(round(testPoint[1])), np.int64(round(testPoint[0]))] == 1:
                return True
             
            t = i/(self.step_size)
            actualX = startX*(1-t) + t*endX
            actualY = startY*(1-t) + t*endY
            actualAngle = startAngle*(1-t) + t*endAngle
            rotation_matrix = np.array([[np.cos(actualAngle), -np.sin(actualAngle)],
                                        [np.sin(actualAngle), np.cos(actualAngle)]])
            rotated_vertices = np.dot(rrt.polygon, rotation_matrix)
            center = np.mean(rotated_vertices, axis = 0)
            translation_poin = np.array([actualX, actualY])
            translation_vector = translation_poin - center
            translated_vertices = rotated_vertices + translation_vector

            for obstacle in self.Obstacles:
                obstacle_polygon = Polygon(obstacle)
                translated_polygon = Polygon(translated_vertices)
                if obstacle_polygon.intersects(translated_polygon): 
                    return True

            # Check if the translated vertices are outside the grid
            for vertex in translated_vertices:
                x, y = int(round(vertex[0])), int(round(vertex[1]))
                if x < 0 or x >= self.grid.shape[1] or y < 0 or y >= self.grid.shape[0]:
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
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children: 
            self.findNearest(child, point)
        pass

    #find euclidean distance between  a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2 + (node1.angle - point[2])**2)
        return dist
    
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
        self.numWaypoints += 1
        currentPoint = np.array([goal.locationX, goal.locationY, goal.angle])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance += self.step_size
        self.retraceRRTPath(goal.parent) 

    def Nears(self, root, point):
        if not root: 
            return
        dist = self.distance(root, point)
        if dist <= self.step_size*4:
            self.nearestNodes.append(root) 
        for child in root.children:  
            self.Nears(child, point) 
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
        newNode_XY = [newNode.locationX, newNode.locationY, newNode.angle]
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


'''----------------------HELP FUNCTIONS-------------------------------'''
def paint_area(polygon, point, area_clr):
    angle_radians = np.radians(point[2]) 
    rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                                [np.sin(angle_radians), np.cos(angle_radians)]])
    rotated_vertices = np.dot(polygon, rotation_matrix)
    center = np.mean(rotated_vertices, axis = 0)
    translation_poin = np.array([point[0],point[1]])
    translation_vector = translation_poin - center
    translated_vertices = rotated_vertices + translation_vector 
    x1, y1 = translated_vertices.T 
    x1 = np.append(x1, x1[0])
    y1 = np.append(y1, y1[0])
    plt.fill(x1, y1, color= area_clr, alpha=0.5) 
    plt.plot(x1, y1, color= area_clr) 
'''-------------------------------------------------------------------'''




'''--------------------------PARAMETERS-------------------------------'''
rows, cols = 1500, 1500
grid = np.zeros((rows, cols))    
start = np.array([100.0, 100.0, 28])
# goal = np.array([grid.shape[1]-750, grid.shape[0]-750, 67]) 
goal = np.array([grid.shape[1]-100, grid.shape[0]-1200, 67]) 
# goal = np.array([400.0, 1000.0, 113]) 

grid[int(start[0])][int(start[1])] = 0
grid[int(goal[0])][int(goal[1])] = 0

numIterations = 1000000
stepSize = 100
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='r', fill = False)
smooth = 10

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'bo')
plt.plot(goal[0], goal[1], 'ro')
ax = fig.gca()
# ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$') 
'''---------------------------------------------------------------------'''




'''------------------------------POLYGONS-------------------------------'''
polygonL = np.array([(0, 0), (0, 120), (40, 120), (40, 40), (80, 40), (80, 0)]) * 0.7
polygonO = np.array([(0, 0), (0, 160), (160, 160), (160, 0)])
polygonStar = np.array([
    (0, 5), (1, 1), (5, 1), (2, -1), (3, -5),
    (0, -2), (-3, -5), (-2, -1), (-5, 1), (-1, 1)
])
scale_factor = 12
polygonStar *= scale_factor
'''----------------------------------------------------------------------'''




'''---------------------------OBSTACLES----------------------------------'''
obstacleI1 = np.array([(300,0),(300,1300),(310,1300),(310,0)])
obstacleI2 = obstacleI1 + np.array([800,0])
obstacleI3 = obstacleI1 + np.array([400,200])
obstacleO1 = np.array([(0, 0), (0, 160), (460, 160), (460, 0)])*0.5 + np.array([300, 750])
obstacleO2 = np.array([(0, 0), (0, 160), (460, 160), (460, 0)])*0.5 + np.array([870, 750])
obstacleO3 = np.array([(0, 0), (0, 160), (460, 160), (460, 0)])*0.5 + np.array([500, 450])
obstacleO4 = np.array([(0, 0), (0, 160), (460, 160), (460, 0)])*0.5 + np.array([700, 450])
Obstacles = []#[obstacleI1, obstacleI2, obstacleI3, obstacleO1, obstacleO2, obstacleO3, obstacleO4]
for i in range (len(Obstacles)):
    x, y = Obstacles[i].T
    plt.fill(x, y, color='black') 
'''-----------------------------------------------------------------------'''





'''-------------------------MAIN FIGURE------------------------------------'''
polygon = polygonStar
paint_area(polygon, start, 'purple')
paint_area(polygon, goal, 'red')
'''------------------------------------------------------------------------'''






'''---------------------------------RRT------------------------------------'''
rrt = RRTalgo(start, goal, numIterations, grid, stepSize, polygon, Obstacles, smooth) 
for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("Iteration: ",i)
    point = rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if bool == False:
        rrt.Nears(rrt.randomTree, new)
        rrt.chooseParent(new)   
        newNode = rrt.addChild(new[0], new[1], new[2], i) 
        # plt.pause(0.1)
        rrt.lineDict[(round(rrt.nearestNode.locationX,0),round(rrt.nearestNode.locationY,0), round(new[0],0),round(new[1],0))] = plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'bo', markersize=3, linestyle="-", linewidth=0.5)
        rrt.reWire(newNode) 
        if (rrt.goalFound(new)):
            rrt.addChild(new[0], new[1], new[2], i)
            print("Goal found")
            break

rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0,start)
print("Number of waypoints: ", rrt.numWaypoints)
print("Path distance: ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints)
'''------------------------------------------------------------------------'''





'''------------------------------ANIMATION---------------------------------'''  
for i in range(len(rrt.Waypoints)-2):
    plt.plot(
        [rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]],
        [rrt.Waypoints[i][1],rrt.Waypoints[i+1][1],],
        linestyle="-", 
        color = 'r',
        marker='o'
        )  
    startX, startY, startAngle = rrt.Waypoints[i][0], rrt.Waypoints[i][1], np.radians(rrt.Waypoints[i][2])
    endX, endY, endAngle = rrt.Waypoints[i+1][0], rrt.Waypoints[i+1][1], np.radians(rrt.Waypoints[i+1][2]) 
    for j in range(stepSize+1): 
        if (j % 10) == 0:
            t = j/(stepSize)
            actualX = startX*(1-t) + t*endX
            actualY = startY*(1-t) + t*endY
            actualAngle = startAngle*(1-t) + t*endAngle
            rotation_matrix = np.array([[np.cos(actualAngle), -np.sin(actualAngle)],
                                        [np.sin(actualAngle), np.cos(actualAngle)]])
            rotated_vertices = np.dot(polygon, rotation_matrix)
            center = np.mean(rotated_vertices, axis = 0)
            translation_poin = np.array([actualX, actualY])
            translation_vector = translation_poin - center
            translated_vertices = rotated_vertices + translation_vector
            x, y = translated_vertices.T 
            x = np.append(x, x[0])
            y = np.append(y, y[0])
            L, = plt.plot(x, y, color='black') 
            F, = plt.fill(x, y, color='purple', alpha=0.5)
            plt.pause(0.001)
            L.remove()
            F.remove() 

plt.imshow(grid, cmap='binary')
plt.colorbar()  # Add a colorbar for reference (optional)
plt.show()
# quit()
'''-------------------------------------------------------------------------'''
