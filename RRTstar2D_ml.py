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

from scipy.stats import gaussian_kde

import time


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

        self.obsSpace = []
        self.freeSpace = []

    #add the point to the nearest node and add goal when reached
    def addChild(self, locationX, locationY, angle, idx):
        tempNode = treeNode(locationX, locationY, angle)
        self.nearestNode.children.append(tempNode)
        tempNode.parent = self.nearestNode 
        tempNode.cost = self.nearestNode.cost + self.distance(self.nearestNode, [locationX,locationY,angle])
        self.allNodes[idx] = tempNode
        # if self.finish:  
        #     self.goal.parent = tempNode
        #     return self.goal
        return tempNode

    #sample a random point within grid limits
    def sampleAPoint(self):
        random_number = random.random()
        if random_number < 0.5:
            point = np.array([self.goal.locationX,self.goal.locationY,self.goal.angle])
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
        startX, startY, startAngle = locationStart.locationX, locationStart.locationY, locationStart.angle
        endX, endY, endAngle = locationEnd[0], locationEnd[1], locationEnd[2]
        u_hat = self.unitVector(locationStart, locationEnd)
        if np.allclose(u_hat, 0):
            # print("ERROR: Unit vector.")
            return True 
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
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2 + 0.01*(node1.angle - point[2])**2)
        return round(dist,2)
    
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
        print("ERROR: Not Found.")

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
                    self.lineDict[(round(Node.parent.locationX, 0), 
                                   round(Node.parent.locationY, 0), 
                                   round(Node.locationX, 0), 
                                   round(Node.locationY, 0))] = plt.plot([Node.parent.locationX, Node.locationX], 
                                                                         [Node.parent.locationY, Node.locationY], 
                                                                         'bo', markersize = 2, linestyle="-", 
                                                                         linewidth=0.5, alpha = 0.5)
                    # plt.pause(1)

#--------------------ML--------------------------------
    #return probability of the point belongs to obstacle space 
    def obsProb(self):
        obsCount = 0
        S = grid.shape[0] * grid.shape[0]
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i][j] == 1:
                    obsCount += 1 
        obstacleProbability = obsCount/S
        return obstacleProbability
    
    #if point is on Obstacle, not edge like in function isInObstacle
    def OnObstacle(self, x):     
        if np.int64(round(x[1])) >= grid.shape[0] or np.int64(round(x[0])) >= grid.shape[1]:
            return True
        if self.grid[np.int64(round(x[1])), np.int64(round(x[0]))] == 1:
            return True 
        actualX = x[0]
        actualY = x[1]
        actualAngle = x[2]
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

    def appendSpace(self, point, typeSpace): 
        if typeSpace == "obs":
            # if not any(np.array_equal(point, arr) for arr in self.obsSpace):
            self.obsSpace.append(point)
            # plt.plot(point[0],point[1], alpha=0.3, mec='none', color = 'purple', marker = 'o', markersize = 15)
        if typeSpace == "free":
            # if not any(np.array_equal(point, arr) for arr in self.freeSpace):
            self.freeSpace.append(point)
            # plt.plot(point[0], point[1], alpha=0.3, mec='none', color = 'green', marker = 'o', markersize = 15)

    #for the start data collection, because if have to empty arrays obsSpace and freeSpace we cant compute KDE and probabilities properly
    def adaptiveSampling(self): 
        while len(self.obsSpace) <= 100 or len(self.freeSpace) <= 100: 
            x = self.sampleAPoint() 
            if self.OnObstacle(x):
                self.appendSpace(x,"obs") 
            else:
                self.appendSpace(x,"free")   
    
    def sampleDensity(self): 
        q_free = 0
        q_obs = 1 
        cnt = 0
        while q_free < q_obs:
            cnt+=1
            x_rand = self.sampleAPoint()
            P_free = len(self.freeSpace) / (len(self.freeSpace) + len(self.obsSpace))
            P_obs = 1-P_free
            # P_obs = self.obsProb()
            # P_free = 1 - P_obs
            b_free = self.GKDE(x_rand, np.array(self.obsSpace))
            b_obs = self.GKDE(x_rand, np.array(self.freeSpace)) 
            q_free = b_free * P_free
            q_obs = b_obs * P_obs
        # print(cnt)
        # print(f"q_free: {q_free}, q_obs: {q_obs}")
        return x_rand

    def GKDE(self, x, X):
        # x...point; X...space(obs/free) 
        kde = gaussian_kde(X.T) 
        density_value = kde(x)[0] *1000  
        # print(type(density_value))
        # if density_value != 0:
        #     print(density_value)
        return density_value


    def mlSample(self):
        x = self.sampleDensity()
        cnt = 0
        # if self.OnObstacle(x):
        #     self.appendSpace(x, "obs")
        #     x = self.sampleAPoint()
        while self.OnObstacle(x):
            cnt+=1
            self.appendSpace(x, "obs")
            x = self.sampleDensity()
            if cnt == 2:
                x = self.sampleAPoint()
                break 

        if not self.OnObstacle(x):
            self.appendSpace(x,"free") 
        return x 
    
    def epanechnikovKernel(self, x):
        d = len(x)
        norm_x_squared = np.dot(x, x) 
        if norm_x_squared <= 1:
            return (d + 2) / (2 * np.pi) * (1 - norm_x_squared)
        else:
            return 0
    
    # Kernel Density Estimator        
    def KDE(self, x, X):
        # x...point; X...space(obs/free) 
        m = len(X)
        d = len(x)
        if m == 0:
            print("ERROR: empty array")
            return 0
        h = (np.log(m)/m)**(1/d)  
        H = np.diag([h]*d)  
        detH = np.linalg.det(H)
        kernel_sum = 0
        for i in range(m): 
            tmp = np.matmul(np.linalg.inv(H), (x-X[i]))   
            kernel_value = self.epanechnikovKernel(tmp) 
            kernel_sum += kernel_value
        density_value = 1/(m * detH**m) * kernel_sum
        # print("density_value: ", density_value)
        return density_value


'''----------------------HELP FUNCTIONS-------------------------------'''
def paint_area(polygon, point, area_clr, alpha):
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
    plt.fill(x1, y1, color= area_clr, alpha=alpha) 
    plt.plot(x1, y1, color= area_clr) 
'''-------------------------------------------------------------------'''




'''--------------------------PARAMETERS-------------------------------'''
rows, cols = 150, 150
grid = np.zeros((rows, cols))    
start = np.array([80.0, 20.0, 28])
# goal = np.array([grid.shape[1]-750, grid.shape[0]-750, 67]) 
goal = np.array([grid.shape[1]-71, grid.shape[0]-60, 67]) 
# goal = np.array([400.0, 1000.0, 113]) 

grid[int(start[0])][int(start[1])] = 0
grid[int(goal[0])][int(goal[1])] = 0

numIterations = 1000
stepSize = 10
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='r', fill = False)
smooth = 10

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'bo')
plt.plot(goal[0], goal[1], 'ro')
ax = fig.gca()
# ax.add_patch(goalRegion)
# plt.xlabel('X-axis $(m)$')
# plt.ylabel('Y-axis $(m)$') 
plt.xticks([])
plt.yticks([]) 
plt.xlim(0,150)
plt.ylim(0,150)
'''---------------------------------------------------------------------'''




'''------------------------------POLYGONS-------------------------------'''
polygonL = np.array([(0, 0), (0, 12), (4, 12), (4, 4), (8, 4), (8, 0)])
polygonO = np.array([(0, 0), (0, 160), (160, 160), (160, 0)])
polygonStar = np.array([
    (0, 5), (1, 1), (5, 1), (2, -1), (3, -5),
    (0, -2), (-3, -5), (-2, -1), (-5, 1), (-1, 1)
])*1.2 
'''----------------------------------------------------------------------'''




'''---------------------------OBSTACLES----------------------------------'''
obstacleI1 = np.array([(30,0),(30,67),(120,67),(120,0)])
obstacleI2 = obstacleI1 + np.array([0,83])
obstacleI3 = np.array([(105,50),(105,130),(125,130),(125,50)]) 

obstacleU1 = np.array([(35,50),(115,50),(115,70),(35,70)])
obstacleU2 = np.array([(35,50),(55,50),(55,130),(35,130)])
obstacleU3 = np.array([(105,50),(125,50),(125,130),(105,130)])

Obstacles = [obstacleU1,obstacleU2,obstacleU3]
for i in range (len(Obstacles)):
    x, y = Obstacles[i].T
    plt.fill(x, y, color='black')  
'''-----------------------------------------------------------------------'''





'''-------------------------MAIN FIGURE------------------------------------'''
polygon = polygonStar
paint_area(polygon, start, 'purple', 0.5)
paint_area(polygon, goal, 'red', 0.5)
'''------------------------------------------------------------------------'''





start_time = time.time() 
'''---------------------------------RRT------------------------------------'''
rrt = RRTalgo(start, goal, numIterations, grid, stepSize, polygon, Obstacles, smooth) 
rrt.adaptiveSampling()

draw_prediction = True
for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("Iteration: ",i)
    # point = rrt.sampleAPoint()
    point = rrt.mlSample()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if bool == False:
        rrt.Nears(rrt.randomTree, new, rrt.step_size*4)
        rrt.chooseParent(new)   
        newNode = rrt.addChild(new[0], new[1], new[2], i) 
        if i%100==0:
            plt.pause(0.1)  
        rrt.lineDict[(round(rrt.nearestNode.locationX,0),
                      round(rrt.nearestNode.locationY,0), 
                      round(new[0],0),round(new[1],0))] = plt.plot([rrt.nearestNode.locationX, new[0]], 
                                                                   [rrt.nearestNode.locationY, new[1]], 
                                                                   'bo', markersize=2, linestyle="-", 
                                                                   linewidth=0.5, alpha = 0.5)
        rrt.reWire(newNode) 
        if (rrt.goalFound(new)):
            # rrt.addChild(new[0], new[1], new[2], i)
            # print("Goal found")
            rrt.resetNearestValues()
            rrt.Nears(rrt.randomTree, [rrt.goal.locationX, rrt.goal.locationY,rrt.goal.angle], rrt.step_size)
            rrt.chooseParent([rrt.goal.locationX, rrt.goal.locationY,rrt.goal.angle])
            rrt.goal.parent = rrt.nearestNode
            dist_to_goal = rrt.distance(rrt.nearestNode, [rrt.goal.locationX,rrt.goal.locationY,rrt.goal.angle])
            rrt.goal.cost = rrt.nearestNode.cost + dist_to_goal
            print(rrt.goal.cost)
            # break

rrt.resetNearestValues()
rrt.Nears(rrt.randomTree, [rrt.goal.locationX, rrt.goal.locationY, rrt.goal.angle], rrt.step_size)
rrt.chooseParent([rrt.goal.locationX, rrt.goal.locationY, rrt.goal.angle])
rrt.goal.parent = rrt.nearestNode
dist_to_goal = rrt.distance(rrt.nearestNode, [rrt.goal.locationX, rrt.goal.locationY, rrt.goal.angle])
rrt.goal.cost = rrt.nearestNode.cost + dist_to_goal

print([rrt.nearestNode.locationX,  rrt.nearestNode.locationY, rrt.goal.angle], rrt.goal.cost, dist_to_goal)

rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0,start)
print("Number of waypoints: ", rrt.numWaypoints)
print("Path distance: ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints)
'''------------------------------------------------------------------------'''
end_time = time.time()
code_time = round(end_time - start_time, 2)

print(f"Time: {code_time} seconds")




'''------------------------------ANIMATION---------------------------------'''  
for i in range(len(rrt.Waypoints)-1):
    plt.plot(
        [rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]],
        [rrt.Waypoints[i][1],rrt.Waypoints[i+1][1],],
        'ro', linestyle="-", markersize=5, linewidth=2,
        )  
    startX, startY, startAngle = rrt.Waypoints[i][0], rrt.Waypoints[i][1], np.radians(rrt.Waypoints[i][2])
    endX, endY, endAngle = rrt.Waypoints[i+1][0], rrt.Waypoints[i+1][1], np.radians(rrt.Waypoints[i+1][2]) 
    for j in range(stepSize+1): 
        if (j % 5) == 0:
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
            plt.pause(0.1) 
            # L.remove()
            # F.remove() 
L, = plt.plot(x, y, color='black') 
F, = plt.fill(x, y, color='purple', alpha=0.5)


for i in range(10):
    print(i)
    plt.pause(1)

# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap4/RRTstar2DML_maze'+str(rrt.goal.cost)+'.pdf', bbox_inches='tight', pad_inches=0)

if draw_prediction:
    for i in range(len(rrt.obsSpace)): 
        plt.plot(rrt.obsSpace[i][0],rrt.obsSpace[i][1], alpha=0.3, mec='none', color = 'purple', marker = 'o', markersize = 15)
    for i in range(len(rrt.freeSpace)): 
        plt.plot(rrt.freeSpace[i][0],rrt.freeSpace[i][1], alpha=0.3, mec='none', color = 'green', marker = 'o', markersize = 15)

# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap4/RRTstar2DML_learning'+str(rrt.goal.cost)+'.pdf', bbox_inches='tight', pad_inches=0)
plt.pause(30)

plt.imshow(grid, cmap='binary')
# plt.colorbar()  # Add a colorbar for reference (optional)
plt.show()
# quit()
'''-------------------------------------------------------------------------'''
