import numpy as np
# import matplotlib
# matplotlib.use('tkAgg')
import matplotlib.pyplot as plt
import random
import time

from scipy.stats import gaussian_kde

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

        self.obsSpace = []
        self.freeSpace = []

    #add the point to the nearest node and add goal when reached
    def addChild(self, locationX, locationY, idx):
        tempNode = treeNode(locationX, locationY)
        self.nearestNode.children.append(tempNode)
        tempNode.parent = self.nearestNode 
        tempNode.cost = self.nearestNode.cost + self.distance(self.nearestNode, [locationX,locationY])
        self.allNodes[idx] = tempNode
        # if self.finish:  
        #     self.goal.parent = tempNode
        #     return self.goal
        return tempNode
    
    #sample a random point within grid limits
    def sampleAPoint(self):
        if np.random.rand() < 0.5:
            point = np.array([self.goal.locationX,self.goal.locationY]) 
        else: 
            x = random.randint(1, self.grid.shape[1]-1)
            y = random.randint(1, self.grid.shape[0]-1)
            point = np.array([x, y])
        return point

    #steer a distance stepsize from start to end location
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.step_size * self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]-1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0]-1
        return point
    
    #check if obstacle lies between the start node and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        # return False
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0]) 
        for i in range(int(round(self.distance(locationStart,locationEnd),0))):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            if np.int64(round(testPoint[1])) >= self.grid.shape[0] or np.int64(round(testPoint[0])) >= self.grid.shape[1]:
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
        return round(dist,2)
    
    #check if the goal has been reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= 10:
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
        currentPont = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0,currentPont)
        self.path_distance += self.step_size
        self.retraceRRTPath(goal.parent)

    def Nears(self, root, point, radius):
        if not root: 
            return
        dist = self.distance(root, point)
        if dist <= radius:
            # if self.distance(root, [self.goal.locationX,self.goal.locationY]) > stepSize:
            self.nearestNodes.append(root)
            # else:
            #     print("awdf") 
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
        return -1

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
        S = self.grid.shape[0] * self.grid.shape[0]
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if self.grid[i][j] == 1:
                    obsCount += 1 
        obstacleProbability = obsCount/S
        return obstacleProbability
    
    #if point is on Obstacle, not edge like in function isInObstacle
    def OnObstacle(self, x): 
        if np.int64(round(x[1])) >= self.grid.shape[0] or np.int64(round(x[0])) >= self.grid.shape[1]:
            return True
        if self.grid[np.int64(round(x[1])), np.int64(round(x[0]))] == 1:
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
        while len(self.obsSpace) <= 10 or len(self.freeSpace) <= 10: 
            x = self.sampleAPoint() 
            if self.OnObstacle(x):
                self.appendSpace(x,"obs") 
            else:
                self.appendSpace(x,"free") 
            # elif not np.allclose(x, np.array([self.goal.locationX,self.goal.locationY])):
            #     self.appendSpace(x,"free")   
        # print(self.obsSpace)
        # plot_density(self.obsSpace)
        # plot_density(self.freeSpace)
        # quit()
    
    def sampleDensity(self): 
        q_free = 0
        q_obs = 1  
        while q_free < q_obs: 
            x_rand = self.sampleAPoint()
            P_free = len(self.freeSpace) / (len(self.freeSpace) + len(self.obsSpace))
            P_obs = 1-P_free 
            b_free = self.GKDE(x_rand, np.array(self.obsSpace))
            b_obs = self.GKDE(x_rand, np.array(self.freeSpace)) 
            q_free = b_free * P_free
            q_obs = b_obs * P_obs 
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
        while self.OnObstacle(x):
            cnt+=1
            self.appendSpace(x, "obs")
            x = self.sampleDensity()
            # print(cnt)
            if cnt > 1:
                x = self.sampleAPoint()
                break
        # print(cnt)
        if not self.OnObstacle(x):
            self.appendSpace(x,"free") 
        return x

    
    # def Classifier(self, b_obs, b_free):
    #     P_free = len(self.freeSpace) / (len(self.freeSpace) + len(self.obsSpace))
    #     P_obs = 1-P_free
    #     if P_free*b_free >= P_obs*b_obs:
    #         return 0 #collision free
    #     else:
    #         return 1 #on obstacle

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
 