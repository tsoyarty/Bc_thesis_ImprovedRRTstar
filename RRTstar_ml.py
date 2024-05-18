import numpy as np 
import matplotlib.pyplot as plt 
import time

from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif' 
plt.rcParams['font.size'] = 10 

from RRTstar_ml_Func import RRTalgo, treeNode
from my_utils import fill_polygon, read_maze_file 



#===============================================================================================
#=======================================PARAMETERS==============================================
#===============================================================================================
rows, cols = 150, 150
grid = np.zeros((rows, cols))   

start, goal, Obstacles = read_maze_file("Maze_clutter")
for poly in Obstacles:
    fill_polygon(grid, poly)

grid[int(start[0])][int(start[1])] = 0
grid[int(goal[0])][int(goal[1])] = 0

numIterations = 5000
stepSize = 10
goalRegion = plt.Circle((goal[0], goal[1]), 10, color='r', fill = False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'bo')
plt.plot(goal[0], goal[1], 'ro')
ax = fig.gca()
ax.add_patch(goalRegion)
# plt.xlabel('X')
# plt.ylabel('Y')
plt.xticks([])
plt.yticks([])
plt.xlim(0,grid.shape[0])
plt.ylim(0,grid.shape[1])


#===============================================================================================
#==========================================MAIN=================================================
#===============================================================================================
rrt = RRTalgo(start, goal, numIterations, grid, stepSize) 
rrt.adaptiveSampling() 

# rrt.KDE(np.array([20,20]), [np.array([20,20]),np.array([20,20])])
# quit()
nodes = [treeNode(start[0],start[1])]

start_time = time.time()
# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap4/SimpleMaze.eps', bbox_inches='tight', pad_inches=0)
# quit()
# plt.pause(10)
draw_prediction = False
for i in range(rrt.iterations):
    rrt.resetNearestValues()
    # print("Iteration: ",i)
    # point = rrt.sampleAPoint()
    point = rrt.mlSample()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    # if i==100 or i==250 or i==500 or i==1000 or i==1500 or i==2000 or i==2500 or i==5000:
        # plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap5/Maze_clutter_RRTstarML_learning2'+str(i)+'.pdf', bbox_inches='tight', pad_inches=0)   
        # plt.pause(1)
    if bool == False:
        rrt.Nears(rrt.randomTree, new, 40)
        rrt.chooseParent(new)   
        newNode = rrt.addChild(new[0], new[1], i)
        # if i%100==0:
            # print(f"RRTstar_ml: cost: {round(rrt.goal.cost,2)}; iterations: {i}")
            # plt.pause(0.1)
        # rrt.lineDict[(round(rrt.nearestNode.locationX,0),
        #               round(rrt.nearestNode.locationY,0), 
        #               round(new[0],0),round(new[1],0))] = plt.plot([rrt.nearestNode.locationX, new[0]], 
        #                                                            [rrt.nearestNode.locationY, new[1]], 
        #                                                            'bo', markersize=2, linestyle="-", 
        #                                                            linewidth=0.5, alpha = 0.5)
        rrt.reWire(newNode) 
        # plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'bo', linestyle="-", linewidth=0.5)
        
        # print(rrt.lineDict)
        if (rrt.goalFound(new)):
            rrt.step_size = 10
            # rrt.addChild(new[0],new[1], i)
            # print("Goal found")
            rrt.resetNearestValues()
            rrt.Nears(rrt.randomTree, [rrt.goal.locationX, rrt.goal.locationY], 40)
            rrt.chooseParent([rrt.goal.locationX, rrt.goal.locationY])
            rrt.goal.parent = rrt.nearestNode
            dist_to_goal = rrt.distance(rrt.nearestNode, [rrt.goal.locationX,rrt.goal.locationY])
            rrt.goal.cost = rrt.nearestNode.cost + dist_to_goal
            print(f"RRTstar_ml: cost: {round(rrt.goal.cost,2)}; iterations: {i}")
          
end_time = time.time()
code_time = round(end_time - start_time, 6)

print(f"Time: {code_time} seconds")


rrt.resetNearestValues()
rrt.Nears(rrt.randomTree, [rrt.goal.locationX, rrt.goal.locationY], 30)
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



#===============================================================================================
#=============================================GUI===============================================
#===============================================================================================
for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]],
             [rrt.Waypoints[i][1],rrt.Waypoints[i+1][1],], 
             'bo', linestyle="-", linewidth=3)
    # plt.plot(rrt.Waypoints[i][0], rrt.Waypoints[i][1], 'ro')
    plt.pause(0.1)

# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap4/RRTstar_maze'+str(rrt.goal.cost)+'.pdf', bbox_inches='tight', pad_inches=0)

plt.pause(10)

if draw_prediction:
    for i in range(len(rrt.obsSpace)): 
        plt.plot(rrt.obsSpace[i][0],rrt.obsSpace[i][1], alpha=0.3, mec='none', color = 'purple', marker = 'o', markersize = 15)
    for i in range(len(rrt.freeSpace)): 
        plt.plot(rrt.freeSpace[i][0],rrt.freeSpace[i][1], alpha=0.3, mec='none', color = 'green', marker = 'o', markersize = 15)

# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap4/RRTstarML_learning'+str(rrt.goal.cost)+'.pdf', bbox_inches='tight', pad_inches=0)

# plt.imshow(grid, cmap='binary')
# plt.colorbar()  # Add a colorbar for reference (optional)
# plt.savefig('RRTstarML3.eps', bbox_inches='tight', pad_inches=0)
plt.show()
# quit()