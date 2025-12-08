import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single polygon 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()

    # Helper - find distance between two points
    def distance(point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    

    # Helper - find nearest point along a segment considering newest point
    def nearestPointOnSegment(newPoint, point1, point2):
        # if they are the same just return
        if point1 == point2:
            return point1
       
        x0, y0 = newPoint
        x1, y1 = point1
        x2, y2 = point2
        
        # point1->point2 vector
        dx = x2 - x1
        dy = y2 - y1

        # Take dot to find placement of newPoint
        dot = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)
        
        # Make sure dot is in [0, 1] (0 means point1, 1 means point2)
        dot = max(0, min(1, dot))

        # Find nearest point on segment using parametric equation
        nearestX = x1 + dot * dx
        nearestY = y1 + dot * dy

        return (nearestX, nearestY)
        
    
    # Go through all points and build RRT
    nextAvailableID = 1
    for i in range(1, len(points)+1):
        current = points[i]
        if i == 1:
            newPoints[nextAvailableID] = current
            adjListMap[nextAvailableID] = []
            nextAvailableID += 1
        else:
            # Find nearest point 
            nearestIndex = -1
            nearestPoint = None
            nearestDist = float('inf')
            for j in newPoints.keys():
                thisPoint = newPoints[j]
                dist = distance(current, thisPoint)
                if dist < nearestDist:
                    nearestDist = dist
                    nearestPoint = thisPoint
                    nearestIndex = j

            # find second nearest point adjacent to nearest points
            secondNearestPoint = nearestPoint
            secondNearestDist = float('inf')
            secondNearestIndex = -1

            for j in adjListMap[nearestIndex]:
                thisPoint = newPoints[j] 
                dist = distance(current, thisPoint)

                if dist < secondNearestDist:
                    secondNearestDist = dist
                    secondNearestPoint = thisPoint
                    secondNearestIndex = j
                        
            # Find point on segment from nearestPoint to secondNearestPoint
            newPoint = nearestPointOnSegment(current, nearestPoint, secondNearestPoint)

            # Find if point is already in newPoints
            exists = False
            closestIndex = -1
            for j in newPoints.keys():
                currentPoint = newPoints[j]
                print (currentPoint, newPoint)
                if currentPoint == newPoint:
                    exists = True
                    closestIndex = j
                    break
            

            # FROM HERE IS UNFINISHED

            # Create adjaceny matrix
            if exists:
                current = newPoints[closestIndex]
                adjListMap[closestIndex].append(i)
                adjListMap[nextAvailableID] = []
                adjListMap[nextAvailableID].append(closestIndex)
                nextAvailableID += 1
            else:
                # Add a new point
                newPoints[nextAvailableID]
                adjListMap[nextAvailableID] = []

    print (newPoints)
    print ("")
    print (adjListMap)
    
    return newPoints, adjListMap

'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
    path = []
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    
    return path

'''
Display the RRT and Path
'''
def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):
    
    # Your code goes here
    # You could start by copying code from the function
    # drawProblem and modify it to do what you need.
    # You should draw the problem when applicable.
    return 

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.
    
    return False

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    tree = dict()
    path = []
    # Your code goes here.
    
    return points, tree, path

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print ("Five arguments required: python rrt.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print ("Robot:")
    print (str(robot))
    print ("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print (str(obstacles[p]))
    print ("")

    # Visualize
    robotStart = []
    robotGoal = []

    for i in range(0, len(robot)):
        robotStart.append((robot[i][0] + x1, robot[i][1] + y1))
        robotGoal.append((robot[i][0] + x2, robot[i][1] + y2))
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print ("")
    print ("The input points are:")
    print (str(points))
    print ("")
    
    points, adjListMap = growSimpleRRT(points)

    # Search for a solution  
    path = basicSearch(adjListMap, 1, 20)    

    # Your visualization code 
    displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 



