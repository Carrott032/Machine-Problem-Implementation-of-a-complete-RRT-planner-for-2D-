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
    newPoints = dict(points)
    adjListMap = {}

    
    adjListMap[1] = []
    treeNodes = set([1]) 

    nextKey = max(newPoints.keys()) + 1

    def add_edge(a, b):
        adjListMap.setdefault(a, [])
        adjListMap.setdefault(b, [])
        if b not in adjListMap[a]:
            adjListMap[a].append(b)
        if a not in adjListMap[b]:
            adjListMap[b].append(a)

    for i in sorted(newPoints.keys()):
        if i == 1:
            continue  # root is already in tree

        p = newPoints[i]

        bestDistance = float('inf')
        bestVertex = None
        bestEdge = None #??
        bestPoint = None

        for index in treeNodes:
            coord = newPoints[index]
            d = (coord[0]-p[0])**2 + (coord[1]-p[1])**2
            if d < bestDistance:
                bestDistance = d
                bestVertex = index

        for u in adjListMap:
            for v in adjListMap[u]:
                if v <= u:
                    continue

                pu = np.array(newPoints[u])
                pv = np.array(newPoints[v])
                edgeVec = pv - pu
                length2 = np.dot(edgeVec, edgeVec)
                if length2 == 0:
                    continue

                t = np.dot(np.array(p) - pu, edgeVec) / length2
                t_clamped = max(0.0, min(1.0, t))
                proj = pu + t_clamped * edgeVec
                projCoords = (float(proj[0]), float(proj[1]))

                d2 = (projCoords[0]-p[0])**2 + (projCoords[1]-p[1])**2
                if d2 < bestDistance:
                    bestDistance = d2
                    bestEdge = (u, v, t_clamped)
                    bestPoint = projCoords
                    bestVertex = None  

        if bestEdge is None:
            closestVertex = bestVertex
            add_edge(i, closestVertex)
            treeNodes.add(i)
            continue  

        u, v, t = bestEdge

        if 0 < t < 1:
            newID = nextKey
            nextKey += 1

            newPoints[newID] = bestPoint
            adjListMap.setdefault(newID, [])

            if v in adjListMap.get(u, []):
                adjListMap[u].remove(v)
            if u in adjListMap.get(v, []):
                adjListMap[v].remove(u)

            add_edge(u, newID)
            add_edge(newID, v)

            add_edge(i, newID)

            treeNodes.add(newID)
            treeNodes.add(i)

        else:
            closestVertex = u if t == 0 else v
            add_edge(i, closestVertex)
            treeNodes.add(i)

    return newPoints, adjListMap

    
    
    

'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
    path = []
    
    queue = [start]
    visited = set([start])
    parent = {start: None}
    while queue:
        v = queue.pop(0)
        if v == goal:
            break

        i = 0
        neighbors = tree.get(v, [])

        while i < len(neighbors):
            n = neighbors[i]
            if n not in visited:
                visited.add(n)
                parent[n] = v
                queue.append(n)
            i += 1

    if goal not in visited:
        return []

    i = goal
    while i is not None:
        path.append(i)
        i = parent[i]
    
    i = 0
    j = len(path) - 1
    while i < j:
        path[i], path[j] = path[j], path[i]
        i += 1
        j -= 1

    print (path)

    return path
    


'''
Display the RRT and Path
'''
def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):
    _, axes = setupPlot()
    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green')
        axes.add_patch(patch)

    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red')
        axes.add_patch(patch)
    if polygons is not None:
        for p in polygons:
            patch = createPolygonPatch(p, 'gray')
            axes.add_patch(patch)

    for u, neighbors in tree.items():
        for v in neighbors:
            if u < v:  
                if u in points and v in points:
                    axes.plot(
                        [points[u][0] / 10.0, points[v][0] / 10.0],
                        [points[u][1] / 10.0, points[v][1] / 10.0],
                        color='black',
                    )

    if path and len(path) > 1:
        for i in range(len(path) - 1):
            u = path[i]
            v = path[i + 1]
            if u in points and v in points:
                axes.plot(
                    [points[u][0] / 10.0, points[v][0] / 10.0],
                    [points[u][1] / 10.0, points[v][1] / 10.0],
                    color='orange',
                )
    plt.show()
    return 

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.
    
    worldRobot = [(a + point[0], b + point[1]) for (a, b) in robot]

    for (x, y) in worldRobot:
        if x < 0.0 or x > 10.0 or y < 0.0 or y > 10.0:
            return False



    def insidePolygon(pt, poly):
        x, y = pt
        output = False

        for i in range(len(poly)):
            n = len(poly)
            if (poly[i][1] > y) != (poly[(i + 1) % n][1] > y):
                intersect = (poly[(i + 1) % n][0] - poly[i][0]) * (y - poly[i][1]) / (poly[(i + 1) % n][1] - poly[i][1] + 0.0) + poly[i][0]
                if x < intersect:
                    output = not output
        return output

    edges = []

    for i in range(len(worldRobot)):
        edges.append((worldRobot[i], worldRobot[(i + 1) % len(worldRobot)]))
    for o in obstacles:
        for v in worldRobot:
            if insidePolygon(v, o):
                return False
        for v in o:
            if insidePolygon(v, worldRobot):
                return False
        n_o = len(o)
        for i in range(n_o):
            c = o[i]
            d = o[(i + 1) % n_o]
            for (a, b) in edges:


                def cross(p, q, r):
                    return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

                def on_segment(p, q, r):
                    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

                if ((cross(a, b, c) == 0 and on_segment(a, c, b)) or (cross(a, b, d) == 0 and on_segment(a, d, b)) 
                or (cross(c, d, a) == 0 and on_segment(c, a, d)) or (cross(c, d, b) == 0 and on_segment(c, b, d))):
                    return False

                if (cross(a, b, c) > 0) != (cross(a, b, d) > 0) and (cross(c, d, a) > 0) != (cross(c, d, b) > 0):
                    return False
    return True

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    tree = dict()
    path = []

    points[1] = startPoint
    tree[1] = []
    nextKey = 2
    goalKey = None

        

    def noIntersect(q1, q2):
        changeX = q2[0] - q1[0]
        changeY = q2[1] - q1[1]
        distance = np.hypot(changeX, changeY)
        for i in range(max(1, int(distance / 0.2)) + 1):
            t = i / float(max(1, int(distance / 0.2)))
            if not isCollisionFree(robot, (q1[0] + t * changeX, q1[1] + t * changeY), obstacles):
                return False
        return True

    stepSize = 0.5

    for _ in range(5000):

        if np.random.rand() < 0.1:
            q_rand = goalPoint
        else:
            q_rand = (np.random.uniform(0.0, 10.0), np.random.uniform(0.0, 10.0))

        if not isCollisionFree(robot, q_rand, obstacles):
            continue

        closestID = None
        minDistance = float('inf')
        for index, coord in points.items():
            if (coord[0] - q_rand[0])**2 + (coord[1] - q_rand[1])**2 < minDistance:
                minDistance = (coord[0] - q_rand[0])**2 + (coord[1] - q_rand[1])**2
                closestID = index

        vec = np.array([q_rand[0] - points[closestID][0], q_rand[1] - points[closestID][1]])
        distance = np.linalg.norm(vec)
        if distance == 0.0:
            continue
        if distance > stepSize:
            vec = vec / distance * stepSize
        newNode = (points[closestID][0] + float(vec[0]), points[closestID][1] + float(vec[1]))

        if not noIntersect(points[closestID], newNode):
            continue

        newKey = nextKey
        nextKey += 1
        points[newKey] = newNode
        tree.setdefault(newKey, [])
        tree.setdefault(closestID, [])
        tree[closestID].append(newKey)
        tree[newKey].append(closestID)

        if noIntersect(newNode, goalPoint):
            goalKey = nextKey
            nextKey += 1
            points[goalKey] = goalPoint
            tree.setdefault(goalKey, [])
            tree[goalKey].append(newKey)
            tree[newKey].append(goalKey)
            path = basicSearch(tree, 1, goalKey)
            break


    robotStart = [(a + startPoint[0], b + startPoint[1]) for (a, b) in robot]
    robotGoal = [(a + goalPoint[0], b + goalPoint[1]) for (a, b) in robot]
    displayRRTandPath(points, tree, path, robotStart, robotGoal, obstacles)
    
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