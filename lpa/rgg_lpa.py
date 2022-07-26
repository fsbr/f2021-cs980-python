# i really want the V, E architechture
import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches

inf = np.Inf
class Visualizer:
    def __init__(self, planningInstance):
        print("Visualizer")
        self.planningInstance = planningInstance
        self.obsMap = planningInstance.obs
        self.yMax = planningInstance.yMax

    def plotMotionTree(self):
        fig, ax = plt.subplots()

        for idx in self.planningInstance.solution1:
            pathRect = patches.Rectangle((np.floor(idx[0]),np.floor(self.yMax - idx[1]-1)), 1,1, linewidth=1, edgecolor="k",facecolor="m")
            ax.add_patch(pathRect)
        countY = 0
        for i in self.obsMap:
            countX = 0
            for j in i:
                if j == 1:
                    rect = patches.Rectangle((np.floor(countX),np.floor(self.yMax - countY-1)), 1,1, linewidth=1, edgecolor="k",facecolor="k")
                    ax.add_patch(rect) 
                countX+=1
            countY +=1

        for idx in self.planningInstance.solution2:
            pathRect = patches.Rectangle((np.floor(idx[0]),np.floor(self.yMax - idx[1]-1)), 1,1, linewidth=1, edgecolor="k",facecolor="y")
            ax.add_patch(pathRect)
        startRect = patches.Rectangle((np.floor(self.planningInstance.start.iX),np.floor(self.yMax - self.planningInstance.start.iY-1)), 1,1, linewidth=1, edgecolor="k",facecolor="c")
        ax.add_patch(startRect)
        goalRect = patches.Rectangle((np.floor(self.planningInstance.goal.iX),np.floor(self.yMax - self.planningInstance.goal.iY-1)), 1,1, linewidth=1, edgecolor="k",facecolor="r")
        ax.add_patch(goalRect)
            
        plt.title("A* Planning Instance")
        plt.xlim((0,50))
        plt.ylim((0,50))
        plt.axis("equal")
        plt.grid()
        plt.show()

class State():
    def __init__(self):
        self.x = inf
        self.y = inf
        self.g = inf
        self.h = inf
        self.rhs = inf
        self.f = inf

        self.iX = inf
        self.iY = inf

        #flags
        self.isStart = False
        self.isGoal = False

        # gathering the shortest path
        self.cameFromIdx = ()
        self.cameFromCoord = ()
        

class Edge():
    def __init__(self):
        self.source_state = State()
        self.target_state = State()
        self.edgeCost = inf

class LPASTAR:
    def __init__(self):
        # v,e are the full representation
        self.V = {}
        self.E = {}

        # motion e, v are the states that we consider as part of the search frontier
        self.motionE = {}
        self.motionV = {}
    
        self.start = State()
        self.goal = State()

        # init for readEnvironment
        self.envFileList = []

        # tiebreak the heap in order that you enqueue stuff
        self.stateId = 0

        # changed edges
        self.changedEdges = []

    def calculate_L2(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2 + (y2-y1)**2)

    def calcDist(self, state1, state2):
        x1 = state1.x
        y1 = state1.y
        x2 = state2.x
        y2 = state2.y
        return self.calculate_L2(x1,y1,x2,y2)

    def convertToIdx(self, x,y):
        return np.floor(x), np.floor(self.yMax - y)

    def convertToCoordinate(self, xIdx, yIdx):
        return "unfinished function"

    def readEnvironment(self, envFile, updated = False):
        self.envFileList.append(envFile)
        A = []
        f = open(envFile)
        for x in f:
            print(x.rstrip("\n"))
            A.append(x.rstrip("\n"))
        print(A)
        self.xMax = int(A[0])
        self.yMax = int(A[1])
        print(self.xMax) 
        print(self.yMax) 
        self.obs = np.zeros((self.xMax, self.yMax))
        for j in range(self.yMax):
            for i in range(self.xMax):
                if A[2+j][i] == "-":
                    self.obs[j][i] = 0
                else:
                    self.obs[j][i] = 1

        # i think i need a copy of the original maps to make sure that I can compare edge costs
        if updated == False:
            # stuff we only want to do on the first time
            self.obs1 = self.obs
            print("self.obs")
            print(self.obs)
            self.start.x = float(A[2+self.yMax])
            self.start.y = float(A[3+self.yMax])
            self.goal.x = float(A[4+self.yMax])
            self.goal.y = float(A[5+self.yMax])

            print(self.start.x)
            print(self.start.y)
            print(self.goal.x)
            print(self.goal.y)
            
            #convert the start x to nearest 0.5
            self.start.x = np.floor(self.start.x) + 0.5
            self.start.y = np.floor(self.start.y) + 0.5
            self.goal.x = np.floor(self.goal.x) + 0.5
            self.goal.y = np.floor(self.goal.y) + 0.5

            print("centered start and goal to ")
            print(self.start.x)
            print(self.start.y)
            print(self.goal.x)
            print(self.goal.y)

            
            self.start.g = 0
            self.start.h = self.calcDist(self.start,self.goal)
            self.start.f = self.start.g + self.start.h
            self.start.rhs = 0
            self.start.isStart = True

            # a difference between g and ghat
            self.goal.g = inf #self.calcDist(self.start, self.goal)
            self.goal.h = 0
            self.goal.f = self.goal.g + self.goal.h
            self.start.isGoal = True
            print("ENVIRONMENT READING fHat goal", self.start.f)
            print("ENVIRONMENT READING fHat goal", self.goal.f)

            # all the below was originally not in the thing 
            #self.Grid = np.empty((self.xMax, self.yMax)) 
            self.Grid = [[State() for x in range(self.xMax)] for y in range(self.yMax)]
            # then i have to put the start and goal at the correct index
            sX, sY = self.convertToIdx(self.start.x, self.start.y)
            gX, gY = self.convertToIdx(self.goal.x, self.goal.y)
            self.sX = sX
            self.sY = sY
            self.gX = gX
            self.gY = gY

            self.Grid[int(sY)][int(sX)] = self.start
            self.Grid[int(gY)][int(gX)] = self.goal

            self.start.iX = sX
            self.start.iY = sY
            self.goal.iX = sX
            self.goal.iY = gY
        else:
            print("I SET SELF.OBS 2") 
            self.obs2 = self.obs
            print(self.obs2)


    def convertGridToGraph(self):
        # so what i want to do is convert my obstacle map to the V, E architecture
        # inputs are self.ob occupancy grid, and the output are the Vertices and Edges of the graph 
        # the states can be indexed by themselves I guess, but I really want the explicit representation
        # for finishing up LPA* in a timely manner
        # need to rewrite this in a smaller grid so i can actually see whats happening
        print("self.obs", self.obs)

        #connectivity 8
        #           N       S       E       W
        cardinals = [(0, -1), (0, 1), (1, 0), (-1, 0)]
        #               NE      SE      NW      SW
        interCardinals = [(1,-1), (1, 1), (-1, -1), (-1, 1) ]
        self.connectivity8 = cardinals + interCardinals
        #self.connectivity8 = #interCardinals

        # i think want to iterate over every cell in the grid, then add the state to V
        # and add its edges as neighbors in E
        #self.V[self.start] = self.start
        #self.V[self.goal] = self.goal
        for yIdx in range(self.yMax):
            for xIdx in range(self.xMax):
                #print(self.obs[yIdx][xIdx])
                stateToAdd = self.Grid[yIdx][xIdx]
                if (stateToAdd != self.start) or (stateToAdd !=self.goal):
                    stateToAdd.x = xIdx - 0.5
                    stateToAdd.y = self.yMax - yIdx + 0.5
                    #stateToAdd.h = self.calcDist(stateToAdd, self.goal)

                    # the heuristic is wrt to the start?!?! (in d*
                    stateToAdd.h = self.calcDist(stateToAdd, self.start)
                    #self.V[stateToAdd] = stateToAdd
                    #making my life easier
                    stateToAdd.iX = xIdx
                    stateToAdd.iY = yIdx
                    #self.V[stateToAdd] = stateToAdd
                    self.V[(xIdx, yIdx)] = stateToAdd
                    #print("self.V", self.V)

        # now that all the states are in there, assemble the edges
        for key, vertex in self.V.items():
            #print("vertex", vertex)
            xIdx = vertex.iX
            yIdx = vertex.iY
            #print("a new vertex")
            for direction in self.connectivity8:
                stepX = xIdx + direction[0]
                stepY = yIdx + direction[1]
                # if im inbounds

                # because we did this minus one here, we can afford to be equal to it
                if (0 <= stepX <= self.xMax -1) and (0<= stepY <= self.yMax -1): 
                    edgeToAdd = Edge() 
                    edgeToAdd.source_state = vertex
                    #for targetVertex in list(self.V):
                    #    if targetVertex.iX == stepX and targetVertex.iY == stepY:
                    targetVertex = self.V[(stepX, stepY)]
                    edgeToAdd.target_state = targetVertex
                    
                    if self.obs[stepY][stepX] ==1 or self.obs[yIdx][xIdx] == 1:
                        edgeToAdd.edgeCost = inf
                    else:
                        edgeToAdd.edgeCost = self.calcDist(vertex, targetVertex)
                    #self.E[edgeToAdd] = edgeToAdd
                    self.E[(xIdx, yIdx), (stepX, stepY)] = edgeToAdd
        print("goal in ", self.goal in self.V)
        print("start in", self.start in self.V)

    def updateEdgeCosts(self):
        # okay so anywhere the map changes, i need new edge costs and then i 
        print(self.obs1)
        print(self.obs2)
        changedIndices = []
        for y in range(self.yMax):
            for x in range(self.xMax):
                # obstacle changed, was added
                candidateTuple = [False, False] 
                candidateTuple[0] = bool(self.obs1[y][x]) ^ bool(self.obs2[y][x])  # i had ^ here before
                if candidateTuple[0] == True:
                    changedIndices.append((x,y)) 
                if self.obs1[y][x] == 1:
                    candidateTuple[1] = False
                else:
                    candidateTuple[1] = True
                self.changedMap[y][x] = candidateTuple            
        print(" 69 uncomment the line below me to see the actual map")
        print(self.changedMap)
        print("changed Indices List", changedIndices)

        for index in changedIndices:
            #print(self.V[index])
            for direction in self.connectivity8:
                stepX = index[0] + direction[0]
                stepY = index[1] + direction[1]

                # if im inbounds
                if (0 <= stepX <= self.xMax -1) and (0<= stepY <= self.yMax -1): 
                    #print("can i access an edge this way?", self.E[(index, (stepX, stepY))])
                    #print("can i access an edge this other  way?", self.E[((stepX, stepY), index)])
                    edge = self.E[(index, (stepX, stepY))]
                    reversedEdge = self.E[((stepX, stepY), index)]
                    
                    sourceIsObs = self.obs[index[1]][index[0]] == 1
                    targetIsObs = self.obs[stepY][stepX] == 1

                    if self.obs[index[1]][index[0]] == 1 or self.obs[stepY][stepX]:
                    #if sourceIsObs or targetIsObs:
                        edge.edgeCost = inf
                        reversedEdge.edgeCost = inf
                    #elif targetIsObs:
                    #    edge.edgeCost = inf
                    #    reversedEdge.edgeCost = inf
                    else:
                        edge.edgeCost = self.calcDist(edge.source_state, edge.target_state) 
                        reversedEdge.edgeCost = self.calcDist(edge.source_state, edge.target_state) 
                    self.changedEdges.append(edge)
                    self.changedEdges.append(reversedEdge)

    def CalculateKey(self, state):                                                                           
        # put that shit right on the state, then this function is easy                                       
        # so i need to be putting the g, rhs, and h values on the state when i update vertex                 
        # this assumes that that stuff is calculated on the state already when you go to look for it         
        # and that might not be true                                                                         
        g = state.g                                                                                          
        rhs = state.rhs                                                                                      
        h = state.h                                                                                          
        minItem = min(g,rhs)                                                                                 
        return [minItem + h, minItem]     

    def Initialize(self):
        self.changedMap = [[ [False, False] for x in range(self.xMax)] for y in range(self.yMax)]
        self.U = []         #{01}, {04 i do somewhere else  }
        startState = self.V[(self.start.iX, self.start.iY)]
        startKey = self.CalculateKey(startState)
        self.stateId += 1 
        heapq.heappush(self.U, (startKey,self.stateId, startState)) # {05}

    #def UpdateLaggingState(self):
    #    if self.utransfer != self.start:
    #        preds = self.getPred(self.utransfer) 
    def UpdateVertex(self, edge, secondCase = False):
        # my s is his u
        s = edge.target_state
        root = edge.source_state

        #print("root.g", root.g)
        #print("root == start", root == self.start)

        # this if s!=self.start is a hack!! :]
        #if s != self.start:
        #    #print("successor wasn't the start")
        #    tentativeRhs = edge.edgeCost + root.g
        #    #print("tentativeRhs", tentativeRhs)
        #    #print("edgeCost", edge.edgeCost)
        #    if tentativeRhs < s.rhs:
        #        s.rhs = tentativeRhs
        #        s.cameFromIdx = (root.iX, root.iY)
        #        s.cameFromCoord = (root.x, root.y)

        rhsList = []
        cameFromIdxList = []
        cameFromCoordList = []
        preds = self.getPred(s)

        #if secondCase == True:
        #    upreds = self.getPred(self.utransfer)
        #    
        #    print("upreds", upreds)
        #    print("preds before", preds)
        #    #preds.append(upreds)
        #    preds = preds + upreds
        #print("preds after", preds)

        for pred in preds:
            #print(pred)
            #if pred.target_state !=  self.start:
            tentativeRhs = pred.edgeCost + pred.source_state.g
            rhsList.append( tentativeRhs )
            cameFromIdxList.append( (pred.source_state.iX, pred.source_state.iY) )
            #cameFromIdxList.append( (s.iX, s.iY) )
            #if s == self.goal:
            #    print("DOING RHS FOR GOAL")
            #print("predecessor.edgeCost", predecessor.edgeCost)
            #print("pred.source_state.g", predecessor.source_state.g)
            #tentativeRhs = predecessor.edgeCost + predecessor.source_state.g
            #rhsList.append(tentativeRhs)
            #cameFromIdxList.append((predecessor.source_state.iX, predecessor.source_state.iY))
            #cameFromCoordList.append((predecessor.source_state.x, predecessor.source_state.y))
                
        if s != self.start:
            minRhs = min(rhsList)
            print("min Rhs", minRhs)
            print("from rhsList", rhsList)
            min_index = rhsList.index(minRhs)
            print("min index", min_index)
            print("cameFromIdxList", cameFromIdxList)
            s.rhs = minRhs
            s.f = s.rhs + s.h
            #s.g = inf
            s.cameFromIdx = cameFromIdxList[min_index]
            #s.cameFromCoordList = cameFromCoordList[min_index]
        else:   #s must be the start
            s.rhs = 0

        # this line once stopped me from terminating
        for keyStatePair in self.U:
            #print("keystatepair", keyStatePair)
            #print("keystatepair[2]", keyStatePair[2])
            if keyStatePair[2] == s:
                self.U.remove(keyStatePair)
        print("s.g", s.g)
        print("s.rhs", s.rhs)
        print("s.camefrom", s.cameFromIdx)

        if s.g != s.rhs:
            self.stateId+=1
            heapq.heappush(self.U, (self.CalculateKey(s), self.stateId, s) )
            print("enqueued onto U")
        print("finished update vertex")

    def getSucc(self,u):
        # given a state, return the successors 
        self.succs = []
        for key, edge in self.E.items():
            # only print these for debugging it makes everythin really slow
            #print("key", key)
            #print("edge", edge)
            if edge.source_state == u:
                self.succs.append(edge)
                #print("succs.edgeCost", edge.edgeCost)
        #print("self.succs", self.succs)
        return self.succs

    def getPred(self,u):
        self.succs = []
        for key, edge in self.E.items():
            if edge.target_state == u:
                self.succs.append(edge)
                #print("succs.edgeCost", edge.edgeCost)
        #print("self.succs", self.succs)
        return self.succs


    def bestU(self):
        # return the key in lex order unless the set is emtpy then return [inf, inf]
        if len(self.U) > 0:
            return self.U[0][0]
        else:
            return [inf,inf] 
    def ComputeShortestPath(self):
        #print("self.U", self.U)
        #print("self.U[0]", self.U[0])
        #while self.U[0][0] < self.CalculateKey(self.V[(self.goal.iX, self.goal.iY)]) or (self.V[(self.goal.iX, self.goal.iY)].rhs != self.V[(self.goal.iX, self.goal.iY)].g):
        while self.bestU() < self.CalculateKey(self.V[(self.goal.iX, self.goal.iY)]) or (self.V[(self.goal.iX, self.goal.iY)].rhs != self.V[(self.goal.iX, self.goal.iY)].g):
            #print("self.U", self.U)
            u = heapq.heappop(self.U)[2]
            #print("u", u)
            if u.g > u.rhs:
                print("first case")
                u.g = u.rhs
                self.succs = self.getSucc(u)
                #print("self.succs", self.succs)
                for edge in self.succs:
                    self.UpdateVertex(edge, True)
                    self.motionE[edge] = edge
                    #self.motionE[(edge.source_state, edge.target_state)] = edge
            else:
                print("second case")
                if u != self.start:
                    u.g = inf
                self.succs = self.getSucc(u)

                #edgeU = Edge()
                #edgeU.source_state = u
                #edgeU.target_state = u      # {union of {16}}
                #edgeU.edgeCost = 0
                # actually appending this loops the algorithm forever 
                # self.succs.append(edgeU)       # lets worry about this later
                self.utransfer = u
                for edge in self.succs:
                    
                    self.UpdateVertex(edge, True)
                    self.motionE[edge] = edge
                    #self.motionE[(edge.source_state, edge.target_state)] = edge

    def Main(self):
        self.Initialize()
        #print("before compute shortest, self.start.g", self.start.g)

        self.ComputeShortestPath()
        self.solution1 = []
        stateOfInterest = self.goal
        while stateOfInterest != self.start:
            self.solution1.append(stateOfInterest.cameFromIdx)
            try: 
                stateOfInterest = self.V[stateOfInterest.cameFromIdx]            
            except KeyError:
                print("NO PATH FOUND")
                self.solution1 = [(self.start.iX, self.start.iY)]
                self.solution1.append( (self.goal.iX, self.goal.iY))
                break
            print("i am stuck in the first solution path :[")
        print("solution path 1", self.solution1)

        #self.solution2 = self.solution1

        LPA.readEnvironment(fileList[1], True)
        LPA.convertGridToGraph()

        ##for all directed edges with changed edge costs
        ##update the edge costs c(u,v)
        LPA.updateEdgeCosts()
        # updatevertex(v, with root u)

        for edge in self.changedEdges:
            # updating costs 
            print("updating vertex because the map changed")
            self.UpdateVertex(edge)
        print("before the second compute shortest")

        ## u need to recompute the shortest path when the map changes
        self.ComputeShortestPath()
        self.solution2 = []
        stateOfInterest = self.goal

        howManyStates = 0
        while stateOfInterest != self.start:
            self.solution2.append(stateOfInterest.cameFromIdx)
            try:
                stateOfInterest = self.V[stateOfInterest.cameFromIdx]            
            except KeyError:
                print("NO PATH FOUND")
                self.solution2 = [ (self.start.iX, self.start.iY) ]
                self.solution2.append( (self.goal.iX, self.goal.iY))
            howManyStates+=1
            print("how Many States", howManyStates)
            print("stateOfInterest xi " +  str(stateOfInterest.iX) + " yi " + str(stateOfInterest.iY) )
            print("stateOfInterest x " +  str(stateOfInterest.x) + " y " + str(stateOfInterest.y) )
            
            print("where it came from" +  str(stateOfInterest.cameFromIdx) )
            print("where it came from is the start?" + str(stateOfInterest == self.start))
        print("solution path 2", self.solution2)
        # then smoosh that thing into bit*



if __name__ == "__main__":
    LPA = LPASTAR()
    #LPA.readEnvironment("../test_environments/grid_envs_changing10/environment10_A_69.txt")
    # using environment 50_A/B_95.txt
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_16.txt","../test_environments/grid_envs_changing/environment50_A_16.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_16.txt","../test_environments/grid_envs_changing/environment50_B_16.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_42.txt","../test_environments/grid_envs_changing/environment50_B_42.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_42.txt","../test_environments/grid_envs_changing/environment50_A_42.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_59.txt","../test_environments/grid_envs_changing/environment50_A_59.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_59.txt","../test_environments/grid_envs_changing/environment50_B_59.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_73.txt","../test_environments/grid_envs_changing/environment50_B_73.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_73.txt","../test_environments/grid_envs_changing/environment50_A_73.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_98.txt","../test_environments/grid_envs_changing/environment50_B_98.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_98.txt","../test_environments/grid_envs_changing/environment50_A_98.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_41.txt","../test_environments/grid_envs_changing/environment50_B_41.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_41.txt","../test_environments/grid_envs_changing/environment50_A_41.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_11.txt","../test_environments/grid_envs_changing/environment50_A_11.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_39.txt","../test_environments/grid_envs_changing/environment50_A_39.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_39.txt","../test_environments/grid_envs_changing/environment50_B_39.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_39.txt","../test_environments/grid_envs_changing/environment50_A_39.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_62.txt","../test_environments/grid_envs_changing/environment50_A_62.txt"]
    fileList = ["../test_environments/grid_envs_changing/environment50_B_51.txt","../test_environments/grid_envs_changing/environment50_A_51.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_B_72.txt","../test_environments/grid_envs_changing/environment50_A_72.txt"]
    
    #fileList = ["snake_A.txt", "snake_B.txt"]
    #fileList = ["snake_B.txt", "snake_A.txt"]
    #fileList = ["snake_A.txt", "snake_C.txt"]
    #fileList = ["snake_A.txt", "snake_D.txt"]
    #fileList = ["snake_D.txt", "snake_A.txt"]
    #LPA.readEnvironment("../test_environments/grid_envs_changing/environment50_B_16.txt")
    LPA.readEnvironment(fileList[0])

    # too slow to do this one
    #LPA.readEnvironment("../test_environments/grid_envs1000/environment1000_0.txt")
    #LPA.readEnvironment("../snake.txt")
    #LPA.readEnvironment("snake_B.txt")
    LPA.convertGridToGraph()
    print("CALLING MAIN")
    LPA.Main()
    for key, edge in LPA.E.items():
        if edge.target_state == LPA.V[(6,0)]:
            
            print("INCOMING TO PROBLEM VERTEX")
            print("g 6,0", edge.source_state.g)
            print("rhs 6,0", edge.source_state.rhs)
        if edge.source_state == LPA.V[(6,0)]:
            print("OUTGOING FROM PROBLEM VERTEX")
            print("g source  6,0", edge.target_state.g)
            print("rhs 6,0", edge.target_state.rhs)
    print("LPA.solution1", LPA.solution1)
    gv = Visualizer(LPA)
    gv.plotMotionTree()

    
