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
        countY = 0
        for i in self.obsMap:
            countX = 0
            for j in i:
                if j == 1:
                    rect = patches.Rectangle((np.floor(countX),np.floor(self.yMax - countY-1)), 1,1, linewidth=1, edgecolor="k",facecolor="k")
                    ax.add_patch(rect) 
                countX+=1
            countY +=1

        for idx in self.planningInstance.solution1:
            pathRect = patches.Rectangle((np.floor(idx[0]),np.floor(self.yMax - idx[1]-1)), 1,1, linewidth=1, edgecolor="k",facecolor="m")
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

    def readEnvironment(self, envFile,updated = False):
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
        else:
            self.obs2 = self.obs

        #self.Grid = np.empty((self.xMax, self.yMax)) 
        self.Grid = [[State() for x in range(self.xMax)] for y in range(self.yMax)]
        # then i have to put the start and goal at the correct index
        sX, sY = self.convertToIdx(self.start.x, self.start.y)
        gX, gY = self.convertToIdx(self.goal.x, self.goal.y)
        self.Grid[int(sY)][int(sX)] = self.start
        self.Grid[int(gY)][int(gX)] = self.goal

        self.start.iX = sX
        self.start.iY = sY
        self.goal.iX = sX
        self.goal.iY = gY

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
        connectivity8 = cardinals + interCardinals

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
                    stateToAdd.h = self.calcDist(stateToAdd, self.goal)
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
            for direction in connectivity8:
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
                    if self.obs[stepY][stepX] == 1:
                        edgeToAdd.edgeCost = inf
                    else:
                        edgeToAdd.edgeCost = self.calcDist(vertex, targetVertex)
                    self.E[edgeToAdd] = edgeToAdd

        print("goal in ", self.goal in self.V)
        print("start in", self.start in self.V)

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
        self.U = []         #{01}, {04 i do somewhere else  }
        startState = self.V[(self.start.iX, self.start.iY)]
        startKey = self.CalculateKey(startState)
        self.stateId += 1 
        heapq.heappush(self.U, (startKey,self.stateId, startState)) # {05}

    def UpdateVertex(self, edge):
        s = edge.target_state
        root = edge.source_state
        #print("root.g", root.g)
        #print("root == start", root == self.start)

        if s != self.start:
            #print("successor wasn't the start")
            tentativeRhs = edge.edgeCost + root.g
            #print("tentativeRhs", tentativeRhs)
            #print("edgeCost", edge.edgeCost)
            if tentativeRhs < s.rhs:
                s.rhs = tentativeRhs
                s.cameFromIdx = (root.iX, root.iY)
                s.cameFromCoord = (root.x, root.y)


        # this line once stopped me from terminating
        for keyStatePair in self.U:
            #print("keystatepair", keyStatePair)
            if keyStatePair[2] == s:
                self.U.remove(keyStatePair)
            
        print("s.g", s.g)
        print("s.rhs", s.rhs)

        # this feels like a hack
        if s.g != s.rhs:
            self.stateId+=1
            heapq.heappush(self.U, (self.CalculateKey(s), self.stateId, s) )
            print("enqueued onto U")
        print("finished update vertex")

    def getSucc(self,u):
        # given a state, return the successors 
        self.succs = []
        for edge in list(self.E):
            if edge.source_state == u:
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
            print("self.U", self.U)
            u = heapq.heappop(self.U)[2]
            #print("u", u)
            if u.g > u.rhs:
                print("first case")
                u.g = u.rhs
                self.succs = self.getSucc(u)
                #print("self.succs", self.succs)
                for edge in self.succs:
                    self.UpdateVertex(edge)
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
                for edge in self.succs:
                    self.UpdateVertex(edge)
                    self.motionE[edge] = edge
                    #self.motionE[(edge.source_state, edge.target_state)] = edge

    def Main(self):
        self.Initialize()
        #print("before compute shortest, self.start.g", self.start.g)
        self.ComputeShortestPath()

        #print("goal came from idx", self.goal.cameFromIdx)
        self.solution1 = []
        stateOfInterest = self.goal

        while stateOfInterest != self.start:
            self.solution1.append(stateOfInterest.cameFromIdx)
            stateOfInterest = self.V[stateOfInterest.cameFromIdx]            
        print("solution path", self.solution1)

        # wait for changes in the edge costs
        # i force that, and then i do that cool XOR thing here
        # then i smoosh that into bit*



if __name__ == "__main__":
    LPA = LPASTAR()
    #LPA.readEnvironment("../test_environments/grid_envs_changing10/environment10_A_69.txt")
    # using environment 50_A/B_95.txt
    #LPA.readEnvironment("../test_environments/grid_envs_changing/environment50_A_79.txt")

    # too slow to do this one
    #LPA.readEnvironment("../test_environments/grid_envs1000/environment1000_0.txt")
    #LPA.readEnvironment("../snake.txt")
    LPA.readEnvironment("snake_B.txt")
    LPA.convertGridToGraph()
    LPA.Main()

    gv = Visualizer(LPA)
    gv.plotMotionTree()

    
