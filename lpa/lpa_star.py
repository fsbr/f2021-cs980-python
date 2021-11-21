# programming lpa star (later lol)
# this implementatin only needs to work in a grid
# i can feel my skills increasing lol
# tckf

import numpy as np
import heapq

# lets plan on a 50x50
world = np.zeros((50,50))
print(world)
inf = np.Inf

class State:
    def __init__(self):
        self.x = inf
        self.y = inf
        self.g = inf
        self.h = inf
        self.f = inf
        self.rhs = inf

        # for a grid, I guess its the index where you'd find the state
        self.cameFrom = ()
        self.edgeCost = inf

        # for grid worlds
        self.iX = inf
        self.iY = inf

        # flags
        self.isStart = False
        self.isGoal = False 
        self.explored = False

class Edge:
    def __init(self):
        self.source_state = State()
        self.target_state = State()
        self.edgeCost = inf

class LPASTAR:
    def __init__(self):
        self.start = State()
        self.goal = State()
        self.stateId = 0
        self.V = {}
        self.E = {}
        self.envFileList = []
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
            self.start.g = 0
            self.start.h = self.calcDist(self.start,self.goal)
            self.start.f = self.start.g + self.start.h
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


    def getSucc(self, u):
        # ALL this should do is append the successor states
        connectivity = 8 #means check the intercardinals and the cardinals
        #           N       S       E       W
        cardinals = [(0,-1), (0,1), (1,0), (-1,0)] 
        #               NE      SE      SW          NW
        interCardinals = [(-1, 1), (1,1), (1, -1), (-1,-1)]
        connectivity8 = cardinals + interCardinals
        print("connectivity 8 ", connectivity8)
        succ = []               # list of successors

        for direction in connectivity8:
            print("direction", direction) 
            stepX = u.iX + direction[0]
            stepY = u.iY + direction[1]

            # i think this line is checking if i'm in the boundary
            if (0 <= stepX < self.xMax -1) and (0<= stepY < self.yMax -1):
                if self.obs[stepY][stepX] == 0:
                    stateToAdd = self.Grid[stepY][stepX]
                    # add the successors to the list
                    # let's worry about edges later
                    #stateToAdd = State()
                    print("debug stepX", stepX)
                    print("debug stepY", stepY)
                    stateToAdd.iX = stepX
                    stateToAdd.iY = stepY
                    stateToAdd.x = stepX - 0.5
                    stateToAdd.y = self.yMax - stepY + 0.5

                    ## i dont think h changes as long as the goal choice doesn't change
                    stateToAdd.h = self.calcDist(stateToAdd, self.goal)

                    succ.append(stateToAdd)
                else:
                    print("state has a collision")
            else:
                print("state out of bounds")
        print("finished collision check")
        # i think this part is right
        # and stateToAdd is a by reference assignment so it should also change the copy in the grid
        return succ

    def UpdateVertex(self, s, root):
        # should JUST do the update vertex process on a SINGLE state
        # u is the root in that case
        #print("state in U",s)
        if s.explored == False:
            if s != self.start:
                print("s was not equal to self.start")
                print("distance between root and state", self.calcDist(root,s))
                print("s.g", s.g)
                edgeCost = self.calcDist(root,s)
                #tentativeRhs = root.g + self.calcDist(root,s)
                tentativeRhs = root.g + edgeCost
                print("tentativeRhs", tentativeRhs)
                print("s.rhs", s.rhs)
                if tentativeRhs < s.rhs:
                    s.rhs = tentativeRhs
                    s.edgeCost = edgeCost
                    s.cameFrom = (root.iX,root.iY)

            # lets remove it later
            print("in UpdateVertex, before pushing state")
            print("s.g", s.g)
            print("s.rhs", s.rhs)
            for keyStatePair in self.U:
                if keyStatePair[2] == s:
                    self.U.remove(keyStatePair)
            if s.g != s.rhs:
                self.stateId+=1
                # in order for the call to CalculateKey to be valid, I need to make sure that it has 
                # g, h, rhs, which it looks like (at least in the one that i check), it does
                heapq.heappush(self.U, (self.CalculateKey(s), self.stateId, s) )


    def TopKey(self, U):
        # returns smallest prioirty in the queu or inf U is empty
        if len(U) == 0:
            return [inf,inf]
        else:
            return U[0][0]

    # FOLLOWING D*-LITE PAPER
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
        self.U = []             # {02}
        # {03} we did when we generate the state
        self.start.rhs = 0      # {04}
        print("self.U", self.U)
        sX,sY = self.convertToIdx(self.start.x, self.start.y)
        gX,gY = self.convertToIdx(self.goal.x, self.goal.y)
        self.start.iX = int(sX)
        self.start.iY = int(sY)
        self.goal.iX = int(gX)
        self.goal.iY = int(gY)
        print("sX" + str(sX) + " sY " + str(sY))
        print("gX" + str(gX) + " gY " + str(gY))

        self.Grid[self.start.iY][self.start.iX] = self.start
        self.Grid[self.goal.iY][self.goal.iX] = self.goal 
        #print(self.Grid)
        heapq.heappush(self.U, (self.CalculateKey(self.start), self.stateId, self.start)) #{05}

    def ComputeShortestPath(self):
        print("self.U top value", self.U)
        print("self.U top value, broken up", self.U[0][0])
        goalKey = self.CalculateKey(self.goal)
        print("goal Key", goalKey)

        debugCounter = 0
        # so this algorithm will terminate when the topkey is greater than the goal key
        while self.TopKey(self.U) < self.CalculateKey(self.goal) or (self.goal.rhs != self.goal.g):

        #while debugCounter < 3:
            print("self.CalculateKey(self.goal)", self.CalculateKey(self.goal))
            print("self.TopKey(self.U)", self.TopKey(self.U))
            print("self.goal.rhs", self.goal.rhs)
            print("self.goal.g")
            print("stateId ", self.stateId)
            u0 = heapq.heappop(self.U)
            u = u0[2]
            print("POPPED STATE")
            print("small u is",u)
            print("u.g", u.g)
            print("u.rhs", u.rhs)
            self.V[u] = u
            if u.g > u.rhs:
                print("we are in the first cae")
                #if u!=self.start:
                print("the vertex in question is not the start, so I'm setting u.g = u.rhs")
                u.g = u.rhs

                # for all succ(u)
                self.succs = self.getSucc(u) 
                for s in self.succs:
                    # i feel like i "need" this u term here
                    self.UpdateVertex(s, u)
            else:
                # the start state does tot his one.... not sure if thats supposed to happen or not, its just what IS happening
                print("we are in the second case")
                if u != self.start:
                    u.g = inf
                self.succs = self.getSucc(u) 
                for s in self.succs:
                    self.UpdateVertex(s, u)
            u.explored = True
            debugCounter+=1

    def updateEdgeCosts(self):
        print("I would be updating the edge costs here if I knew wtf I was doing")

        # okay this xor made me feel smart AF
        for y in range(self.yMax):
            for x in range(self.xMax):
                # obstacle changed, was added
                candidateTuple = [False, False] 
                candidateTuple[0] = bool(self.obs1[y][x]) ^ bool(self.obs2[y][x]) 
                if self.obs1[y][x] == 1:
                    candidateTuple[1] = False
                else:
                    candidateTuple[1] = True
                self.changedMap[y][x] = candidateTuple            
        print(self.changedMap)

    def LPASTAR_MAIN(self):
        self.Initialize()
        self.ComputeShortestPath()
        gridX = self.goal.iX
        gridY = self.goal.iY
        pathList = []
        pathList.append( (gridX, gridY))
        gridLocation = self.Grid[gridY][gridX]
        print("gridLocation.cameFrom", gridLocation.cameFrom)
        while gridLocation != self.start:
            newX = gridLocation.cameFrom[0]
            newY = gridLocation.cameFrom[1]
            pathList.append((newX,newY))
            gridLocation = self.Grid[newY][newX]
        print("Final Path", pathList)
        
        # wait for (or i guess induce) changes in edge cost
        LPA.readEnvironment("environment50_B_0.txt", True)
        self.updateEdgeCosts()

        # gameplan is basically 
        # update the edge cost c(u,v)
        # self.UpdateVertex(v,u)

if __name__ == "__main__":
    LPA = LPASTAR()
    #LPA.readEnvironment("environment50_3.txt")
    LPA.readEnvironment("environment50_A_0.txt")
    LPA.LPASTAR_MAIN()
    print("LPA.goal.g = ", LPA.goal.g)
    print("LPA maps", LPA.envFileList)
