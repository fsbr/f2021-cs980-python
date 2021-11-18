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

        # for grid worlds
        self.iX = inf
        self.iY = inf

        # flags
        self.isStart = False
        self.isGoal = False 
        self.explored = False

        #self.cameFrom

class Edge:
    def __init(self):
        self.source_state = State()
        self.target_state = State()


class LPASTAR:
    def __init__(self):
        self.start = State()
        self.goal = State()
        self.stateId = 0
        self.V = {}
        self.E = {}

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

    def readEnvironment(self, envFile):
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
        if s != self.start:
            print("s was not equal to self.start")
            print("distance between root and state", self.calcDist(root,s))
            print("s.g", s.g)
            tentativeRhs = root.g + self.calcDist(root,s)
            print("tentativeRhs", tentativeRhs)
            print("s.rhs", s.rhs)
            if tentativeRhs < s.rhs:
                s.rhs = tentativeRhs

        # lets remove it later
        print("in UpdateVertex, before pushing state")
        print("s.g", s.g)
        print("s.rhs", s.rhs)
        if s.g != s.rhs:
            self.stateId+=1
            if s.explored == False:
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
        # this assumes that teh 
        g = state.g
        rhs = state.rhs
        h = state.h
        minItem = min(g,rhs)
        return [minItem + h, minItem]

    def Initialize(self):
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

    def ComputeShortest(self):

if __name__ == "__main__":
    LPA = LPASTAR()
    LPA.readEnvironment("environment50_3.txt")
    LPA.LPASTAR_MAIN()
    print("LPA.goal.g = ", LPA.goal.g)
