# programming lpa star (later lol)

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
        self.rhs = 0 

class Edge:
    def __init(self):
        self.source_state = State()
        self.target_state = State()


class LPASTAR:
    def __init__(self):
        self.start = State()
        self.goal = State()
        self.stateId = 0

    def calculate_L2(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2 + (y2-y1)**2)

    def calcDist(self, state1, state2):
        x1 = state1.x
        y1 = state1.y
        x2 = state2.x
        y2 = state2.y
        return self.calculate_L2(x1,y1,x2,y2)

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

        self.goal.g = self.calcDist(self.start, self.goal)
        self.goal.h = 0
        self.goal.f = self.goal.g + self.goal.h
        print("ENVIRONMENT READING fHat goal", self.start.f)
        print("ENVIRONMENT READING fHat goal", self.goal.f)




    # FOLLOWING D*-LITE PAPER
    def CalculateKey(self, state):
        # eventually the real key has to go here
        return state.f
    def Initialize(self):
        self.U = []             # {02}
        # {03} we did when we generate the state
        self.start.rhs = 0      # {04}
        heapq.heappush(self.U, (self.CalculateKey(self.start), self.start)) #{05}
        print("self.U", self.U)

    def UpdateVertex(self, u):
        pass
    def ComputeShortestPath(self):
        print("self.U top value", self.U)
        while  (self.U[0][0] < self.CalculateKey(self.goal)) or (self.goal.rhs != self.goal.g):
            u = heapq.heappop(self.U)[1]
            if u.g > u.rhs:
                u.g = u.rhs
                #update vertex someday
                #self.UpdateVertex(u)
            else:
                u.g = inf
                #{16}
            pass

    def LPASTAR_MAIN(self):
        if u d!= self.start
        self.Initialize()
        self.ComputeShortestPath()
if __name__ == "__main__":
    LPA = LPASTAR()
    LPA.readEnvironment("environment50_3.txt")
    LPA.LPASTAR_MAIN()
