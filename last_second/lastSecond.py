# JUST implementing an edge queu to do LPA*
import sys
sys.path.append("../")
import Visualizer
import heapq
import numpy as np
import random
import matplotlib.pyplot as plt
plt.style.use("seaborn-dark")
inf = np.Inf

class State():
    def __init__(self):
        self.x = 5 
        self.y = 5 
        self.gT = inf
        

class Edge():
    def __init__(self):
        self.source_state = State() 
        self.target_state = State()

class LAST:
    def __init__(self):
        # stuff for the environmnet
        self.xMin = 0.0
        self.xMax = 10.0

        self.yMin = 0.0
        self.yMax = 10.0

        self.start = State()
        self.start.x = 2
        self.start.y = 4

        self.goal = State()
        self.goal.x = 15
        self.goal.y = 12

        # map stuff
        self.obs = np.array([]) 
        # stuff for sampling
        self.m = 100
        self.Xsamples = {}

        # rgg stuff
        self.r = 2
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


    def readEnvironment(self, envFile, updated = False):
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
        
        if updated == False:
            self.obs1 = self.obs
            print("self.obs")
            print(self.obs)
            # our program has an 
            self.start.x = float(A[2+self.yMax])
            self.start.y = float(A[3+self.yMax])
            self.goal.x = float(A[4+self.yMax])
            self.goal.y = float(A[5+self.yMax])

            print(self.start.x)
            print(self.start.y)
            print(self.goal.x)
            print(self.goal.y)
            self.start.gHat = 0
            self.start.hHat = self.calcDist(self.start,self.goal)
            self.start.fHat = self.start.gHat + self.start.hHat

            self.goal.gHat = self.calcDist(self.start, self.goal)
            self.goal.hHat = 0
            self.goal.fHat = self.goal.gHat + self.goal.hHat
            print("ENVIRONMENT READING fHat goal", self.start.fHat)
            print("ENVIRONMENT READING fHat goal", self.goal.fHat)
        else:
            print("I SET SELF.OBS 2")
            self.obs2 = self.obs

    def Sample(self):
        # add self.m number of valid samples
        #self.dbgSampleCount +=1

        # something to break out of the directly connected case
        i = 0 
        ##print("DEBUG OUTPUT FOR def Sample(self):")
        ##print("self.m", self.m)
        ##print("self.c", self.c)
        while (i < self.m):
            xRand = random.uniform(self.xMin, self.xMax)
            yRand = random.uniform(self.yMin, self.yMax)
            xIdx = int(np.floor(xRand))
            yIdx = int(np.floor(self.yMax - yRand))

            # i'm pretty sure its supposed to sample from around where you are in the search 
            tmpG = self.calculate_L2(xRand, yRand, self.start.x, self.start.y)
            tmpH = self.calculate_L2(xRand, yRand, self.goal.x, self.goal.y)

            ##print("xrand: " + str(xRand) + " yrand: " + str(yRand))
            ##print("tmpG " + str(tmpG) + " tmpH " + str(tmpH) )
            ##print("tmpH + tmpH = ", tmpG+ tmpH)
            ##print("goal.gT",self.goal.gT)
            if (tmpG+tmpH) < self.goal.gT: 
                ##print("adding samples into Xsamples")
                # this sometimes causes an index error
                if self.obs[yIdx][xIdx] == 0:
                    stateToAdd = State()
                    stateToAdd.x = xRand
                    stateToAdd.y = yRand
                    stateToAdd.gHat = tmpG
                    stateToAdd.hHat = tmpH
                    stateToAdd.fHat = tmpG + tmpH
                    self.Xsamples[stateToAdd] = stateToAdd
                    i+=1
            ##print("length of Xsamples in Samples", len(self.Xsamples))
        return self.Xsamples

    def makeExplicitRGG(self):
        #self.V = self.Xsamples
        self.Xsamples[self.start] = self.start
        self.Xsamples[self.goal] = self.goal
        self.Xsamples2 = self.Xsamples.copy()
        for i in self.Xsamples:
            for j in self.Xsamples2:
                dist = self.calcDist(i,j)
                print(dist)
                if dist <= self.r:
                    edgeToAdd = Edge()
                    edgeToAdd.source_state = i
                    edgeToAdd.target_state = j
                    self.E[edgeToAdd] = edgeToAdd
        self.V = self.Xsamples

    def Initialize(self):
        self.changedMap = [[ [False, False] for x in range(self.xMax)] for y in range(self.yMax)]
        self.U = []         #{01}, {04 i do somewhere else  }
        startState = self.V[self.start]
        startKey = self.CalculateKey(startState)
        self.stateId += 1 
        heapq.heappush(self.U, (startKey,self.stateId, startState)) # {05}

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

    def Main(self):
        Xsamples = self.Sample()
        print("Xsamples", Xsamples)
        self.makeExplicitRGG()
        
        self.Initialize()

        
        
if __name__ == "__main__":
    random.seed(420)
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_clear.txt", "../test_environments/grid_envs_changing/environment50_A_77.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_69.txt", "../test_environments/grid_envs_changing/environment50_A_77.txt"]
    fileList = ["../test_environments/grid_envs_changing10/environment10_A_69.txt", "../test_environments/grid_envs_changing/environment50_A_77.txt"]
    print("LAST CHANCE")
    L = LAST()

    L.readEnvironment(fileList[0], False)
    L.Main()

    gv = Visualizer.Visualizer()
    gv.plotEnv(L)
    gv.plotEdges(L)
    gv.plotSamples(L) 
    gv.labelLastChance(L)
