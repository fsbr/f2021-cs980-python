# JUST implementing an edge queu to do LPA*
import sys
sys.path.append("../")
import code

import Visualizer
import time
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
        self.g = inf
        self.h = inf
        self.gT = inf

        self.rhs = inf
        

class Edge():
    def __init__(self):
        self.source_state = State() 
        self.target_state = State()
        self.edgeCost = inf

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
        self.m = 1000
        self.Xsamples = {}

        # rgg stuff
        self.r = 10.0 
        self.V = {}
        self.E = {}

        # state appending
        self.stateId = 0

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



    def collisionCheck(self, Vm, Xm):
        #self.dbgCollisionCheckCount += 1
        # returns true if there is a collision
        x1 = Vm.x
        y1 = Vm.y
        x2 = Xm.x
        y2 = Xm.y

        # get the equation of the line
        m = (Xm.y - Vm.y)/(Xm.x - Vm.x)
        b = y1 - m*x1
        ##print("slope m = ",m)
        ##print("intercept b = ", b)
        # 200 checks per unit in X
        numberOfChecks = 200
        checkY = int(abs(y2-y1)*numberOfChecks)
        checkX = int(abs(x2-x1)*numberOfChecks)
        if checkY > checkX:
            collCheck = checkY
        else:
            collCheck = checkX
        t = np.linspace(x1, x2,collCheck) #10,000 to avoid the diag
        ##print("len t", len(t))
        Y = m*t + b
        ##plt.plot(t,Y)
        ##plt.xlim((0,11))
        ##plt.ylim((0,11))
        #plt.show()
        coordinate_t = np.floor(t) 
        #print(Y)
        #print(self.xMax)
        #print(self.yMax-Y)
        coordinate_Y = np.floor(self.yMax-Y) 
        #print(coordinate_t)
        #print(coordinate_Y)
        #print("PRINTING OBSTACLES")
        #print(self.obs)
        #print("PRINTING OBSTACLES FINISHED")
        # i iterates over the 
        for i in range(len(coordinate_t)): 
            idxToObs_t = int(coordinate_t[i])
            idxToObs_Y = int(coordinate_Y[i]) 
            #print("idxToObs_t" + str(idxToObs_t) + " idxToObs_Y " + str(idxToObs_Y))
            #print("iterating over the obstacles")
            #print(self.obs[idxToObs_Y][idxToObs_t])
            if self.obs[idxToObs_Y][idxToObs_t] == 1:
                return 1  
        return 0


    def makeExplicitRGG(self):
        #self.V = self.Xsamples
        self.Xsamples[self.start] = self.start
        self.Xsamples[self.goal] = self.goal

        for sample in self.Xsamples:
            sample.h = self.calcDist(sample, self.goal)

        self.Xsamples2 = self.Xsamples.copy()
    
        # start and goal stuff
        self.start.g = 0
        self.start.h = self.calcDist(self.start, self.goal)

        self.goal.h = 0
        self.goal.g = self.calcDist(self.start, self.goal)

        for i in self.Xsamples:
            for j in self.Xsamples2:
                dist = self.calcDist(i,j)
                #print(dist)
                if dist <= self.r and i != j:
                    edgeToAdd = Edge()
                    edgeToAdd.source_state = i
                    edgeToAdd.target_state = j
                    edgeToAdd.edgeCost = self.calcDist(i,j)
                    self.E[edgeToAdd] = edgeToAdd
        self.V = self.Xsamples.copy()

    def updateEdgeCosts(self):
        # to be called AFTER you load in second map
        print("UPDATING EDGE COSTS")
        changedEdges = []
        for E in self.E:
            tmpEdgeCost = E.edgeCost
            if self.collisionCheck(E.source_state, E.target_state) == 1:
                E.edgeCost = inf
            else:
                E.edgeCost = self.calcDist(E.source_state, E.target_state)

            if E.edgeCost != tmpEdgeCost:
                changedEdges.append(E)
        return changedEdges
            

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

    def UpdateVertex(self, edge, secondCase = False):
        # my s is his u
        s = edge.target_state
        root = edge.source_state

        print("root.x, root.y", root.x, root.y)
        print("s.x, s.y", s.x, s.y)
    

        rhsList = []
        cameFromIdxList = []
        cameFromCoordList = []
        fList = []
        preds = self.getPred(s)

        for pred in preds:
            #print(pred)
            #if pred.target_state !=  self.start:

            if self.collisionCheck(pred.source_state,pred.target_state) == 1:
                pred.edgeCost = inf
            tentativeRhs = pred.edgeCost + pred.source_state.g
            rhsList.append( tentativeRhs )
            cameFromIdxList.append(pred.source_state)
                
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
            print(cameFromIdxList)

            #code.interact(local=locals())
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

        if s.g != s.rhs:
            self.stateId+=1
            heapq.heappush(self.U, (self.CalculateKey(s), self.stateId, s) )
            print("enqueued onto U")
        print("finished update vertex")

    def getPred(self,u):
        self.succs = []
        for key, edge in self.E.items():
            if edge.target_state == u:
                self.succs.append(edge)
                #print("succs.edgeCost", edge.edgeCost)
        #print("self.succs", self.succs)
        return self.succs

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
        while self.bestU() < self.CalculateKey(self.V[self.goal]) or (self.V[self.goal].rhs != self.V[self.goal].g):
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
                    self.UpdateVertex(edge, True)
                    #self.motionE[(edge.source_state, edge.target_state)] = edge

    def Main(self):
        Xsamples = self.Sample()
        print("Xsamples", Xsamples)
        self.makeExplicitRGG()
        
        self.Initialize()

        tcsp1_start = time.time()
        self.ComputeShortestPath()
        tcsp1_end = time.time()
        tcsp1 = tcsp1_end - tcsp1_start

        self.solution1 = []
        self.solution2 = []

        stateOfInterest = self.goal
        self.solution1.append(self.goal)
        timeOut = 0
        while stateOfInterest != self.start :
            if timeOut >500:
                break
            self.solution1.append(stateOfInterest.cameFromIdx)
            stateOfInterest = self.V[stateOfInterest.cameFromIdx]            
            print("i am stuck in the first solution path :[")
            timeOut+=1
            print("timeOut", timeOut)
        print("solution path 1", self.solution1)


        ## REACT TO UPDATES
        self.readEnvironment(fileList[1], True)
        changedEdges = self.updateEdgeCosts()

        print(changedEdges)
        for changedEdge in changedEdges:
            self.UpdateVertex(changedEdge)

        tcsp2_start = time.time()
        self.ComputeShortestPath()
        tcsp2_end = time.time()
        tcsp2 = tcsp2_end - tcsp2_start

        stateOfInterest = self.goal
        self.solution2.append(self.goal)
        timeOut = 0
        while stateOfInterest != self.start :
            if timeOut >500:
                break
            self.solution2.append(stateOfInterest.cameFromIdx)
            stateOfInterest = self.V[stateOfInterest.cameFromIdx]            
            print("i am stuck in the first solution path :[")
            timeOut+=1
            print("timeOut", timeOut)
        print("solution path 2", self.solution2)

        return self.solution1, self.solution2, changedEdges, tcsp1, tcsp2
        
        
if __name__ == "__main__":
    #random.seed(69) # my lucky number
    #random.seed(10)
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_clear.txt", "../test_environments/grid_envs_changing/environment50_A_77.txt"]
    #fileList = ["../test_environments/grid_envs_changing/environment50_A_69.txt", "../test_environments/grid_envs_changing/environment50_A_77.txt"]
    #fileList = ["../test_environments/grid_envs_changing10/environment10_A_10.txt", "../test_environments/grid_envs_changing10/environment10_B_10.txt"]
    #fileList = ["../test_environments/grid_envs_changing10/environment10_B_10.txt", "../test_environments/grid_envs_changing10/environment10_A_10.txt"]
    #fileList = ["../test_environments/grid_envs_changing10/environment10_A_11.txt", "../test_environments/grid_envs_changing10/environment10_B_11.txt"]
    #fileList = ["../test_environments/grid_envs_changing10/environment10_B_11.txt", "../test_environments/grid_envs_changing10/environment10_A_11.txt"]
    fileList = ["../test_environments/grid_envs_changing10/environment10_A_69.txt", "../test_environments/grid_envs_changing10/environment10_B_69.txt"]
    #fileList = ["../test_environments/grid_envs_changing10/environment10_B_69.txt", "../test_environments/grid_envs_changing10/environment10_A_69.txt"]
    #fileList = ["../test_environments/grid_envs_changing10/environment10_A_42.txt", "../test_environments/grid_envs_changing10/environment10_B_42.txt"]
    fileList = ["../test_environments/grid_envs_changing/environment50_A_3.txt", "../test_environments/grid_envs_changing/environment50_A_3.txt"]
    print("LAST CHANCE")
    L = LAST()

    L.readEnvironment(fileList[0], False)
    #solution1, solution2, changedEdges, tcsp1, tcsp2 = L.Main()
    solution1X = []
    solution1Y = []
    #for state in solution1:
    #    #print("statex, statey", state.x, state.y)
    #    solution1X.append(state.x)
    #    solution1Y.append(state.y)
    #
    #solution2X = []
    #solution2Y = []
    #for state in solution2:
    #    #print("statex, statey", state.x, state.y)
    #    solution2X.append(state.x)
    #    solution2Y.append(state.y)


    #print("time for each results")
    #print("tcsp1 = %s\n"%tcsp1)
    #print("tcsp2 = %s\n"%tcsp2)
    gv = Visualizer.Visualizer()
    #gv.plotEdges(L)
    gv.plotEnv(L)
    ##gv.plotSolutionEdges(solution1)
    #gv.lcPlotSolution(solution1X, solution1Y, "#420690", 5)
    #gv.lcPlotSolution(solution2X, solution2Y, "#007fff", 3)
    #gv.plotSamples(L) 
    #gv.labelLastChance(L)

    # generating figure for the paper, comment out later
    gv.labelLastChance2(L)

