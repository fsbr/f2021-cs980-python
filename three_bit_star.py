# trying to do LPA star on the samples
import sys,os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import random
import Visualizer
#from numba import jit

import time

import queue # for tree pruning
plt.style.use("seaborn-dark")

inf = np.Inf

# comment out to supress the terminal
#sys.stdout = open(os.devnull, 'w')

class State:
    def __init__(self):
        self.x = inf
        self.y = inf
        self.f = inf
        self.gT = inf
        self.hHat = inf
        self.gHat = inf
        self.fHat = inf 
        self.inCollision = False

class Edge:
    def __init__(self):
        self.source_state = State()
        self.target_state = State()
        self.f = inf
        cHat = 0.0
        self.inCollision = False 


class BIT_STAR:
    def __init__(self):
        # size of the world
        self.xMin = 0.0
        self.xMax = 10.0

        self.yMin = 0.0
        self.yMax = 10.0

        # adjacency grid
        self.obs = np.array([]) 

        self.tmpWhileBound = 10000 
        # Sample() params
        self.m = 100
        self.nNearest = 10   # must be smaller than m (actually turned out to not be true)
        # what would happen if nNearest was actually bigger than the batch is there any reason this isn't allowed

        # i would rather have a project than no project
        self.start = State()
        self.start.x = 1.1
        self.start.y = 3.0
        self.start.gT = 0.0

        self.goal = State()
        self.goal.x = 9.0
        self.goal.y = 8.0

        # the vertex and edges need to be dictionary
        self.V = {} 
        self.E = {}                                                     # A1.1

        self.oldV = []
        self.oldE = []

        self.Vold = {}
        self.Xsamples = {}
        self.Xnear = {}
    
        # vertex and edge queues, i guess can be heapq.
        self.Qe = []                                                     # A1.2
        self.Qv = []                                                     # A1.2
        self.r = inf                                                     # A1.2

        self.QeCount = 0
        self.QvCount = 0

        # solution cost
        self.c = inf
        #self.c = 9999

        # DEBUG PARAMS
        # temporarily run the while loop
        self.tmpWhile = 0
        self.dbgAttemptedEdgeList = []
        self.dbgSampleCount = 0
        self.dbgExpandVertexCount = 0
        self.dbgCollisionCheckCount = 0

        # anytime plot
        self.cVector = []
        self.tmpWhileVector = []
        self.timeVector = []

        #dynamic environment
        self.environmentUpdated = False
        self.SkipPruning = False
    
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

    def calculate_L2(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def calcDist(self, state1, state2):
        x1 = state1.x
        y1 = state1.y
        x2 = state2.x
        y2 = state2.y
        return self.calculate_L2(x1,y1,x2,y2)
        
    def collisionCheck(self, Vm, Xm):
        self.dbgCollisionCheckCount += 1
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
        numberOfChecks = 500
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

    def testCheckObs(self): 
        Vm = State()
        Vm.x = 1
        Vm.y = 4
        Xm = State()
        Xm.x = 3
        Xm.y = 3
        hit = self.collisionCheck(Vm,Xm)
        #print("hit in the check obs")
        #print(hit)
        return hit

    def Prune(self):
        ##print("A3.1")
        for state in list(self.Xsamples):                                               #A3.1
            ##print("IN PRUNING")
            #print(state) 
            #print("state.fHat", state.fHat)
            #print("self.c ", self.c)
            if state.fHat >= self.c:
                #print("from samples: pruning state of x=" + str(state.x) + " y= " + str(state.y))
                self.Xsamples.pop(state)  
                ##print("A3.1 pruned a sample")
                #print("The number of samples we have == ", len(self.Xsamples))

        ##print("A3.2")
        for state in list(self.V):                                                      #A3.2
            if state.fHat > self.c:
                #print("attempting to prune a useless state from self.V")
                #print("from self.V pruning state of x=" + str(state.x) + " y= " + str(state.y))
                self.V.pop(state)
                ##print("A3.2 pruned a state")

        ##print("A3.3")
        for edge in list(self.E):                                                       #A3.3
            ##print("edge.source_state.fHat", edge.source_state.fHat)  
            ##print("edge.target_state.fHat", edge.target_state.fHat)  
            ##print("self.c", self.c)  
            ##print("edge.target_state.x", edge.target_state.x)
            ##print("edge.target_state.y", edge.target_state.y)
            ##print("edge.source_state.fHat > self.c", edge.source_state.fHat > self.c)
            ##print("edge.target_state.fHat > self.c", edge.target_state.fHat > self.c)

            if ((edge.source_state.fHat > self.c) or (edge.target_state.fHat > self.c)):  # worked wiht fHat i think correct is "OR"
                ##print("attempting to prune an edge")
                ##print("with target coordinates")
                ##print("edge.target_state.x", edge.target_state.x)
                ##print("edge.target_state.y", edge.target_state.y)

                # before we pop this edge, we need to make sure that we set its gT == inf in the vertex
                # IF it is no longer conected in the tree
                foundInTree = False

                # pop the edge, then check that we need to update the corresponding vertex

                #edge.source_state.gT = inf          # we are about to disconnect it, dont think source_state.gT = inf is right
                ##print("state not found as successor")
                #rootState = edge.source_state       # i think source state is correct
                rootState = edge.target_state
                trimQueue = queue.Queue()
                trimQueue.put(rootState)
                edgesToPop = [] 
                while not trimQueue.empty():
                    bfsV = trimQueue.get()
                    for edge in self.E:
                        if edge.source_state == bfsV:
                        #if edge.target_state == bfsV:
                            #edge.source_state.gT = inf
                            edge.target_state.gT = inf 
                            # do i pop the edge here?
                            edgesToPop.append(edge)
                            trimQueue.put(edge.target_state)

                # dont prune if the last hting there was the goal
                # this is definitely wrong
                for e in edgesToPop:
                    self.E.pop(e)


                #self.E.pop(edge)
                ##print("A3.3 pruned an edge")
                            
                # uncomment this block below if u break something
                                
            
        ##print("A3.4")
        #for state in list(self.V):
        #    if state.gT == inf:
        #        self.Xsamples[state] = state

        #for state in list(self.V):
        #    if state.gT == inf:
        #        self.V.pop(state)
        for state in list(self.V):                                                      #A3.4
            if state.gT == inf:
                self.Xsamples[state] = state
                self.V.pop(state)

                ##print("from V removed state")
                ##print("Xsamples added state")
                ##print(" state was x= " + str(state.x) + " y = " +str(state.y))

    def Radius(self,q):        
        # i have no idea how to do this one
        n = 2
        A = np.sqrt((1+1/n))
        B = np.sqrt((2/np.pi))
        C = np.sqrt((np.log(q)/q))
        return A*B*C

    def Sample(self):
        # add self.m number of valid samples
        self.dbgSampleCount +=1

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
       
    def bestQueueValue(self, queue):
        ##print("printing the queue" + " " + str(type(queue)))
        ##print(len(queue))
        if len(queue) == 0:
            return inf
        else:
            #print("what is the data structure coming out of bestQueueValue")
            #print(queue)
            ##print("smallest 1 values")
            ##print(heapq.nsmallest(1,queue))
            bestValue = heapq.nsmallest(1,queue)[0][0]
            ##print("BEST VALUE IS")
            ##print(bestValue)
            return bestValue 

    def NearestNeighborsX(self):
        # sorts the samples by whos nearest to me 
        XnearQueue = []
        for i in self.Xsamples:
            heapq.heappush(XnearQueue, (calculate_L2(i.x,i,y,v.y,v.y), i))

        for j in range(self.nNearest):
            a = heapq.heappop(XnearQueue)[1]
            self.Xnear[a] = a
        
    def ExpandVertex2(self):
        # nearest neighbors are guaranteed to get put onto the edge queue
        self.dbgEV2X = 0
        self.dbgEV2V = 0
        self.dbgVVold = False
        v0 = heapq.heappop(self.Qv)                                          # A2.1
        v = v0[2]
        
        # sorting the nearest samples
        self.Xnear = {} 
        sampleQueue = [] 
        ##print("len(sampleQueue)", len(sampleQueue))
        ##print("len(self.Xsamples)", len(self.Xsamples))
        for i in self.Xsamples:
            heapq.heappush(sampleQueue, (self.calculate_L2(v.x, v.y, i.x, i.y), i) )
        ##print("after placing stuff into my sampleQueue")
        ##print("lenght of sampleQueue", len(sampleQueue))
        #print(sampleQueue)
        
        neighborSampleCounter = 0
        if self.nNearest < len(sampleQueue):
            sampleBoundary = self.nNearest
        else:
            sampleBoundary = len(sampleQueue)

        # there isn't always something in the sample queue
        ##print("neighborSampleCounter: ", neighborSampleCounter)
        ##print("sampleBoundary", sampleBoundary)
        while neighborSampleCounter < sampleBoundary:
            ##print("len sampleQueue", len(sampleQueue))
            protoSample = heapq.heappop(sampleQueue)
            ##print("protoSample", protoSample)
            ##print("len(protoSample)", len(protoSample))
            ##print("protoSample", protoSample[0])
            ##print("protoSample", protoSample[1])
            sampleToAdd = protoSample[1]
            #sampleToAdd = heapq.heappop(sampleQueue)[1]
            gHatV = self.calcDist(v, self.start)
            cHat = self.calcDist(v, sampleToAdd)
            hHatX = self.calcDist(sampleToAdd, self.goal)

            edgeToAdd = Edge()
            edgeToAdd.source_state = v
            edgeToAdd.target_state = sampleToAdd 
            edgeToAdd.cHat = cHat
            edgeToAdd.f = gHatV + cHat + hHatX

            self.QeCount+=1
            heapq.heappush(self.Qe, (edgeToAdd.f, self.QeCount, edgeToAdd)) # A2.3

            neighborSampleCounter+=1
            self.dbgEV2X +=1

        ##print("IS V IN VOLD", v in self.Vold)
        ##print("len self.V", len(self.V))
        vNotIn = v not in self.Vold
        noSamplesLeft = len(self.Xsamples) == 0
        #if v not in self.Vold:
        if vNotIn: #or noSamplesLeft:
            self.dbgVVold = True
            nearestQueue = []
            self.Vnear = {} 
            for i in self.V:
                heapq.heappush(nearestQueue, (self.calculate_L2(v.x, v.y, i.x,i.y), i))

            ##print("self.Vnear in vold part", self.Vnear)
            ##print("start in self.Vnear = ", self.start in self.Vnear)
            ##print("v in self.Vnear = ", v in self.Vnear)
            #print("nearestQueue", nearestQueue)

            # pop off the first element, which is always itself
            heapq.heappop(nearestQueue)
            #print("nearestQueue", nearestQueue)
            
            nearestTreeCounter = 0
            if len(nearestQueue) < self.nNearest:
                boundary = len(nearestQueue)
            else:
                boundary = self.nNearest
            ##print("boundary is", boundary)
            ##print("len(nearestQueue)", len(nearestQueue))
            ##print("self.nNearest", self.nNearest)
            ##print("nearestTreeCounter before loop = " , nearestTreeCounter)

            while nearestTreeCounter < boundary:
                sampleToAdd = heapq.heappop(nearestQueue)[1]
                ##print("popped the following sample", sampleToAdd)
                gHatV = self.calcDist(v, self.start)
                cHat = self.calcDist(v, sampleToAdd)
                hHatX = self.calcDist(sampleToAdd, self.goal)

                edgeToAdd = Edge()
                edgeToAdd.source_state = v
                edgeToAdd.target_state = sampleToAdd 
                edgeToAdd.cHat = cHat
                edgeToAdd.f = gHatV + cHat + hHatX
                self.QeCount+=1
                ##print("EDGE TO ADD IN SELF.E ?", edgeToAdd in self.E)
                if edgeToAdd not in self.E:
                    heapq.heappush(self.Qe, (edgeToAdd.f, self.QeCount, edgeToAdd)) # A2.3
                    ##print("PLACED AN EDGE BASED ON VOLD PART")
                    ##print("EDGE ADDED WAS( " + str(edgeToAdd.f) + " self.QeCount " + str(self.QeCount) + " edgeToAdd " + str(edgeToAdd))
                    self.dbgEV2X +=1 
                else:
                    pass
                    ##print("SKIPPED THE DGE")
                nearestTreeCounter+=1
                ##print("in loop nearestTreeCounter", nearestTreeCounter)

    def BIT_STAR_MAIN(self, startTime, stopTime, mode = "replan") :
        self.V[self.start] = self.start                                     # A1.1
        self.Xsamples[self.goal] = self.goal                                # A1.1
                                                                            # A1.2 is in the __init__ part 
        #while self.tmpWhile <self.tmpWhileBound:                                             # A1.3 this version has done the most testing
        while time.time() < stopTime:
        #while True:
            # i think each iteration of this we dump the motion tree
            #print("LINE A1.4 CHECK")
            #print("Qe Size" + str(len(self.Qe)) + "Qv Size" + str(len(self.Qv)))

            # THIS IS A HACK NOT PART OF THE ALGORITHM
            #if (len(self.Xsamples) == 0):
            #    self.Sample()
            if (len(self.Qe) == 0 and len(self.Qv) == 0):                   # A1.4
                #print("LINE A1.5")
                ##print("prune")                                              # A1.5 
                
                self.Prune()

                #print("LINE A1.6")
                self.Xsamples = self.Sample()                               # A1.6
                ##print("A1.6, length of Xsamples", len(self.Xsamples))

                #print("LINE A1.7")
                self.Vold = self.V.copy()                                          # A1.7
                ##print("self.Vold == self.V", self.Vold == self.V)

                ##print("before A1.8 length of self.V", len(self.V))
                ##print("before A1.8 length of self.Qv", len(self.V))
                for vertex in self.V:
                    self.QvCount+=1
                    ##print("pushing every state in v to the vertex queue")
                    #heapq.heappush(self.Qv, (vertex.gT, self.QvCount, vertex))        # A1.8 was working but i broke it
                    ##print("sorting value", vertex.gT + vertex.hHat)
                    sortValue = vertex.gT + vertex.hHat
                    heapq.heappush(self.Qv, (sortValue, self.QvCount, vertex))        # A1.8
                ##print("after A1.8 length of self.V", len(self.V))
                ##print("after A1.8 length of self.Qv", len(self.Qv))

                # RADIUS STUFF
                #self.r = len(self.V) + len(self.Xsamples)                   # A1.9
                #self.r = 10*self.Radius(len(self.V) + len(self.Xsamples))
                #self.r = 5.1 
                #print(" printing that weird radius thing", self.r)
        

            # you actually dont want to pop from the vertex queue
            ##print("checking Qe")
            #print("self.Qe", self.Qe)
            ##print("self.Qe lenght", len(self.Qe))
            ##print("qv bqv", self.bestQueueValue(self.Qv))
            ##print("apparent number of elements in Qe",len(self.Qe))
            ##print("qe bqv", self.bestQueueValue(self.Qe))

            while self.bestQueueValue(self.Qv) <= self.bestQueueValue(self.Qe): # A1.10
                #print("getting to expand next vertex")

                self.ExpandVertex2()                                         # A1.11

            #if len(self.Qe) > 0:
            
            ##print("POST VERTEX EXPANSION")
            #print("SELF> QE", self.Qe)
            ##print("QE length", len(self.Qe))

            currentEdge0 = heapq.heappop(self.Qe)                           # A1.12, A1.13
            currentEdge = currentEdge0[2]

            #self.dbgAttemptedEdgeList.append(currentEdge)
            ##print("PRINTING CURRENT EDGE")
            ##print(currentEdge)
            self.tmpWhile += 1

            Vm = currentEdge.source_state
            Xm = currentEdge.target_state


            ##print("Vm.gT", Vm.gT) 
            ##print("currentEdge.source_statex x = " + str(currentEdge.source_state.x) + " " + str(currentEdge.source_state.y))
            ##print("currentEdge.target_statex x = " + str(currentEdge.target_state.x) + " " + str(currentEdge.target_state.y))
            ##print("currentEdge.cHat", currentEdge.cHat) 

            # IMPORTANT TO CALCULATE hHAT 
            Vm.gHat = self.calcDist(Vm, self.start)
            Xm.hHat = self.calcDist(Xm, self.goal)

            ##print("trying to add hHat into the vertex set")
            ##print("PRE CHECK OF LINE 14")
            ##print("Vm.gT", Vm.gT)
            ##print("currentEdge.cHat", currentEdge.cHat)
            ##print("Xm.hHat", Xm.hHat)
            ##print("self.goal.gT = ", self.goal.gT)

            if Vm.gT + currentEdge.cHat + Xm.hHat < self.goal.gT:                     # A1.14 
                ##print("passed check of a1.14")
                ##print("Vm.x " +str(Vm.x) + " Vm.y " + str(Vm.y))
                ##print("Xm.x " +str(Xm.x) + " Xm.y " + str(Xm.y))
                gHatVm = self.calcDist(Vm, self.start)  
                
                # look into the occupance grid, and if the line formed by Vm->Xm
                # is intersecting a 1, then set the cost == inf, else the cost = L2 norm
                collisionHappened = self.collisionCheck(Vm, Xm)
                ##print("collisionHappened", collisionHappened)

                ## my motion tree gets fully pruned if i do colliison checking
                if collisionHappened == True:
                    ##print(" COLLISION IN OBSTALCE SET")
                    realCost = inf 
                else:
                    ##print("NO COLLISION")
                    currentEdge.cHat = self.calcDist(currentEdge.source_state, currentEdge.target_state)
                    realCost = currentEdge.cHat                             
                    self.dbgAttemptedEdgeList.append(currentEdge)

                #realCost = currentEdge.cHat                             
                ##print("Vm.gT", Vm.gT)
                ##print("Xm.gT", Xm.gT)
                ##print("realCost", realCost)
                ##print("Vm.gHat", Vm.gHat)
                ##print("Xm.hHat", Xm.hHat)
                ##print("self.goal.gT", self.goal.gT)

                if Vm.gHat + realCost +Xm.hHat < self.goal.gT:                  # A1.15
                    ##print("passed check of #A1.15")
                    ##print("Vm.gT", Vm.gT)
                    ##print("realCost", realCost)
                    ##print("Xm.gT", Xm.gT)

                    if Vm.gT + realCost < Xm.gT:                                # A1.16
                        ##print("passed check of #A1.16")
                        ##print("type of Xm", type(Xm))
                        if Xm in self.V:                                        # A1.17 
                            ##print("state was in" )
                            edgeToPop = Edge()
                            for edge in list(self.E):                           # added list and removed break
                                if edge.target_state == Xm:
                                    ##print(type(edge))
                                    ##print("PRUNING EDGES INSIDE ELLIPSE")
                                    self.E.pop(edge)                             # A1.18 
                                    #break
                        else:                                                   # A1.19

                            ##print("doing the non member stuff")
                            ##print("Lenght of self.Xsamples before", len(self.Xsamples))
                            self.Xsamples.pop(Xm)                               #A1.20
                            ##print("Length of self.Xsamples after", len(self.Xsamples))

                            # a hack not part of the algorithm
                            #if len(self.Xsamples) == 0:
                            #    self.Sample()
                            ##print("Length of Vertex Set A1.21 before", len(self.V))
                            self.V[Xm] = Xm                                     #A1.21
                            ##print("Length of Vertex Set A1.21 after", len(self.V))
                            ##print("is Xm in V?", Xm in self.V)
                            ##print("is Xm in Vold?", Xm in self.Vold)

                            Xm.gT = Vm.gT + realCost 
                            #Xm.gT = Vm.gT + currentEdge.cHat
                            self.QvCount+=1
                            sortvalue = Xm.gT +Xm.hHat
                            heapq.heappush(self.Qv, (sortvalue, self.QvCount, Xm))   #A1.21 #self.E[currentEdge] = currentEdge 
                        ##print("EDGE ADDED TO MOTION TREE")
                        ##print("CHECK EDGE CONTAINS GOAL STATE")
                        if currentEdge.target_state == self.goal:
                            ##print("ADDED EDGE CONTAINS GOAL STATE")
                            edgeOfInterest = currentEdge
                            tmpCost =  0

                            ##print("ENTERING COST TRAVERSAL")
                            while edgeOfInterest.source_state != self.start:
                                tmpCost += edgeOfInterest.cHat
                                print("in COST TRAVERSAL tmp Cost", tmpCost)
                                #if tmpCost > self.c:
                                #    break
                                # i dont want to iterate thru the whole tree just to find the edge target_state = edgeOfInterest.source_state  
                                # but its whats going to finish my project before december 2
                                for edge in list(self.E.values()):
                                    if edge.target_state == edgeOfInterest.source_state:
                                        edgeOfInterest = edge
                                        break
                            tmpCost +=edgeOfInterest.cHat

                            ##print("FINAL tmpCost", tmpCost)
                            if tmpCost < self.c:
                                self.oldE.append(self.E)
                                self.oldV.append(self.V)
                                # have to also attach the new cost to the goal state i think?
                                self.c = tmpCost
                                self.goal.gT = self.c
                                self.cVector.append(self.c)
                                #self.tmpWhileVector.append(self.tmpWhile)
                                self.timeVector.append(time.time() - startTime)
                        #print(self.E)                                           
                        self.E[currentEdge] = currentEdge                       #A1.22
                        #if (Vm.gT + currentEdge.cHat >= Xm.gT):                 #A1.23
                            #heapq.heappop(self.Qe)                              #A1.23
                        for element in self.Qe:
                            edge = element[2]
                            if edge.target_state == Xm: 
                                if edge.cHat + edge.source_state.gT >= Xm.gT:
                                    heapq.heappop(self.Qe)
                                    #self.Qe.remove(element)

                        # stop if the goal and start can be directly connected
                        # this is a hack and not in the real algorithm
                        #if self.goal in self.V and self.start in self.V:
                        #    break
                        # terminate if within 1% of the optimal solution
                        if self.c < 1.01*(self.calcDist(self.start, self.goal)):
                            break
            else:                                                               #A1.24
                ##print("Failed check of #A1.14")
                self.Qe = []                                                    #A1.25
                self.Qv = []                                                    #A1.25
            ##print("SELF.tmpWhile", self.tmpWhile)
            #print("time.time() - t_end", t_end - time.time())
            if t_end - time.time() <=   test_length/2 and self.environmentUpdated == False:
                # half way thru we will read in the B version of the environment
                self.readEnvironment(fileList[1], updated = True)
                self.environmentUpdated = True
                costFile.write("new environment detected at %s; mode = %s\n"%((t_end - time.time(), mode)))
                # clear the motion tree
                if mode == "replan":
                    self.V = {}
                    self.E = {}
                    self.Xsamples = {} 

                    # reset goal parameters
                    self.goal.gT = inf
                    self.c = inf

                    #ggv = Visualizer.Visualizer()
                    #ggv.plotMotionTree(self.V, self.E, self.obs, self.xMax, self.yMax, self.Xsamples, self.start, self.goal, self.oldE)
                    # reset it to the initial conditions
                    self.V[self.start] = self.start  
                    self.Xsamples[self.goal] = self.goal
                    self.Qe = []
                    self.Qv = []
                    # i think i have to move the goal.gT as well...
                if mode == "prune":

                    # we want to prune the motion tree and replan from there
                    # specifically, we want to prune subtrees in order from largest to smallest.
                    # collisionRoot is where we will start collision checking again
                    prune_t_start = time.time()
                    print("STARTING TO PRUNE MOTION TREE", prune_t_start)
                    collisionRoot = self.start
                    collisionQueue = queue.Queue()
                    collisionQueue.put(collisionRoot)
                    self.collisionEdgesToPop = []
                    self.rootsToPrune = [] 
                    subtreeCount = 0

                    # i can do this in ONE BFS 
                    while not collisionQueue.empty():
                        stateToCheck = collisionQueue.get()
                        for edge in list(self.E):
                            if edge.source_state == stateToCheck:
                                isCollision = self.collisionCheck(edge.source_state, edge.target_state)

                                # end of collsion checking queue
                                collisionQueue.put(edge.target_state)
                                if isCollision == True:
                                    self.collisionEdgesToPop.append(edge)
                                    #print("this subtree is in collision")
                                    subtreeCount+=1 
                                    self.rootsToPrune.append(edge)

                    #print("length of the list of edges to pop", len(self.collisionEdgesToPop))
                    #print("lenght of the list of edges at this exact moment", len(self.E))
                    #print("the number of subtrees in collision", subtreeCount)
                    #print("print self.rootsToPrune", self.rootsToPrune)

                    # this takes like 3 seconds!!, which is way too long
                    # this method is so fucking dumb i need to do one BFS
                    pruneTheseEdges = []
                    for e in self.collisionEdgesToPop:
                        #subTreeForE = []         
                        bfsQ = queue.Queue()
                        poppedEdgeRootState = e.target_state
                        #poppedEdgeRootState = e.source_state
                        bfsQ.put(poppedEdgeRootState)
                        while not bfsQ.empty():
                            bfsV = bfsQ.get()
                            for edge in self.E:
                                if edge.source_state == bfsV:
                                    edge.target_state.gT = inf
                                    if edge.target_state == self.goal:
                                        self.c = inf
                                    #subTreeForE.append(edge)
                                    pruneTheseEdges.append(edge)
                                    bfsQ.put(edge.target_state)
                                    #bfsQ.put(edge.source_state)
                        #for prunedEdge in subTreeForE:
                        #    if prunedEdge in self.E:
                        #        self.E.pop(prunedEdge)
                        #        if prunedEdge.source_state in self.V:
                        #            self.V.pop(prunedEdge.source_state) 
                        #        if prunedEdge.target_state in self.V:
                        #            self.V.pop(prunedEdge.target_state)
                    #for edge in self.collisionEdgesToPop:
                    #    pruneTheseEdges.append(edge)
                    
                    #for e in pruneTheseEdges:

                    
                    for e in self.collisionEdgesToPop + pruneTheseEdges:
                        if e in self.E:
                            self.E.pop(e)
                        if e.source_state in self.V:
                            self.V.pop(e.source_state)
                        if e.target_state in self.V:
                            self.V.pop(e.target_state)

                        for keyStatePair in self.Qe:
                            if keyStatePair[2] == e:
                                self.Qe.remove(keyStatePair)

                        for keyStatePair in self.Qv:
                            if keyStatePair[2] == e.source_state:
                                self.Qv.remove(keyStatePair)
                            if keyStatePair[2] == e.target_state:
                                self.Qv.remove(keyStatePair)

                    #print("len self.V", len(self.V))
                    #print("len self.E", len(self.E))
                    # i need to plot the motion tree right now. 
                    #ggv = Visualizer.Visualizer()
                    #ggv.plotMotionTree(self.V, self.E, self.obs, self.xMax, self.yMax, self.Xsamples, self.start, self.goal, self.oldE)

                    # NO WE DONT JUST ERASE THIS THING
                    #self.Qe = []
                    #self.Qv = []
                    #self.Xsamples = {}
                    #self.Xsamples = self.Sample()
                    self.V[self.start] = self.start  
                    self.Xsamples[self.goal] = self.goal
                    #print("i need to make this fast but the idea is decent")    
                    prune_t_end = time.time()
                    print("ending pruning process at ", prune_t_end - prune_t_start)
                    costFile.write("pruning process end, %s\n"%(prune_t_end - prune_t_start))
            
            #print("broke out") 
        return self.V, self.E 

if __name__ == "__main__":
    #vertices = open("vertices.txt","w")
    costFile = open("costs%s.csv"%time.time(), "w")

    # for debugging, but i'm pretty sure randomness can cause issues
    #random.seed(6)
    # input stuff
    #
    BS = BIT_STAR()

    # for changing environments
    #fileList = ["test_environments/grid_envs_changing10/environment10_B_5.txt", "test_environments/grid_envs_changing10/environment10_A_5.txt"]
    #fileList = ["test_environments/grid_envs_changing10/environment10_B_5.txt", "test_environments/grid_envs_changing10/environment10_A_5.txt"]

    # instance 7 is interesting
    #fileList = ["test_environments/grid_envs_changing/environment50_B_7.txt", "test_environments/grid_envs_changing/environment50_A_7.txt"]
    #fileList = ["test_environments/grid_envs_changing/environment50_A_7.txt", "test_environments/grid_envs_changing/environment50_B_7.txt"]
    #fileList = ["test_environments/grid_envs_changing/environment50_B_19.txt", "test_environments/grid_envs_changing/environment50_A_19.txt"]
    #fileList = ["test_environments/grid_envs_changing/environment50_B_99.txt", "test_environments/grid_envs_changing/environment50_A_99.txt"]

    # 
    #fileList = ["test_environments/grid_envs_changing/environment50_B_77.txt", "test_environments/grid_envs_changing/environment50_A_77.txt"]
    #fileList = ["test_environments/grid_envs_changing/environment50_A_77.txt", "test_environments/grid_envs_changing/environment50_B_77.txt"]
    #fileList = ["test_environments/grid_envs_changing/environment50_B_54.txt", "test_environments/grid_envs_changing/environment50_A_54.txt"]
    fileList = ["test_environments/grid_envs_changing/shortcutA.txt", "test_environments/grid_envs_changing/shortcutB.txt"]
    #fileList = ["test_environments/grid_envs_changing/shortcutB.txt", "test_environments/grid_envs_changing/shortcutA.txt"]
    BS.readEnvironment(fileList[0], False)
    #BS.readEnvironment("test_environments/grid_envs50/environment50_3.txt")
    #BS.readEnvironment("test_environments/grid_envs1000/environment1000_3.txt")
    #BS.readEnvironment("test_environments/grid_envs/environment69.txt")
    #BS.readEnvironment("test_environments/grid_envs/environment104.txt")
    # env 90 caused an out of bounds error on collision checking. might be hard to track down
    #BS.readEnvironment("test_environments/grid_envs/environment699.txt")
    #BS.readEnvironment("snake.txt")
    #hit = BS.testCheckObs()
    #print("pritning hit")
    #print(hit)
    #print("asdfadsf")

    #from timeit import Timer
    # most testing in 20 seconds
    test_length = 20 
    t_start = time.time()
    t_end = time.time() + test_length
    V,E = BS.BIT_STAR_MAIN(t_start, t_end, "prune")
    #t.timeit()


    ##output stuff
    #print("VERTICES")
    ##print(V)
    #print(len(V))

    #print("EDGES")
    #print(E)
    #print(len(E))
    print(BS.obs)
    #for i in V:
    #    print("x", i.x)
    #    print("y", i.y)

    #gv = Visualizer.Visualizer()
    #gv.plotMotionTree(V,E,BS.obs,BS.xMax, BS.yMax,BS.Xsamples,BS.start, BS.goal, BS.oldE)
    #gv.plotAttemptedEdge(BS.dbgAttemptedEdgeList, BS.obs, BS.yMax)

    #print(BS.tmpWhileVector)
    #print("time vector", BS.timeVector)
    #print("cost vector", BS.cVector)
    #plt.step(BS.timeVector, BS.cVector)
    #plt.xlabel("Time (s)")
    #plt.ylabel("Solution Cost")
    #plt.title("Solution Cost vs. Time")
    #plt.show()
    costFile.write(str(BS.m)+","+str(BS.nNearest)+"\n")
    costFile.write(str(test_length) + "\n")
    for item in range(len(BS.cVector)):
        costFile.write(str(BS.timeVector[item]) + "," + str(BS.cVector[item]))
        costFile.write("\n")
    costFile.close()
