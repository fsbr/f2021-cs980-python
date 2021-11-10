# giving in

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import random

inf = np.Inf

class State:
    def __init__(self):
        self.x = inf
        self.y = inf
        self.f = inf
        self.gT = inf
        self.hHat = inf
        self.gHat = inf
        self.fHat = inf
        

class Edge:
    def __init__(self):
        self.source_state = State()
        self.target_state = State()
        self.f = inf
        cHat = 0.0


class BIT_STAR:
    def __init__(self):
        # size of the world
        self.xMin = 0.0
        self.xMax = 10.0

        self.yMin = 0.0
        self.yMax = 10.0

        # adjacency grid
        self.obs = np.array([]) 

        # Sample() params
        self.m = 9 

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

        # DEBUG PARAMS
        # temporarily run the while loop
        self.tmpWhile = 0
        self.dbgAttemptedEdgeList = []
        self.dbgSampleCount = 0
        self.dbgExpandVertexCount = 0
    
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
        self.start.gHat = 0
        self.start.hHat = self.calcDist(self.start,self.goal)
        self.start.fHat = self.start.gHat + self.start.hHat

        self.goal.gHat = self.calcDist(self.start, self.goal)
        self.goal.hHat = 0
        self.goal.fHat = self.goal.gHat + self.goal.hHat
        print("ENVIRONMENT READING fHat goal", self.start.fHat)
        print("ENVIRONMENT READING fHat goal", self.goal.fHat)
    def calculate_L2(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def calcDist(self, state1, state2):
        x1 = state1.x
        y1 = state1.y
        x2 = state2.x
        y2 = state2.y
        return self.calculate_L2(x1,y1,x2,y2)
        
    def collisionCheck(self, Vm, Xm):
        # returns true if there is a collision
        x1 = Vm.x
        y1 = Vm.y
        x2 = Xm.x
        y2 = Xm.y

        # get the equation of the line
        m = (Xm.y - Vm.y)/(Xm.x - Vm.x)
        b = y1 - m*x1
        print("slope m = ",m)
        print("intercept b = ", b)
        t = np.linspace(x1, x2,50)
        print("len t", len(t))
        Y = m*t + b
        plt.plot(t,Y)
        plt.xlim((0,11))
        plt.ylim((0,11))
        #plt.show()
        coordinate_t = np.floor(t) 
        #print(Y)
        #print(self.xMax)
        #print(self.yMax-Y)
        coordinate_Y = np.floor(self.yMax-Y) 
        print(coordinate_t)
        print(coordinate_Y)
        #print("PRINTING OBSTACLES")
        print(self.obs)
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
        print("hit in the check obs")
        print(hit)
        return hit

    def Prune(self):
        print("A3.1")
        for state in list(self.Xsamples):                                               #A3.1
            print("IN PRUNING")
            print(state) 
            if state.fHat > self.c:
                print("from samples: pruning state of x=" + str(state.x) + " y= " + str(state.y))
                self.Xsamples.pop(state)  

        print("A3.2")
        for state in list(self.V):                                                      #A3.2
            if state.fHat > self.c:
                print("attempting to prune a useless state from self.V")
                print("from self.V pruning state of x=" + str(state.x) + " y= " + str(state.y))
                self.V.pop(state)

        print("A3.3")
        for edge in list(self.E):                                                       #A3.3
            if ((edge.source_state.fHat > self.c) or (edge.target_state.fHat > self.c)):
                print("attempting to prune an edge")
                self.E.pop(edge)

        print("A3.4")
        for state in list(self.V):                                                      #A3.4
            if state.gT == inf:
                self.Xsamples[state] = state
                self.V.pop(state)


                 
    def Sample(self):
        # add self.m number of valid samples
        self.dbgSampleCount +=1
        i = 0 
        print("DEBUG OUTPUT FOR def Sample(self):")
        print("self.m", self.m)
        print("self.c", self.c)
        while (i < self.m):
            xRand = random.uniform(self.xMin, self.xMax)
            yRand = random.uniform(self.yMin, self.yMax)
            xIdx = int(np.floor(xRand))
            yIdx = int(np.floor(self.yMax - yRand))
            # i'm pretty sure its supposed to sample from around where you are in the search 
            tmpG = self.calculate_L2(xRand, yRand, self.start.x, self.start.y)
            tmpH = self.calculate_L2(xRand, yRand, self.goal.x, self.goal.y)

            print("xrand: " + str(xRand) + " yrand: " + str(yRand))
            print("tmpG " + str(tmpG) + " tmpH " + str(tmpH) )
            print("tmpH + tmpH = ", tmpG+ tmpH)
            print("goal.gT",self.goal.gT)
            if (tmpG+tmpH) < self.goal.gT: 
                print("adding samples into Xsamples")
                if self.obs[yIdx][xIdx] == 0:
                    stateToAdd = State()
                    stateToAdd.x = xRand
                    stateToAdd.y = yRand
                    stateToAdd.gHat = tmpG
                    stateToAdd.hHat = tmpH
                    stateToAdd.fHat = tmpG + tmpH
                    self.Xsamples[stateToAdd] = stateToAdd
                    i+=1
            print("length of Xsamples in Samples", len(self.Xsamples))
        return self.Xsamples
       
    def bestQueueValue(self, queue):
        print("printing the queue" + " " + str(type(queue)))
        print(len(queue))
        if len(queue) == 0:
            return inf
        else:
            print("what is the data structure coming out of bestQueueValue")
            print(queue)
            print("smallest 2 values")
            print(heapq.nsmallest(1,queue))
            bestValue = heapq.nsmallest(1,queue)[0][0]
            print("BEST VALUE IS")
            print(bestValue)
            return bestValue 

    def ExpandVertex(self):
        self.dbgExpandVertexCount+=1
        print("queue in expand vertex")
        #print(self.Qv)
        print("Edge Queue length", len(self.Qe)) 
        print("Vertex Queue length", len(self.Qe)) 
        # we're interested in the State that we are searching on, not the value its sorted by really
        v0 = heapq.heappop(self.Qv)                                          # A2.1
        v = v0[2]
        print("v in ExpandVertex",v)
        print("Does v Belong to self.Vold", v in self.Vold)

        # i think we clear it each time
        self.Xnear = {}
        print("self.Xsamples in ExpandVertex",self.Xsamples)
        for i in self.Xsamples:                                             # A2.2
            if self.calculate_L2(i.x, i.y, v.x, v.y) < self.r:  
                self.Xnear[i] = i 
        #print("self.Xnear", self.Xnear)
        print("len Xnear contains " + str(len(self.Xnear)) + " elements")
        for i in self.Xnear:
            gHatV = self.calcDist(v, self.start)
            cHat = self.calcDist(v,i)                                   # this number should be changing pers sample/
            hHatX = self.calcDist(i, self.goal)
            fHat = gHatV + cHat +hHatX
            #print("goal Gt", self.goal.gT)
            print("gHatV", gHatV)
            print("cHat", cHat)
            print("hHatX", hHatX)
            print("fHat", fHat)
            if gHatV + cHat + hHatX < self.goal.gT:
                edgeToAdd = Edge()
                edgeToAdd.source_state = v
                #i.gT = gHatV + cHat
                edgeToAdd.target_state = i
                edgeToAdd.cHat = cHat
                edgeToAdd.f = gHatV + cHat + hHatX
                print("edgeToAdd.f", edgeToAdd.f)
                #print("pushing edge X case")
                #print("edgeToAdd.f", edgeToAdd.f)
                print("adding edge of V.x = " + str(v.x) + " V.y " + str(v.y) + " i.x "  + str(i.x) + " i.y " + str(i.y))
                #print("Qe length", len(self.Qe))
                #print("Qe", self.Qe)

                heapq.heappush(self.Qe, (edgeToAdd.f, self.QeCount, edgeToAdd)) # A2.3
                self.QeCount+=1

        if v not in self.Vold:                                              # A2.4
            print("in self.Vold section")
            self.Vnear = {} 
            for i in self.Vold:                                             # A2.2
                if self.calculate_L2(i.x, i.y, v.x, v.y) < self.r:  
                    self.Vnear[i] = i 
            for i in self.V:
                gHatV = self.calcDist(v, self.start)
                cHat = self.calcDist(v,i)
                hHatX = self.calcDist(i, self.goal)
                print("gHatV", gHatV)
                print("cHat", cHat)
                print("hHatX", hHatX)
                if gHatV + cHat + hHatX < self.goal.gT:
                    edgeToAdd = Edge()
                    edgeToAdd.source_state = v
                    edgeToAdd.target_state = i
                    edgeToAdd.cHat = cHat
                    edgeToAdd.f = gHatV + cHat + hHatX
                    self.QvCount+=1
                    heapq.heappush(self.Qe, (edgeToAdd.f,self.QvCount, edgeToAdd))
        print("self.c cost ", self.c)
        print("EXPAND NEXT VERTEX FINISHED") 
    def BIT_STAR_MAIN(self):
        self.V[self.start] = self.start                                     # A1.1
        self.Xsamples[self.goal] = self.goal                                # A1.1
                                                                            # A1.2 is in the __init__ part 
        while self.tmpWhile <500:                                             # A1.3
        #while True:
            # i think each iteration of this we dump the motion tree
            print("LINE A1.4 CHECK")
            print("Qe Size" + str(len(self.Qe)) + "Qv Size" + str(len(self.Qv)))
            if (len(self.Qe) == 0 and len(self.Qv) == 0):                   # A1.4
                print("LINE A1.5")
                print("prune")                                              # A1.5 
                self.Prune()
                Xsamples = self.Sample()                                    # A1.6
                #print("length of Xsamples", len(Xsamples))
                self.Vold = self.V                                          # A1.7
                print("self.Vold == self.V", self.Vold == self.V)
                self.QvCount+=1
                #print("self.Qv", self.Qv)
                #print("self.start.gT", self.start.gT)
                #print("self.QvCount", self.QvCount)
                #print("self.start", self.start)
                # the line below at least did something
                #heapq.heappush(self.Qv, (self.start.gT, self.QvCount, self.start))        # A1.8
                print("before A1.8 length of self.V", len(self.V))
                print("before A1.8 length of self.Qv", len(self.V))
                for vertex in self.V:
                    print("pushing every state in v to the vertex queue")
                    heapq.heappush(self.Qv, (vertex.gT, self.QvCount, vertex))        # A1.8
                print("after A1.8 length of self.V", len(self.V))
                print("after A1.8 length of self.Qv", len(self.Qv))
                #print("printing self.Qv in bit star main ", self.Qv)
                #print(self.Qv)
                #self.r = len(self.V) + len(self.Xsamples)                   # A1.9
                #print(" printing that weird radius thing", self.r)
        

            # you actually dont want to pop from the vertex queue
            print("checking Qe")
            print("self.Qe", self.Qe)
            print("self.Qe lenght", len(self.Qe))
            print("qv bqv", self.bestQueueValue(self.Qv))
            print("apparent number of elements in Qe",len(self.Qe))
            print("qe bqv", self.bestQueueValue(self.Qe))
            #while (len(self.Qv) > 0) and self.bestQueueValue(self.Qv) <= self.bestQueueValue(self.Qe): # A1.10
            while self.bestQueueValue(self.Qv) <= self.bestQueueValue(self.Qe): # A1.10
                print("getting to expand next vertex")
                self.ExpandVertex()                                         # A1.11

            #if len(self.Qe) > 0:
            
            print("POST VERTEX EXPANSION")
            print("SELF> QE", self.Qe)
            print("QE length", len(self.Qe))

            currentEdge0 = heapq.heappop(self.Qe)                           # A1.12, A1.13
            currentEdge = currentEdge0[2]

            #self.dbgAttemptedEdgeList.append(currentEdge)
            print("PRINTING CURRENT EDGE")
            print(currentEdge)
            self.tmpWhile += 1

            Vm = currentEdge.source_state
            Xm = currentEdge.target_state


            print("Vm.gT", Vm.gT) 
            print("currentEdge.cHat", currentEdge.cHat) 

            # IMPORTANT TO CALCULATE hHAT 
            Vm.gHat = self.calcDist(Xm, self.goal)
            Xm.hHat = self.calcDist(Xm, self.goal)

            print("trying to add hHat into the vertex set")
            print("PRE CHECK OF LINE 14")
            print("Vm.gT", Vm.gT)
            print("currentEdge.cHat", currentEdge.cHat)
            print("Xm.hHat", Xm.hHat)

            if Vm.gT + currentEdge.cHat + Xm.hHat < self.goal.gT:                     # A1.14 
                print("passed check of a1.14")
                gHatVm = self.calcDist(Vm, self.start)  
                
                # look into the occupance grid, and if the line formed by Vm->Xm
                # is intersecting a 1, then set the cost == inf, else the cost = L2 norm
                collisionHappened = self.collisionCheck(Vm, Xm)
                print("collisionHappened", collisionHappened)

                ## my motion tree gets fully pruned if i do colliison checking
                if collisionHappened == True:
                    print(" COLLISION IN OBSTALCE SET")
                    realCost = inf 
                else:
                    print("NO COLLISION")
                    realCost = currentEdge.cHat                             
                    self.dbgAttemptedEdgeList.append(currentEdge)

                #realCost = currentEdge.cHat                             
                print("Vm.gT", Vm.gT)
                print("Xm.gT", Xm.gT)
                print("realCost", realCost)
                print("Vm.gHat", Vm.gHat)
                print("Xm.hHat", Xm.hHat)
                print("self.goal.gT", self.goal.gT)

                if Vm.gHat + realCost +Xm.hHat < self.goal.gT:                  # A1.15
                    print("passed check of #A1.15")
                    print("Vm.gT", Vm.gT)
                    print("realCost", realCost)
                    print("Xm.gT", Xm.gT)
                    if Vm.gT + realCost < Xm.gT:                                # A1.16
                        print("passed check of #A1.16")
                        print("type of Xm", type(Xm))
                        if Xm in self.V:                                        # A1.17 
                            print("state was in" )
                            edgeToPop = Edge()
                            for edge in self.E:
                                if edge.target_state == Xm:
                                    self.E.pop(edge)                             # A1.18
                                    break
                        else:                                                   # A1.19

                            print("doing the non member stuff")
                            print("Lenght of self.Xsamples before", len(self.Xsamples))
                            self.Xsamples.pop(Xm)                               #A1.20
                            print("Length of self.Xsamples after", len(self.Xsamples))
                            print("Length of Vertex Set A1.21 before", len(self.V))
                            self.V[Xm] = Xm                                     #A1.21
                            print("Length of Vertex Set A1.21 after", len(self.V))
                            print("is Xm in V?", Xm in self.V)

                            Xm.gT = Vm.gT + currentEdge.cHat
                            self.QvCount+=1
                            heapq.heappush(self.Qv, (Xm.gT,self.QvCount, Xm))   #A1.21
                        #self.E[currentEdge] = currentEdge 
                        print("EDGE ADDED TO MOTION TREE")
                        print("CHECK EDGE CONTAINS GOAL STATE")
                        if currentEdge.target_state == self.goal:
                            print("ADDED EDGE CONTAINS GOAL STATE")
                            edgeOfInterest = currentEdge
                            tmpCost =  0

                            print("ENTERING COST TRAVERSAL")
                            while edgeOfInterest.source_state != self.start:
                                tmpCost += edgeOfInterest.cHat
                                print("tmp Cost", tmpCost)
                                # i dont want to iterate thru the whole tree just to find the edge target_state = edgeOfInterest.source_state  
                                # but its whats going to finish my project before december 2
                                for edge in list(self.E.values()):
                                    if edge.target_state == edgeOfInterest.source_state:
                                        edgeOfInterest = edge
                                        break
                            tmpCost +=edgeOfInterest.cHat

                            print("FINAL tmpCost", tmpCost)
                            if tmpCost < self.c:
                                self.c = tmpCost
                        print(self.E)                                           
                        self.E[currentEdge] = currentEdge                       #A1.22
                        if (Vm.gT + currentEdge.cHat >= Xm.gT):                 #A1.23
                            heapq.heappop(self.Qe)                              #A1.23
            else:                                                               #A1.24
                print("Failed check of #A1.14")
                self.Qe = []                                                    #A1.25
                self.Qv = []                                                    #A1.25
           
        print("SELF.tmpWhile", self.tmpWhile)
        return self.V, self.E 

class Visualizer:
    def __init__(self):
        print("Visualizer")

    def plotMotionTree(self,V,E,obsMap,yMax, samples, start, goal):
        fig, ax = plt.subplots()
        print("Plotting the Motion Tree")
        startX = start.x
        startY = start.y
        goalX = goal.x
        goalY = goal.y
        plt.plot(startX,startY, "go", markersize=10)
        plt.plot(goalX,goalY, "ro", markersize=10)
        xVec = []
        yVec = []
        for edge in E:
            xVec.append(edge.source_state.x)
            xVec.append(edge.target_state.x)
            yVec.append(edge.source_state.y)
            yVec.append(edge.target_state.y)
            ax.plot(xVec, yVec, "m-x")
            ax.plot(xVec,yVec, "gx")
            xVec = []
            yVec = []
        #rect = patches.Rectangle( (50, 100), 10, 10, linewidth=1, edgecolor="r", facecolor = "r")
        #ax.add_patch(rect)

        xSampleVector = []
        ySampleVector = []
        for state in samples:
            xSampleVector.append(state.x)
            ySampleVector.append(state.y)
        plt.plot(xSampleVector,ySampleVector, "bo", markersize=2)

        # plotting the obstacles
        countY = 0
        for i in obsMap:
            countX = 0
            for j in i:
                if j == 1:
                    rect = patches.Rectangle((np.floor(countX),np.floor(yMax - countY-1)), 1,1, linewidth=1, edgecolor="r",facecolor="r")
                    ax.add_patch(rect) 
                countX+=1
            countY +=1
        plt.xlim((0,11))
        plt.ylim((0,11))
        plt.show()

    def plotAttemptedEdge(self, attemptedEdges, obsMap, yMax):
        print("plotting attempted edges")
        print("attemptedEdges", attemptedEdges)
        fig, ax = plt.subplots()
        xVec = []
        yVec = []
        for edge in attemptedEdges:
            xVec.append(edge.source_state.x)
            xVec.append(edge.target_state.x)
            yVec.append(edge.source_state.y)
            yVec.append(edge.target_state.y)
        ax.plot(xVec, yVec)
        countY = 0
        for i in obsMap:
            countX = 0
            for j in i:
                if j == 1:
                    rect = patches.Rectangle((np.floor(countX),np.floor(yMax - countY-1)), 1,1, linewidth=1, edgecolor="r",facecolor="r")
                    ax.add_patch(rect) 
                countX+=1
            countY +=1
        
        plt.xlim((0,11))
        plt.ylim((0,11))
        plt.show()

if __name__ == "__main__":
    vertices = open("vertices.txt","w")

    # for debugging, but i'm pretty sure randomness can cause issues
    random.seed(69)
    # input stuff
    #
    BS = BIT_STAR()
    BS.readEnvironment("test_environments/grid_envs/environment619.txt")
    #BS.readEnvironment("environment69.txt")
    #hit = BS.testCheckObs()
    #print("pritning hit")
    #print(hit)
    #print("asdfadsf")


    V,E = BS.BIT_STAR_MAIN()


    ##output stuff
    print("VERTICES")
    print(V)
    print(len(V))

    print("EDGES")
    print(E)
    print(len(E))
    print(BS.obs)
    for i in V:
        print("x", i.x)
        print("y", i.y)

    gv = Visualizer()
    gv.plotMotionTree(V,E,BS.obs, BS.yMax,BS.Xsamples,BS.start, BS.goal)
    #gv.plotAttemptedEdge(BS.dbgAttemptedEdgeList, BS.obs, BS.yMax)

