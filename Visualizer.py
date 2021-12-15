import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
class Visualizer:
    def __init__(self):
        print("Visualizer")
        self.f,self.ax = plt.subplots()

    def plotMotionTree(self,V,E,obsMap,xMax, yMax, samples, start, goal, oldEdges):
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
            #ax.plot(xVec, yVec, "r-x")
            ax.plot(xVec, yVec, "-", color="#FFA71A")
            #ax.plot(xVec,yVec, "r")
            xVec = []
            yVec = []
        #rect = patches.Rectangle( (50, 100), 10, 10, linewidth=1, edgecolor="r", facecolor = "r")
        #ax.add_patch(rect)

        xVec = []
        yVec = []
        
        xSampleVector = []
        ySampleVector = []
        for state in samples:
            xSampleVector.append(state.x)
            ySampleVector.append(state.y)
        plt.plot(xSampleVector,ySampleVector, "o", color = "#757575", markersize=0.75)

        # plotting the obstacles
        countY = 0
        for i in obsMap:
            countX = 0
            for j in i:
                if j == 1:
                    rect = patches.Rectangle((np.floor(countX),np.floor(yMax - countY-1)), 1,1, linewidth=1, edgecolor="k",facecolor="k")
                    ax.add_patch(rect) 
                countX+=1
            countY +=1
        plt.plot(startX,startY, "go", markersize=10)
        plt.plot(goalX,goalY, "ro", markersize=10)
        xVecSolu = []
        yVecSolu = []
        
        foundGoal = False
        stateOfInterest = goal
        while stateOfInterest != start:
            for e in E:
                if e.target_state == stateOfInterest:
                    foundGoal = True
                    xVecSolu.append(e.source_state.x)
                    yVecSolu.append(e.source_state.y)
                    xVecSolu.append(e.target_state.x)
                    yVecSolu.append(e.target_state.y)
                    stateOfInterest = e.source_state
                    plt.plot(xVecSolu, yVecSolu, color="#007FFF", linewidth=3)
                    xVecSolu = []
                    yVecSolu = []
            if foundGoal == False:
                break
                         

                    
        plt.xlim((0,xMax))
        plt.ylim((0,yMax))
        plt.axis("equal")
        plt.title("Motion Tree, Samples, and Solution Path")
        plt.xlabel("Distance (m)")
        plt.ylabel("Distance (m)")
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

        # too lazy
        plt.xlim((0,50))
        plt.ylim((0,50))
        plt.show()

    def plotEnv(self, PLAN):
        obsMap = PLAN.obs
        xMax = PLAN.xMax
        yMax = PLAN.yMax
        # plotting the obstacles
        countY = 0
        for i in obsMap:
            countX = 0
            for j in i:
                if j == 1:
                    rect = patches.Rectangle((np.floor(countX),np.floor(yMax - countY-1)), 1,1, linewidth=1, edgecolor="k",facecolor="k")
                    self.ax.add_patch(rect) 
                countX+=1
            countY +=1

    def plotSamples(self, PLAN):
        samples = PLAN.Xsamples
        xSampleVector = []
        ySampleVector = []
        for state in samples:
            xSampleVector.append(state.x)
            ySampleVector.append(state.y)
        self.ax.plot(xSampleVector,ySampleVector, "o", color = "#757575", markersize=2)

    def plotEdges(self, PLAN):
        xVec = []
        yVec = []
        print("PLOT EDGES")
        for edge in PLAN.E:
            
            xVec.append(edge.source_state.x)
            xVec.append(edge.target_state.x)
            yVec.append(edge.source_state.y)
            yVec.append(edge.target_state.y)

            #print(edge.source_state.x)
            #print(edge.target_state.x)
            #print(edge.source_state.y)
            #print(edge.target_state.y)

            #ax.plot(xVec, yVec, "r-x")
            self.ax.plot(xVec, yVec, "-", color="#FFA71A")
            #ax.plot(xVec,yVec, "r")
            xVec = []
            yVec = []

    def lcPlotSolution(self, X,Y, colorString, width):
        self.ax.plot(X,Y, color=colorString, linewidth=width)
    def plotSolutionEdges(self, stateList):
        xVec = []
        yVec = []
        print("PLOT EDGES")
        for state in stateList:
            
            xVec.append(state.x)
            yVec.append(state.y)

            #print(edge.source_state.x)
            #print(edge.target_state.x)
            #print(edge.source_state.y)
            #print(edge.target_state.y)

            #ax.plot(xVec, yVec, "r-x")
            self.ax.plot(xVec, yVec, "-", color="#007FFF")
            #ax.plot(xVec,yVec, "r")
            xVec = []
            yVec = []
        


    def labelLastChance(self,PLAN):
        startX = PLAN.start.x
        startY = PLAN.start.y
        goalX = PLAN.goal.x
        goalY = PLAN.goal.y
        self.ax.plot(startX,startY, "go", markersize=10)
        self.ax.plot(goalX,goalY, "ro", markersize=10)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Explicit RGG")
        self.ax.axis("equal")
        plt.show()
