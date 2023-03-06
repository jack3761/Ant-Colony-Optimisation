import networkx as nx
import numpy as np
import multiprocessing as mp
import random
import time

#class for the ant to generate its paths with all appropriate methods
class Ant():
    visitedNodes = []
    allowedNodes = []
    path = []
    totFitness = 0

    def __init__(self, n, G):
        self.visitedNodes = []
        self.allowedNodes = []
        self.totFitness = 0
        for i in range(n):
            self.allowedNodes.append(i)

    #randomly finds the next node with a bias based on the pheremone
    def findNextNode(self, G, currentNode):

        #if there's only one allowed node left the next node is declared as that one
        if len(self.allowedNodes) == 1:
            next_node = self.allowedNodes[0]


        #update the visited and allowed node lists
        self.visitedNodes.append(currentNode)
        self.allowedNodes.remove(currentNode)
        
        
        # Select the next node using roulette wheel selection
        rand = random.random()
        cumulative_prob = 0
        count = 0

        for node in self.allowedNodes:
            count += 1
            prob = G[currentNode][node]['pheremone']*random.random()**2
            cumulative_prob += prob
            if rand < cumulative_prob:
                next_node = node
                break
            else:
                next_node = self.allowedNodes[-1]        
        
        return next_node

    #function to create the solution path
    def createPath(self, G, start):
        currentNode = start
        p = []
        #keep finding the next node until there are no more available nodes to find
        while len(self.allowedNodes) > 0:
            nextNode = self.findNextNode(G, currentNode)
            p.append(nextNode)
            if currentNode != nextNode:
                edgeFitness = G[currentNode][nextNode]['distance'] * G[currentNode][nextNode]['flow']
                self.updatePheremones(currentNode, nextNode, edgeFitness)
                currentNode = nextNode
        
        self.totFitness = findCost(p)
        return p

    def updatePheremones(self, currentNode, nextNode, fitness):
        if fitness != 0:
            G[currentNode][nextNode]['pheremone'] += 1/fitness
            


#function to return the matrices and the size of the matrices from the input data
def readFile(fileName):
    file = open(fileName)
    n = int(file.readline())
    blank = file.readline()
    D = []
    for i in range(n):
        D.append([int (x) for x in file.readline().split()])
        
    blank = file.readline()
    F = []
    for i in range(n):
        F.append([int (x) for x in file.readline().split()])
    return n, D, F

#construct the graph using the inputs given
def constructGraph(G, n, D, F, P):
    G.add_nodes_from(range(n))
    for i in range(n):
        for j in range(n):
            if i==j:
                continue
            else:
                G.add_edge(i, j)
                G[i][j]['distance'] = D[i][j]
                G[i][j]['flow'] = F[i][j]
                G[i][j]['pheremone'] = P[i][j]
    return G

#randomly generate a pheremone matrix
def generatePheremones(n):
    P = np.empty((n, n), dtype=float)
    for i in range(n):
        for j in range(n):
            if i == j:
                P[i][j] = 0
            else:
                P[i][j] = random.random()
    return P

#on each iteration, update all pheremones by the evaporation rate
def evaporatePheremones(G, e):
    for u, v in G.edges():
        G[u][v]['pheremone'] = G[u][v]['pheremone'] * e
    return G

#find the total cost based on the solution path given using the cost function
def findCost(p):
    cost = 0
    for i in range(len(p)):
        for j in range(len(p)):
            if i != j:
                cost += D[i][j] * F[p[i]][p[j]]
    return cost



#main program to run all iterations of the code
def run(e):
    startTime = time.time()
    bestFitness = startCost
    solution = []
    #randomly select start node
    startNode = random.randint(0, 49)
    print("Generating ants")
    for i in range(10000):
        #generate m ant paths on each iteration
        ants = [Ant(n, G) for i in range(m)]
        for j in range(m):
            ants[j].createPath(G, startNode)
            #find the best fitness for this run of the program
            if ants[j].totFitness < bestFitness:
                bestFitness = ants[j].totFitness
                solution = ants[j].visitedNodes
        evaporatePheremones(G, e)

    endTime = time.time()
    print("\n\n")
    print("Time taken: " + str(round(endTime-startTime)))
    print("Start cost: " + str(startCost))
    print("Cost: " + str(bestFitness))
    print("Improvement: " + str(round(100-(bestFitness/startCost*100), 2)) + "%")
    print(solution)

    return bestFitness

#extract the size of the matrices and the matrices from the data file
n, D, F = readFile("Uni50a.dat")
P = generatePheremones(n)

#construct the graph from the extracted matrices
G = nx.Graph()
G = constructGraph(G, n, D, F, P)

#create a generic start solution to be compared to
startPath = []
for i in range(n):
    startPath.append(i)
startCost = findCost(startPath)

#state the amount of trials to be run in parallel
numProcesses = 5
m = 10
e = 0.9

print("m: " + str(m) + "   e: " + str(e))
#run trials with the given parameters in parallel
if __name__ == '__main__':
    with mp.Pool(numProcesses) as pool:
        paths = pool.map(run, [e, e, e, e, e])
