# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()
        return

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def search(problem, struct):
    # Recebe uma estrutura de dados propria para a busca solicitada
    struct.push([(problem.getStartState(), "Stop", 0)])
    # Lista com estados expandidos
    expandidos = []

    # Enquanto a estrutura nao estiver vazia
    while not struct.isEmpty():
        # Caminho retira um elemento da fila. O elemento constara em uma lista com o caminho percorrido do estado inicial ate determinado estado
        caminho = struct.pop()
        # estado recebe o successor do ultimo estado no caminho
        estado = caminho[len(caminho) - 1][0]
        # Se o estado ainda nao foi expandido
        if estado not in expandidos:
            # Se o estado for o objetivo
            if problem.isGoalState(estado):
                # directions e uma lista com todas as direcoes partindo do estado inicial para o final
                directions = []
                for estado in caminho:
                    # Adiciona os campos de action dos estados a directions
                    directions.append(estado[1])
                # Retorna a lista de direcoes
                return directions[1:]
            #Inclui o estado na lista de expandidos
            expandidos.append(estado)
            # Se nao for o estado final, verifica todos os sucessores do estado
            for sucessor in problem.getSuccessors(estado):
                # Se o sucessor nao foi expandido, cria uma nova lista com o caminho atual + o sucessor
                if sucessor[0] not in expandidos:
                    novoCaminho = caminho[:]
                    novoCaminho.append(sucessor)
                    # Insere na estrutura
                    struct.push(novoCaminho)
    return []





def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    pilha = util.Stack()
    return search(problem, pilha)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    fila = util.Queue()
    return search(problem, fila)

def createPath(finalNode, father):
    stack = util.Stack()
    path = []
    node = finalNode

    # While node is not its own father (not in start node)
    while node != father[node]:
        # Push node to stack
        stack.push(node[1])
        # node becomes it's father
        node = father[node]

    # Creates path from starting node to goal
    while not stack.isEmpty():
        value = stack.pop()
        path.append(value)

    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    # Caminho com estado inicial
    startNode = (problem.getStartState(), "Stop", 0)
    # Cria uma fila de prioridade
    borda = util.PriorityQueue()
    # Estrutura do tipo dicionario para o caminho de cada no ate o inicio
    caminho = {}
    # Estrutura do tipo dicionario para saber o pai de cada nodo
    father = {}

    # Inicia descobertos como 0 de distancia para os outros
    borda.push([startNode], 0)
    # Inicia o Caminho de startNode
    caminho[startNode] = 0
    # Lista com estados expandidos
    expandidos = []
    # Lista com os estados descobertos
    descobertos = []

    while not borda.isEmpty():
        # Pega o estado com menor custo na borda
        novoCaminho = borda.pop()
        estado = novoCaminho[len(novoCaminho) - 1]
        # Se for o estado objetivo retorna o caminho
        if problem.isGoalState(estado[0]):
            return [x[1] for x in novoCaminho][1:]

        # Para cada sucessor do estado
        if not(estado[0] in expandidos):
            for sucessor in problem.getSuccessors(estado[0]):
                # Se ainda nao conhecemos
                caminhoSucessor = novoCaminho[:]
                caminhoSucessor.append(sucessor)
                if  not ((sucessor in expandidos) or (sucessor in descobertos)):
                    caminho[sucessor] = caminho[estado] + sucessor[2]
                    descobertos.append(sucessor)
                    borda.push(caminhoSucessor, caminho[sucessor])
                else:
                    if caminho[sucessor] > caminho[estado] + sucessor[2]:
                        caminho[sucessor] = caminho[estado] + sucessor[2]

            # Estado espandido
            expandidos.append(estado[0])

    # Nao deveria chegar aqui (caso de erro)
    return []

def hillClimbingSearch(problem, heuristic):
    # i = false indica um pico
    i = True
    # Estado inicial
    estado = (problem.getStartState(), "Stop", 0)
    expandidos = []
    # Lista com o caminho percorrido
    caminho = []
    while i:
        i = False
        # Verifica todos os sucessores do estado atual
        for sucessor in problem.getSuccessors(estado[0]):
            # Compara as distancias dos estados atual e sucessor em relacao ao objetivo
            if heuristic(estado[0], problem) > heuristic(sucessor[0], problem):
                # Se encontrar um maior, continua procurando
                i = True
                # sucessor vira o estado atual
                estado = sucessor
        # Adiciona o estado aos explorados
        # Se um novo estado foi visitado, inclui no caminho
        if i:
            expandidos.append(estado)
            caminho.append(estado[1])
    # Retorna a lista de direcoes
    return caminho

def simulatedAnnealingSearch(problem, schedule, heuristic):
    # Estado inicial
    caminho = [(problem.getStartState(), "Stop", 0)]
    expandidos = []
    t = 1
    while true:
        T = schedule(t)
        if T == 0:
            return caminho[1:]
        estado = caminho[len(caminho - 1)]
        sucessor = random(problem.getSuccessors(estado))
        deltaE = heuristic(estado[0], problem) - heuristic(sucessor[0], problem)
        expandidos.append(sucessor)
        if deltaE > 0:
            caminho.append(sucessor)
        "else if "


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    import util
    from util import manhattanDistance

    # Important variables
    startState = (problem.getStartState(), "Stop", 0)

    # Priority Queue for non explored nodes
    nonExplored = util.PriorityQueue()
    nonExplored.push(startState, heuristic(startState[0], problem))

    # List of explored nodes (empty at begin)
    explored = []
    discovered = []

    # Dictionary that keeps all explored nodes fathers (they migth be changed)
    father = {}
    # Dicttionary that keeps the shortest distance between explored nodes and the inicial one
    distance = {}

    # Initialize both structures with initial info about starter node
    father[startState] = startState
    distance[startState] = startState[2]
    # ** Distance considers manhattanDistance

    # While there is a node to explore
    while not nonExplored.isEmpty():
        # Get node with less heuristic + distance value
        newNode = nonExplored.pop()

        # If it is the goal Node, create path from begin to it
        if problem.isGoalState(newNode[0]):
            return createPath(newNode, father)

        # For each possible successor of node
        if not(newNode[0] in explored):
            for successor in problem.getSuccessors(newNode[0]):
                # If successor was already discovered (found or explored)
                if not successor in discovered:
                    # Create father and distance
                    distance[successor] = distance[newNode] + successor[2]
                    father[successor] = newNode
                    # Includes in discovered nodes to be explored
                    nonExplored.push(successor, heuristic(successor[0], problem) + distance[successor])
                    discovered.append(successor)
            # newNode was explored
            explored.append(newNode[0])

    # We should never get to it
    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
hcs = hillClimbingSearch
sas = simulatedAnnealingSearch
