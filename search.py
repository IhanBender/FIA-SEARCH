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
    "*** YOUR CODE HERE ***"
    pilha = util.Stack()
    return search(problem, pilha)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    fila = util.Queue()
    return search(problem, fila)

def uniformCostSearch(problem, goal):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Caminho com estado inicial
    caminho = [(problem.getStartState(), "Stop", 0)]
    # Cria uma fila de prioridade
    borda = util.PriorityQueue()
    # Insere itens na lista utilizando o custo do caminho como prioridade
    borda.push(caminho[0], problem.getCostOfActions(caminho[1]))
    # Lista com estados expandidos
    expandidos = []
    while true:
        # Se a borda esta vazia retorna falha
        if borda.isEmpty():
            return []
        # Pega o estado com menor custo na borda
        estado = borda.pop()
        # Se for o estado objetivo retorna o caminho
        if problem.isGoalState(estado[0]):
            return (len(expandidos), caminho, len(caminho))
        # Adiciona o estado a lista de expandidos
        explorado.append(estado)
        # Para cada sucessor do estado
        for sucessor in problem.getSuccessors(estado[0]):
            """
            se(filho[0]) nao esta na borda ou explorado entao
                borda <-- INSIRA(caminho-ate-filho, borda)
            senao se (filho[0]) esta na borda com maior CUSTO-DE-CAMINHO entao
                substituir aquele no borda por caminho-ate-filho
            """

def hillClimbingSearch(problem, goal):
    # i = false indica um pico
    i = true
    # Estado inicial
    estado = (problem.getStartState(), "Stop", 0)
    expandidos = []
    # Lista com o caminho percorrido
    caminho = []
    while i:
        i = false
        # Verifica todos os sucessores do estado atual
        for sucessor in problem.getSuccessors(estado):
            # Compara as distancias dos estados atual e sucessor em relacao ao objetivo
            if util.manhattanDistance(estado[0], goal) > util.manhattanDistance(sucessor[0], goal):
                # Se encontrar um maior, continua procurando
                i = true
                # sucessor vira o estado atual
                estado = sucessor
        # Adiciona o estado aos explorados
        # Se um novo estado foi visitado, inclui no caminho 
        if i:
            explorados.append(estado)
            caminho.append(estado[1])
    # directions e uma lista com todas as direcoes partindo do estado inicial para o final
    directions = []
    for estado in caminho:
        # Adiciona os campos de action dos estados a directions
        directions.append(estado[1])
    # Retorna a lista de direcoes
    return (len(expandidos), directions[1:], len(caminho))

def simulatedAnnealingSearch(problem, goal, schedule):
    # Estado inicial
    caminho = [(problem.getStartState(), "Stop", 0)]
    expandidos = []
    t = 1
    while true:
        T = schedule(t)
        if T == 0:
            return (len(expandidos), caminho[1:], len(caminho))
        estado = caminho[len(caminho - 1)]
        sucessor = random(problem.getSuccessors(estado))
        deltaE = problem.getCostOfActions(sucessor) - problem.getCostOfActions(estado)
        expandidos.append(sucessor)
        if deltaE > 0:
            caminho.append(sucessor)
        "else current = next only with probability e^(deltaE/T):


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
