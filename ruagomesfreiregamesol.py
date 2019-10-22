# 86752 Afonso Ribeiro - Joana Alvoeiro

import math
import pickle
import time


class Map_Handler():

    def __init__(self, Map):
        taxi_grid = [[]]
        bus_grid = [[]]
        metro_grid = [[]]
        has_bus = []
        has_metro = []

        for v in range(1, len(Map)):
            vertex = Map[v]
            vertex_taxi = []
            vertex_bus = []
            vertex_metro = []

            for dest in vertex:
                if dest[0] == 0:
                    vertex_taxi.append(dest[1])
                elif dest[0] == 1:
                    vertex_bus.append(dest[1])
                elif dest[0] == 2:
                    vertex_metro.append(dest[1])

            if vertex_bus != []:
                    has_bus.append(v)
            if vertex_metro != []:
                    has_metro.append(v)

            taxi_grid.append(vertex_taxi)
            bus_grid.append(vertex_bus)
            metro_grid.append(vertex_metro)

        self._taxi_grid = taxi_grid
        self._bus_grid = bus_grid
        self._metro_grid = metro_grid
        self._has_bus = has_bus
        self._has_metro = has_metro

    def get_taxi_grid():
        return self._taxi_grid

    def get_bus_grid():
        return self._bus_grid

    def get_has_bus_list():
        return self._has_bus

    def get_metro_grid():
        return self._metro_grid

    def get_has_metro_list():
        return self._has_metro


def distance(auxheur, v1, v2):
    c1 = auxheur[v1-1]
    c2 = auxheur[v2-1]
    return (c1[0]-c2[0])**2 + (c1[1]-c2[1])**2

  
class SearchProblem:

    def __init__(self, goal, model, auxheur = []):
        self._goal = goal
        self._model = model
        self._auxheur = auxheur
        self._map_handler = Map_Handler(model)

    def search(self, init, limitexp = 2000, limitdepth = 10, tickets = [math.inf,math.inf,math.inf], anyorder = False):
        if len(self._goal) == 1 and tickets ==  [math.inf,math.inf,math.inf]:
            return search_1agent_nolim(self, init, limitexp, limitdepth)

        elif len(self._goal) == 1:
            return search_1agent_lim(self, init, tickets, limitexp, limitdepth)

        elif tickets == [math.inf,math.inf,math.inf]:
            return search_3agent_nolim(self, init, limitexp, limitdepth)

        elif anyorder:
            return []

        else:
            return []


#
# Exercise 1 - One agent, no tickets limit
#
def bfs(myMap, init, goal, limitexp, limitdepth):
    visited = set()
    transport = [[]]

    done = False
    queue = [[init]]

    exp = 0
    depth = 0
    while queue and not done and exp <= limitexp and depth <= limitdepth:
        currTrans = transport.pop(0)
        currPath = queue.pop(0)
        depth = len(currPath)
        currVertex = currPath[-1]

        if currVertex == goal:
            done = True
            continue

        if currVertex not in visited:
            for option in myMap[currVertex]:

                if option[1] == goal:
                    currPath.append(option[1])
                    currTrans.append(option[0])
                    depth = len(currPath)
                    exp += 1
                    done = True
                    break

                if option[1] in visited: continue

                queue.append( currPath + [option[1]] )
                transport.append( currTrans + [option[0]] )

            visited.add(currVertex)
            exp += 1


    return currTrans, currPath[1:]


def search_1agent_nolim(self, init, limitexp, limitdepth):
    #print("Map: {}".format(self._model))

    transport, path = bfs(self._model, init[0], self._goal[0], limitexp, limitdepth)

    final = [[[], init]] + [ [[T], [P]] for T, P in zip(transport, path) ]

    #print("final1: {}".format(final))

    return final


#
# Exercise 2 - One agent, tickets limitted
#
def has_ticket(tickets, type):
    return tickets[type] > 0

def search_1agent_lim(self, init, tickets, limitexp, limitdepth):
    myMap = self._model
    goal = self._goal[0]

    myTickets = [tickets]

    visited = [set()]
    transport = [[]]

    done = False
    queue = [[init[0]]]

    exp = 0
    depth = 0
    while queue and not done:
        currPath = queue.pop(0)
        depth = len(currPath)
        currTrans = transport.pop(0)
        currVisited = visited.pop(0)
        currTickets = myTickets.pop(0)
        currVertex = currPath[-1]

        if currVertex == goal:
            done = True
            continue

        # Also isn't worth if i've been to this position with >= tickets (optimization maybe)
        for option in myMap[currVertex]:
            if has_ticket(currTickets, option[0]) and option[1] not in currVisited:

                if option[1] == goal:
                    currPath.append(option[1])
                    currTrans.append(option[0])
                    depth = len(currPath)
                    exp += 1
                    done = True
                    break

                queue.append( currPath + [option[1]] )
                transport.append( currTrans + [option[0]] )

                newTickets = currTickets.copy()
                newTickets[option[0]] += -1
                myTickets.append(newTickets)

                newVisited = currVisited.copy()
                newVisited.add(currVertex)
                visited.append(newVisited)

        exp += 1


    print("exp {} and depth {}".format(exp, depth) )

    
    final = [[[], init]] + [ [[T], [P]] for T, P in zip(currTrans, currPath[1:]) ]

    #print("final2: {}".format(final))

    return final


#
# Exercise 3 - Three agents, no tickets limit
#
def triple_bfs(myMap, init, goal, limitexp, limitdepth):
    transport = [[[]]]

    done = False
    queue = [[init]]

    while queue and not done:
        currTrans3 = transport.pop(0)
        currPath3 = queue.pop(0)
        currVertexes3 = currPath3[-1]

        if currVertexes3 == goal:
            done = True
            continue



        tinit = time.process_time()

        possibilities = [ myMap[currVertex] for currVertex in currVertexes3 ]
        combinations = [ [option1, option2, option3] for option1 in possibilities[0] for option2 in possibilities[1] for option3 in possibilities[2] ]

        valid_combinations = list( filter( lambda option: option[0][1] != option[1][1] != option[2][1] != option[0][1], combinations ) )


        tend = time.process_time()
        print("%.1fms"%((tend-tinit)*1000))

        #print('possibilities: {}\ncombinations: {}\nvalid_combinations: {}'.format(possibilities, combinations, valid_combinations))

        for option in valid_combinations:
            nextPos = [ option[i][1] for i in range(3) ]
            nextTrans = [ option[i][0] for i in range(3) ]

            if nextPos == goal:
                currPath3.append(nextPos)
                currTrans3.append(nextTrans)
                done = True
                break

            queue.append( currPath3 + [nextPos] )
            transport.append( currTrans3 + [nextTrans] )


    final = [ [T, P] for T, P in zip(currTrans3, currPath3) ]

    return final

def path_of_size(myMap, init, goal, limitexp, limitdepth, size):
    transport = [[]]

    done = False
    queue = [[init]]

    exp = 0
    depth = 1
    while queue and not done and exp <= limitexp and depth <= limitdepth:

        depth = len(queue[0])
        if depth == size:
            done = True
            continue

        currTrans = transport.pop(0)
        currPath = queue.pop(0)
        currVertex = currPath[-1]
        
        for option in myMap[currVertex]:

            if depth == size - 1:
                if option[1] == goal:
                    queue.append( currPath + [option[1]] )
                    transport.append( currTrans + [option[0]] )

            else:
                queue.append( currPath + [option[1]] )
                transport.append( currTrans + [option[0]] )

        exp += 1

    if queue == []: return [], []

    return queue, transport


def is_solution(path0, path1, path2):
    if not (len(path0) == len(path1) == len(path2)):
        return False

    for i in range(len(path0)):
        if not (path0[i] != path1[i] != path2[i] != path0[i]):
            return False

    return True


def search_3agent_nolim(self, init, limitexp, limitdepth):

    def return_format(init, pair0, pair1, pair2):
        return [[[], init]] + [ [[pair0[0][i], pair1[0][i], pair2[0][i]], [pair0[1][i], pair1[1][i], pair2[1][i]]] for i in range(len(pair0[0])) ]

    myMap = self._model
    goals = self._goal

    #print('goal is ' + str(goal))

    pair0, pair1, pair2 = [ bfs(myMap, init[i], goals[i], limitexp, limitdepth) for i in range(3) ]

    if is_solution(pair0[1], pair1[1], pair2[1]):
        return return_format(init)

    worst = pair0
    if len(worst[0]) < len(pair1[0]):
        worst = pair1

    if len(worst[0]) < len(pair2[0]):
        worst = pair2



    print("final3: {}".format("df"))

    return [] #final
