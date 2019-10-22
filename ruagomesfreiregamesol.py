# 86752 Afonso Ribeiro - Joana Alvoeiro

import math
import pickle
import time

  
class SearchProblem:

    def __init__(self, goal, model, auxheur = []):
        self._goal = goal
        self._model = model
        self._auxheur = auxheur
        pass

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
# Exercise 1 - One agent, no tickets limmit
#
def bfs(myMap, init, goal, limitexp, limitdepth):
    visited = [ False ] * len(myMap)
    transport = [[]]

    done = False
    queue = [[init]]

    while queue and not done:
        currTrans = transport.pop(0)
        currPath = queue.pop(0)
        currVertex = currPath[-1]

        if currVertex == goal:
            done = True
            continue

        if not visited[currVertex]:
            for option in myMap[currVertex]:

                if option[1] == goal:
                    currPath.append(option[1])
                    currTrans.append(option[0])
                    done = True
                    break

                if visited[option[1]]: continue

                queue.append( currPath + [option[1]] )
                transport.append( currTrans + [option[0]] )

            visited[currVertex] = True

    final = [[[], [init]]] + [ [[T], [P]] for T, P in zip(currTrans, currPath[1:]) ]

    #print("final1: {}".format(final))

    return final


def search_1agent_nolim(self, init, limitexp, limitdepth):
    #print("Map: {}".format(self._model))

    return bfs(self._model, init[0], self._goal[0], limitexp, limitdepth) 


#
# Exercise 2 - One agent, tickets limmited
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

    while queue and not done:
        currPath = queue.pop(0)
        currTrans = transport.pop(0)
        currVisited = visited.pop(0)
        currTickets = myTickets.pop(0)
        currVertex = currPath[-1]

        if currVertex == goal:
            done = True
            continue

        # Also isn't worth if i've been to this position with >= tickets (otimization maiby)
        for option in myMap[currVertex]:
            if has_ticket(currTickets, option[0]) and option[1] not in currVisited:

                if option[1] == goal:
                    currPath.append(option[1])
                    currTrans.append(option[0])
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

    
    final = [[[], init]] + [ [[T], [P]] for T, P in zip(currTrans, currPath[1:]) ]

    #print("final2: {}".format(final))

    return final


#
# Exercise 3 - Three agents, no tickets limmit
#
def search_3agent_nolim(self, init, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal

    visited = [ set() for i in range(3) ]
    queues = [ [[init[i]]] for i in range(3) ]
    transport = [ [[]] for i in range(3) ]
    done = [ False for i in range(3) ]
    finals = [ [] for i in range(3) ]
    depth = [ 0 for i in range(3) ]

    #print('goal is ' + str(goal))

    finals = [ bfs(myMap, init[i], goals[i], limitexp, limitdepth) for i in range(3) ]

    print(finals)



    #print("final3: {}".format(final))

    return ['final']
