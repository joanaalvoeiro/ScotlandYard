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

    def search(self, init, limitexp = 2000, limitdepth = 10, tickets =  [math.inf,math.inf,math.inf], anyorder = False):
        if len(self._goal) == 1 and tickets ==  [math.inf,math.inf,math.inf]:
            final = search_1agent_nolim(init, self)

        elif len(self._goal) == 1:
            final = search_1agent_lim(init, tickets, self)

        elif tickets == []:
            final = search_3agent_nolim(init, self)

        elif anyorder:
            final = []

        else:
            final = []

        return final


#
# Exercise 1 - One agent, no tickets limmit
#
def search_1agent_nolim(init, self):
    #print("Map: {}".format(self._model))

    myMap = self._model
    goal = self._goal[0]

    visited = [ False ] * len(myMap)

    transport = [[]]

    done = False
    queue = [[init[0]]]

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

    final = [[[], init]] + [ [[T], [P]] for T, P in zip(currTrans, currPath[1:]) ]

    #print("final1: {}".format(final))

    return final


#
# Exercise 2 - One agent, tickets limmited
#
def has_ticket(tickets, type):
    return tickets[type] > 0

def search_1agent_lim(init, tickets, self):
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
def search_3agent_nolim(init, self):
    myMap = self._model
    goals = self._goal

    visited = [ set() for i in range(3) ]
    queues = [ [[init[i]]] for i in range(3) ]
    transport = [ [[]] for i in range(3) ]
    done = [ False for i in range(3) ]
    finals = [ [] for i in range(3) ]
    depth = [ 0 for i in range(3) ]

    #print('goal is ' + str(goal))


    while done != [True, True, True]:
        intents = []

        for agent in range(3):
            if done[agent]: continue
            currPath = queues[agent].pop(0)
            currTrans = transport[agent].pop(0)
            currVertex = currPath[-1]

            if currVertex not in visited[agent]:
                #print(currVertex)
                for i in myMap[currVertex]:


                    if i[1] == goals[agent]:
                        done[agent] = True
                        visited[agent].add(i[1])

                        newPath = list(currPath)
                        newPath.append(i[1])

                        newTrans = list(currTrans)
                        newTrans.append(i[0])

                        finals[agent] = newPath

                        #print('done agent ' + str(agent) + ' with path ' + str(newPath))
                        #print('done w path ' + str(newPath))
                        #print('transport: ' + str(newTrans))
                        break

                    newPath = list(currPath)
                    newPath.append(i[1])
                    #print(newPath)
                    queues[agent].append(newPath)

                    newTrans = list(currTrans)
                    newTrans.append(i[0])
                    transport[agent].append(newTrans)

                    visited[agent].add(currVertex)


    #print("final3: {}".format(final))

    return ['final']
