# 86752 Afonso Ribeiro - 89469 Joana Alvoeiro

import math
import pickle
import time

  
class SearchProblem:

    def __init__(self, goal, model, auxheur = []):
        self._goal = goal
        self._model = model
        self._auxheur = auxheur

    def search(self, init, limitexp = 2000, limitdepth = 10, tickets = [math.inf,math.inf,math.inf], anyorder = False):
        if len(self._goal) == 1 and tickets ==  [math.inf,math.inf,math.inf]:
            return search_1agent_nolim(self, init, limitexp, limitdepth)

        elif len(self._goal) == 1:
            return search_1agent_lim(self, init, tickets, limitexp, limitdepth)

        elif tickets == [math.inf,math.inf,math.inf]:
            return search_3agent_nolim(self, init, limitexp, limitdepth)

        elif not anyorder:
            return search_3agent_lim(self, init, tickets, limitexp, limitdepth)
        else:
            return search_3agent_lim_anyorder(self, init, tickets, limitexp, limitdepth)


#
# Exercise 1 - One agent, no tickets limit
#
def bfs(myMap, init, goal, limitexp, limitdepth):
    visited = set()
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

        if currVertex not in visited:
            for option in myMap[currVertex]:

                if option[1] == goal:
                    currPath.append(option[1])
                    currTrans.append(option[0])
                    done = True
                    break

                if option[1] in visited: continue

                queue.append( currPath + [option[1]] )
                transport.append( currTrans + [option[0]] )

            visited.add(currVertex)

    final = [[[], [init]]] + [ [[T], [P]] for T, P in zip(currTrans, currPath[1:]) ]

    #print("final1: {}".format(final))

    return final


def search_1agent_nolim(self, init, limitexp, limitdepth):
    #print("Map: {}".format(self._model))

    return bfs(self._model, init[0], self._goal[0], limitexp, limitdepth) 


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

    while queue and not done:
        currPath = queue.pop(0)
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
# Exercise 3 - Three agents, no tickets limit
#
def triple_bfs(myMap, init, goal, limitexp, limitdepth):
    transport = [[[]]]

    done = False
    queue = [[init]]

    depth = -1
    configurations = set()

    while queue and not done:
        currTrans3 = transport.pop(0)
        currPath3 = queue.pop(0)
        currVertexes3 = currPath3[-1]

        if currVertexes3 == goal:
            done = True
            continue

        possibilities = [ myMap[currVertex] for currVertex in currVertexes3 ]
        combinations = [ [option1, option2, option3] for option1 in possibilities[0] \
                        for option2 in possibilities[1] for option3 in possibilities[2] \
                        if option1[1] != option2[1] != option3[1] != option1[1] \
                        and (option1[1], option2[1], option3[1]) not in configurations ]

        #print("combinations: {}".format(len(combinations)))

        #print('possibilities: {}\ncombinations: {}\nvalid_combinations: {}'.format(possibilities, combinations, valid_combinations))

        for option in combinations:
            nextPos = [ option[i][1] for i in range(3) ]
            nextTrans = [ option[i][0] for i in range(3) ]
            configurations.add( ( option[0][1], option[1][1], option[2][1]) )

            if nextPos == goal:
                currPath3.append(nextPos)
                currTrans3.append(nextTrans)
                done = True
                break

            queue.append( currPath3 + [nextPos] )
            transport.append( currTrans3 + [nextTrans] )


    final = [ [T, P] for T, P in zip(currTrans3, currPath3) ]

    return final


def search_3agent_nolim(self, init, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal

    #print('goal is ' + str(goal))

    final = triple_bfs(myMap, init, goals, limitexp, limitdepth)

    #print("final3: {}".format(final))

    return final


#
# Exercise 4 - Three agents, tickets limited
#
def triple_bfs_lim(myMap, init, goal, tickets, limitexp, limitdepth):
    transport = [[[]]]
    tickets = [tickets]

    done = False
    queue = [[init]]

    depth = -1
    configurations = set()

    while queue and not done:
        currTickets = tickets.pop(0)
        currTrans3 = transport.pop(0)
        currPath3 = queue.pop(0)
        currVertexes3 = currPath3[-1]

        if currVertexes3 == goal:
            done = True
            continue

        possibilities = [ myMap[currVertex] for currVertex in currVertexes3 ]
        combinations = [ [option1, option2, option3] for option1 in possibilities[0] \
                        for option2 in possibilities[1] for option3 in possibilities[2] \
                        if option1[1] != option2[1] != option3[1] != option1[1] \
                        and (option1[1], option2[1], option3[1]) not in configurations ]

        #print("combinations: {}".format(len(combinations)))

        #print('possibilities: {}\ncombinations: {}\nvalid_combinations: {}'.format(possibilities, combinations, valid_combinations))

        for option in combinations:
            nextPos = [ option[i][1] for i in range(3) ]
            nextTrans = [ option[i][0] for i in range(3) ]
            newTickets = currTickets.copy()

            invalid_ticket_n = False
            for trans_index in nextTrans:
                newTickets[trans_index] += -1
                if newTickets[trans_index] < 0:
                    invalid_ticket_n = True

            if invalid_ticket_n:
                continue

            configurations.add( ( option[0][1], option[1][1], option[2][1]) )

            if nextPos == goal:
                currPath3.append(nextPos)
                currTrans3.append(nextTrans)
                done = True
                break

            queue.append( currPath3 + [nextPos] )
            transport.append( currTrans3 + [nextTrans] )
            tickets.append( newTickets )


    final = [ [T, P] for T, P in zip(currTrans3, currPath3) ]

    return final


def search_3agent_lim(self, init, tickets, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal

    #print('goal is ' + str(goal))

    final = triple_bfs_lim(myMap, init, goals, tickets, limitexp, limitdepth)

    #print("final4: {}".format(final))

    return final

#
# Exercise 5 - Three agents, tickets limited
#
def triple_bfs_lim_anyorder(myMap, init, goal, tickets, limitexp, limitdepth):
    transport = [[[]]]
    tickets = [tickets]

    goals = [ [a, b, c] for a in goal for b in goal for c in goal if a != b != c != a ]

    done = False
    queue = [[init]]

    depth = -1
    configurations = set()

    while queue and not done:
        currTickets = tickets.pop(0)
        currTrans3 = transport.pop(0)
        currPath3 = queue.pop(0)
        currVertexes3 = currPath3[-1]

        if currVertexes3 in goals:
            done = True
            continue

        possibilities = [ myMap[currVertex] for currVertex in currVertexes3 ]
        combinations = [ [option1, option2, option3] for option1 in possibilities[0] \
                        for option2 in possibilities[1] for option3 in possibilities[2] \
                        if option1[1] != option2[1] != option3[1] != option1[1] \
                        and (option1[1], option2[1], option3[1]) not in configurations ]

        #print("combinations: {}".format(len(combinations)))

        #print('possibilities: {}\ncombinations: {}\nvalid_combinations: {}'.format(possibilities, combinations, valid_combinations))

        for option in combinations:
            nextPos = [ option[i][1] for i in range(3) ]
            nextTrans = [ option[i][0] for i in range(3) ]
            newTickets = currTickets.copy()

            invalid_ticket_n = False
            for trans_index in nextTrans:
                newTickets[trans_index] += -1
                if newTickets[trans_index] < 0:
                    invalid_ticket_n = True

            if invalid_ticket_n:
                continue

            configurations.add( ( option[0][1], option[1][1], option[2][1]) )

            if nextPos in goals:
                currPath3.append(nextPos)
                currTrans3.append(nextTrans)
                done = True
                break

            queue.append( currPath3 + [nextPos] )
            transport.append( currTrans3 + [nextTrans] )
            tickets.append( newTickets )


    final = [ [T, P] for T, P in zip(currTrans3, currPath3) ]

    return final


def search_3agent_lim_anyorder(self, init, tickets, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal

    #print('goal is ' + str(goal))

    final = triple_bfs_lim_anyorder(myMap, init, goals, tickets, limitexp, limitdepth)

    #print("final5: {}".format(final))

    return final
