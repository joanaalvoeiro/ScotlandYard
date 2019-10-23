# 86752 Afonso Ribeiro - 89469 Joana Alvoeiro

import math
import pickle
import time

heuristic = []
h_trans = []

def floyd_warshall(model):

    V = len(model)

    no_transports_model = [ [option[1] for option in dest] for dest in model ]

    global heuristic
    heuristic = [ [1 if j in no_transports_model[i] and j != i else math.inf for j in range(V) ] for i in range(V) ]


    global h_trans
    h_trans = [ [[] for j in range(V) ] for i in range(V) ]

    for k in range(V):
        for i in range(V):
            for j in range(V):
                heuristic[i][j] = min(heuristic[i][j], heuristic[i][k] + heuristic[k][j])
    
    for i in range(V):
        for option in model[i]:
            h_trans[i][option[1]].append(option[0])

    f = open("heuristic.txt", "w")
    f.write(str(heuristic))
    f.close()

    f = open("h_trans.txt", "w")
    f.write(str(h_trans))
    f.close()
  
class SearchProblem:

    def __init__(self, goal, model, auxheur = []):
        self._goal = goal
        self._model = model
        self._auxheur = auxheur
        global heuristic
        if heuristic == []:
            floyd_warshall(model)

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


def limit_depth_reached():
    print("LIMIT_DEPTH_REACHED")

def limit_expanssion_reached():
    print("LIMIT_EXPANSION_REACHED")


#
# Exercise 1 - One agent, no tickets limit
#
def A_star(init, goal, limitexp, limitdepth, final):
    required_depth = heuristic[init][goal]
    if required_depth == 1:
        final.append([[ h_trans[init][goal][0] ], [goal]])
        return

    print("init: {} || goal: {}".format(init, goal))

    for new_step in range(len(heuristic)):
        if heuristic[init][new_step] == 1 and heuristic[new_step][goal] == required_depth - 1:
            final.append([[ h_trans[init][new_step][0] ], [new_step]])
            A_star(new_step, goal, limitexp, limitdepth, final)
            return


def search_1agent_nolim(self, init, limitexp, limitdepth):
    #print("Map: {}".format(self._model))

    if heuristic[init[0]][self._goal[0]] > limitdepth:
        return []

    final = [ [[], init] ]
    A_star(init[0], self._goal[0], limitexp, limitdepth, final)

    print("final1: {}".format(final))

    return final


#
# Exercise 2 - One agent, tickets limitted
#
def has_ticket(tickets, type):
    return tickets[type] > 0

def A_star_limitted(init, goal, tickets, limitexp, max_depth):
    required_depth = max_depth
    if required_depth == 1:
        for transport in h_trans[init][goal]:
            if has_ticket(tickets, transport):
                return [ [[ h_trans[init][goal][0] ], [goal]] ]
        return []

    for new_step in range(len(heuristic)):
        if heuristic[init][new_step] == 1 and heuristic[new_step][goal] <= required_depth - 1:
            for transport in h_trans[init][new_step]:
                if has_ticket(tickets, transport):
                    new_tickets = tickets.copy()
                    new_tickets[transport] += -1
                    path = A_star_limitted(new_step, goal, new_tickets, limitexp, required_depth-1)

                    if path != []:
                        return [ [[ transport ], [new_step]] ] + path
    return []


def search_1agent_lim(self, init, tickets, limitexp, limitdepth):
    initial = init [0]
    goal = self._goal[0]
    
    final = [[[], init]]
    path = []

    for max_depth in range(heuristic[initial][goal], limitdepth+1):
        path = A_star_limitted(init[0], self._goal[0], tickets, limitexp, max_depth)
        if path != []:
            break

    if path == []
        return []

    final += path
    print("final2: {}".format(final))
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

    exp = 0
    while queue and not done:
        currTrans3 = transport.pop(0)
        currPath3 = queue.pop(0)
        currVertexes3 = currPath3[-1]

        if len(currTrans3)-1 == limitdepth:
            limit_depth_reached()
        if exp == limitexp:
            limit_expanssion_reached()

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

        if combinations != []:
            exp += 1

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

    exp = 0
    while queue and not done:
        currTickets = tickets.pop(0)
        currTrans3 = transport.pop(0)
        currPath3 = queue.pop(0)
        currVertexes3 = currPath3[-1]

        if len(currTrans3)-1 == limitdepth:
            limit_depth_reached()
        if exp == limitexp:
            limit_expanssion_reached()

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

        if combinations != []:
            exp += 1

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

    exp = 0
    while queue and not done:
        currTickets = tickets.pop(0)
        currTrans3 = transport.pop(0)
        currPath3 = queue.pop(0)
        currVertexes3 = currPath3[-1]

        if len(currTrans3)-1 == limitdepth:
            limit_depth_reached()
        if exp == limitexp:
            limit_expanssion_reached()

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

        if combinations != []:
            exp += 1

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
