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
    heuristic = [ [1 if j in no_transports_model[i] else math.inf for j in range(V) ] for i in range(V) ]


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
def A_star(model, init, goal, limitexp, limitdepth, final):
    required_depth = heuristic[init][goal]
    if required_depth == 1:
        final.append([[ h_trans[init][goal][0] ], [goal]])
        return

    for option in model[init]:
        new_step = option[1]
        if heuristic[new_step][goal] == required_depth - 1:
            final.append([[ option[0] ], [new_step]])
            A_star(model, new_step, goal, limitexp, limitdepth, final)
            return


def search_1agent_nolim(self, init, limitexp, limitdepth):

    if heuristic[init[0]][self._goal[0]] > limitdepth:
        return []

    final = [ [[], init] ]
    A_star(self._model, init[0], self._goal[0], limitexp, limitdepth, final)

    print("final1: {}".format(final))

    return final


#
# Exercise 2 - One agent, tickets limitted
#
def has_ticket(tickets, type):
    return tickets[type] > 0

def IDA_star(model, init, goal, tickets, limitexp, max_depth):
    required_depth = max_depth
    if required_depth == 1:
        for transport in h_trans[init][goal]:
            if has_ticket(tickets, transport):
                return [ [[ h_trans[init][goal][0] ], [goal]] ]
        return []

    for option in model[init]:
        transport = option[0]
        new_step = option[1]
        if heuristic[new_step][goal] <= required_depth - 1 and has_ticket(tickets, transport):
            new_tickets = tickets.copy()
            new_tickets[transport] += -1
            path = IDA_star(model, new_step, goal, new_tickets, limitexp, required_depth-1)

            if path != []:
                return [ [[ transport ], [new_step]] ] + path

    return []


def search_1agent_lim(self, init, tickets, limitexp, limitdepth):
    initial = init [0]
    goal = self._goal[0]
    
    final = [[[], init]]
    path = []

    for max_depth in range(heuristic[initial][goal], limitdepth+1):
        path = IDA_star(self._model, init[0], self._goal[0], tickets, limitexp, max_depth)
        if path != []:
            break

    if path == []:
        return []

    final += path
    print("final2: {}".format(final))
    return final

'''
class Node():

    def __init__(self, parent=None, vertex=None, g, h):
        self.parent = parent
        self.vertex = vertex

        self.g = 0
        self.f = g + h

    def __eq__(self, other):
        return self.vertex == other.vertex


def A_star_limitted(my_map, init, goal, tickets, limitexp, limitdepth):
    start_node = Node(None, init)
    end_node = Node(None, goal)

    queue = []

    queue.append(start_node)

    while queue != []:
        current_node = queue[0]


def search_1agent_lim(self, init, tickets, limitexp, limitdepth):
    initial = init [0]
    goal = self._goal[0]
    
    final = [[[], init]]

    path = A_star_limitted(self._model, initial, goal, tickets, limitexp, limitdepth)

    if path == []:
        return []

    final += path
    print("final2: {}".format(final))
    return final
'''
#
# Exercise 3 - Three agents, no tickets limit
#
def A_star_3(model, prev_path, init, goal, limitexp, depth):
    required_depth = depth
    if required_depth == 1:
        return [prev_path + [goal]]

    paths = []
    for option in model[init]:
        new_step = option[1]
        if heuristic[new_step][goal] <= required_depth - 1:
            paths += A_star_3(model, prev_path + [new_step], new_step, goal, limitexp, required_depth - 1)


    print("sub_path: {}".format(paths))

    return paths

def valid_combination(Paths, valid_path):
    valid = True
    for path0 in Paths[0]:
        for path1 in Paths[1]:
            for path2 in Paths[2]:
                valid = True
                for i in range(len(path0)):
                    if path0[i] == path1[i] or path0[i] == path2[i] or path1[i] == path2[i]:
                        valid = False
                        break
                if valid:
                    valid_path[0] = [path0, path1, path2]
                    return True
    return False

            

def search_3agent_nolim(self, init, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal
    model = self._model

    worst_depth = max( heuristic[init[agent]][goals[agent]] for agent in range(3) )

    Paths = {}
    for agent in range(3):
        Paths[agent] = A_star_3(myMap, [init[agent]], init[agent], goals[agent], limitexp, worst_depth)

    valid_path = {}
    while any( [ p == [] for p in Paths.values() ] ) or not valid_combination(Paths, valid_path):
        worst_depth += 1
        for agent in range(3):
            Paths[agent] = A_star_3(myMap, [init[agent]], init[agent], goals[agent], limitexp, worst_depth)

        print("final3: {}".format(Paths))


    print("final3: {}".format(valid_path))

    return []#final

'''
    worst_depth = heuristic[ init[0] ][ goals[0] ]
    worst_agent = 0

    if worst_depth < heuristic[ init[1] ][ goals[1] ]:
        worst_depth = heuristic[ init[1] ][ goals[1] ]
        worst_agent = 1

    if worst_depth < heuristic[ init[2] ][ goals[2] ]:
        worst_depth = heuristic[ init[2] ][ goals[2] ]
        worst_agent = 2
'''

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
