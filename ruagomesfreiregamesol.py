# 86752 Afonso Ribeiro - 89469 Joana Alvoeiro

import math
import pickle
import time

heuristic = []
h_trans = []

def floyd_warshall(model):

    V = len(model)

    no_transports_model = [ [option[1] for option in dest] for dest in model ]

    global h_trans
    h_trans = [ [[] for j in range(V) ] for i in range(V) ]
    
    for i in range(V):
        for option in model[i]:
            h_trans[i][option[1]].append(option[0])

    global heuristic
    heuristic = [ [1 if j in no_transports_model[i] else math.inf for j in range(V) ] for i in range(V) ]

    for k in range(V):
        for i in range(V):
            for j in range(V):
                heuristic[i][j] = min(heuristic[i][j], heuristic[i][k] + heuristic[k][j])

  
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
            return search_3agent_lim(self, self._goal, init, tickets, limitexp, limitdepth)
        else:
            return search_3agent_lim_anyorder(self, init, tickets, limitexp, limitdepth)


def limit_depth_reached():
    print("LIMIT_DEPTH_REACHED")

def limit_expanssion_reached():
    print("LIMIT_EXPANSION_REACHED")

exp = 0
#
# Exercise 1 - One agent, no tickets limit
#
def A_star(model, init, goal, limitdepth, final):
    global exp
    exp += 1
    required_depth = heuristic[init][goal]
    if required_depth == 1:
        final.append([[ h_trans[init][goal][0] ], [goal]])
        return

    for option in model[init]:
        new_step = option[1]
        if heuristic[new_step][goal] == required_depth - 1:
            final.append([[ option[0] ], [new_step]])
            A_star(model, new_step, goal, limitdepth, final)
            return

def search_1agent_nolim(self, init, limitexp, limitdepth):

    if heuristic[init[0]][self._goal[0]] > limitdepth:
        print("Depth limit exceeded: {}".format(heuristic[init[0]][self._goal[0]]))
        return []

    global exp
    exp = 0

    final = [ [[], init] ]
    A_star(self._model, init[0], self._goal[0], limitdepth, final)

    if exp > limitexp:
        print("Expansion limit exceeded: {}".format(exp))
        final = []

    exp = 0

    #print("final1: {}".format(final))

    return final


#
# Exercise 2 - One agent, tickets limited
#
def has_ticket(tickets, *types):
    cost = [0,0,0]
    for typ in types:
        cost[typ] += 1
    return tickets[0] >= cost[0] and tickets[1] >= cost[1] and tickets[2] >= cost[2]


def IDA_star(model, init, goal, tickets, max_depth):
    global exp
    exp += 1
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
            path = IDA_star(model, new_step, goal, new_tickets, required_depth-1)

            if path != []:
                return [ [[ transport ], [new_step]] ] + path

    return []

def search_1agent_lim(self, init, tickets, limitexp, limitdepth):
    initial = init [0]
    goal = self._goal[0]
    
    final = [[[], init]]
    path = []

    global exp
    exp = 0
    for max_depth in range(heuristic[initial][goal], limitdepth+1):
        path = IDA_star(self._model, init[0], self._goal[0], tickets, max_depth)
        if path != []:
            break

    final += path

    if path == []:
        final = []

    if exp > limitexp:
        print("Expansion limit exceeded: {}".format(exp))
        final = []

    exp = 0
    
    #print("final2: {}".format(final))
    
    return final


#
# Exercise 3 - Three agents, no ticket limit
#
def IDA_star_3_no_lim(model, init, goal, depth):
    global exp
    exp += 1
    required_depth = depth
    if required_depth == 1:
        trans = [h_trans[init[0]][goal[0]][0], h_trans[init[1]][goal[1]][0], h_trans[init[2]][goal[2]][0]]
        return [[trans, goal]]

    paths = []


    for option0 in model[init[0]]:
        new_step0 = option0[1]
        trans0 = option0[0]

        if heuristic[new_step0][goal[0]] > required_depth - 1:
            continue

        for option1 in model[init[1]]:
            new_step1 = option1[1]
            trans1 = option1[0]

            if heuristic[new_step1][goal[1]] > required_depth - 1 or new_step0 == new_step1:
                continue

            for option2 in model[init[2]]:
                new_step2 = option2[1]
                trans2 = option2[0]

                if heuristic[new_step2][goal[2]] <= required_depth - 1 and new_step0 != new_step2 != new_step1:
                    new_init = [new_step0, new_step1,new_step2]
                    trans = [trans0, trans1, trans2]
                    paths = IDA_star_3_no_lim(model, new_init, goal, required_depth - 1)
                    if paths != []:
                        return [[trans, new_init]] + paths

    return []
            
def search_3agent_nolim(self, init, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal
    model = self._model

    global exp
    exp = 0

    worst_depth = max( heuristic[init[agent]][goals[agent]] for agent in range(3) )

    paths = []
    while paths == [] and worst_depth <= limitdepth:
        paths = IDA_star_3_no_lim(myMap, init, goals, worst_depth)
        worst_depth += 1

    final = []
    if paths != []:
        final = [[[], init]] + paths

    if worst_depth > limitdepth:
        print("Depth limit exceeded: {}".format(exp))
        final = []


    if exp > limitexp:
        print("Expansion limit exceeded: {}".format(exp))
        final = []

    exp = 0

    print("final3: {}".format(final))

    return final

#
# Exercise 4 - Three agents, tickets limited
#
def IDA_star_3_lim(model, init, goal, tickets, depth):
    global exp
    exp += 1
    required_depth = depth
    if required_depth == 1:
        trans = [h_trans[init[0]][goal[0]][0], h_trans[init[1]][goal[1]][0], h_trans[init[2]][goal[2]][0]]
        if not has_ticket(tickets, trans[0], trans[1], trans[2]):
            return []
        return [[trans, goal]]

    paths = []

    for option0 in model[init[0]]:
        new_step0 = option0[1]
        trans0 = option0[0]

        if heuristic[new_step0][goal[0]] > required_depth - 1 or not has_ticket(tickets, trans0):
            continue

        for option1 in model[init[1]]:
            new_step1 = option1[1]
            trans1 = option1[0]

            if heuristic[new_step1][goal[1]] > required_depth - 1 or new_step0 == new_step1 or not has_ticket(tickets, trans0, trans1):
                continue

            for option2 in model[init[2]]:
                new_step2 = option2[1]
                trans2 = option2[0]

                if heuristic[new_step2][goal[2]] <= required_depth - 1 and new_step0 != new_step2 != new_step1 \
                and has_ticket(tickets, trans0, trans1, trans2):
                    new_init = [new_step0, new_step1,new_step2]
                    trans = [trans0, trans1, trans2]
                    new_tickets = tickets.copy()
                    new_tickets[trans0] += -1
                    new_tickets[trans1] += -1
                    new_tickets[trans2] += -1

                    paths = IDA_star_3_lim(model, new_init, goal, new_tickets, required_depth - 1)
                    if paths != []:
                        return [[trans, new_init]] + paths

    return []

def search_3agent_lim(self, goals, init, tickets, limitexp, limitdepth):
    myMap = self._model
    model = self._model

    global exp
    exp = 0

    worst_depth = max( heuristic[init[agent]][goals[agent]] for agent in range(3) )

    paths = []
    while paths == [] and worst_depth <= limitdepth:
        paths = IDA_star_3_lim(myMap, init, goals, tickets, worst_depth)
        worst_depth += 1

    final = []
    if paths != []:
        final = [[[], init]] + paths

    if worst_depth > limitdepth:
        print("Depth limit exceeded: {}".format(exp))
        final = []


    if exp > limitexp:
        print("Expansion limit exceeded: {}".format(exp))
        final = []

    print("exp: {}".format(exp))

    exp = 0

    #print("final4: {}".format(final))

    return final

#
# Exercise 5 - Three agents, tickets limited
#
def search_3agent_lim_anyorder(self, init, tickets, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal
    model = self._model


    global exp
    exp = 0

    possible_goals = [ [a, b, c] for a in goals for b in goals for c in goals if a != b != c != a ]
    
    worst_depths = [ max( heuristic[init[agent]][goal[agent]] for agent in range(3) ) for goal in possible_goals ]

    worst_dict = {}
    for i in range(1, limitdepth+1):
        worst_dict[i] = []

    for i in range(len(possible_goals)):
        worst_dict[worst_depths[i]] += [possible_goals[i]]

    paths = []
    for i in range(1, limitdepth+1):
        for goal in worst_dict[i]:
            paths = IDA_star_3_lim(myMap, init, goal, tickets, i)
            if paths != []:
                break
        if paths != []:
                break

    final = []
    if paths != []:
        final = [[[], init]] + paths

    if worst_depths[0] > limitdepth:
        print("Depth limit exceeded: {}".format(exp))
        final = []


    if exp > limitexp:
        print("Expansion limit exceeded: {}".format(exp))
        final = []

    print("exp: {}".format(exp))

    exp = 0

    #print("final5: {}".format(final))

    return final
