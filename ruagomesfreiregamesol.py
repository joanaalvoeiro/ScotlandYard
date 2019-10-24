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

    #print("final1: {}".format(final))

    return final


#
# Exercise 2 - One agent, tickets limited
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
    
    #print("final2: {}".format(final))
    
    return final


#
# Exercise 3 - Three agents, no ticket limit
#
def translate_path(path0, path1, path2):
    final = [ [[], [path0[0], path1[0], path2[0]]] ]
    for i in range(1, len( path0)):
        v0, prev0 = path0[i], path0[i-1]
        v1, prev1 = path1[i], path1[i-1]
        v2, prev2 = path2[i], path2[i-1]
        final.append( [ [h_trans[prev0][v0][0], h_trans[prev1][v1][0], h_trans[prev2][v2][0] ], [v0, v1, v2] ] )
    return final

def valid_combination_3_no_lim(Paths, valid_path):
    valid = True
    for path0 in Paths[0]:
        for path1 in Paths[1]:
            for path2 in Paths[2]:
                valid = True
                for i in range(1, len(path0)):
                    if path0[i] == path1[i] or path0[i] == path2[i] or path1[i] == path2[i]:
                        valid = False
                        break
                if valid:
                    valid_path[0] = [path0, path1, path2]
                    return True
    return False

def IDA_star_3_no_lim(model, prev_path, init, goal, limitexp, depth):
    required_depth = depth
    if required_depth == 1:
        return [prev_path + [goal]]

    paths = []
    for option in model[init]:
        new_step = option[1]
        if heuristic[new_step][goal] <= required_depth - 1:
            paths += IDA_star_3_no_lim(model, prev_path + [new_step], new_step, goal, limitexp, required_depth - 1)

    return paths
            
def search_3agent_nolim(self, init, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal
    model = self._model

    worst_depth = max( heuristic[init[agent]][goals[agent]] for agent in range(3) )

    Paths = {}
    for agent in range(3):
        Paths[agent] = IDA_star_3_no_lim(myMap, [init[agent]], init[agent], goals[agent], limitexp, worst_depth)

    valid_path = {}
    while any( [ p == [] for p in Paths.values() ] ) or not valid_combination_3_no_lim(Paths, valid_path):
        worst_depth += 1
        for agent in range(3):
            Paths[agent] = IDA_star_3_no_lim(myMap, [init[agent]], init[agent], goals[agent], limitexp, worst_depth)

    
    final = translate_path(valid_path[0][0], valid_path[0][1], valid_path[0][2])

    #print("final3: {}".format(final))

    return final

#
# Exercise 4 - Three agents, tickets limited
#
def translate_path_lim(path0, path1, path2):
    final = [[[], [path0[0][1], path1[0][1], path2[0][1]] ]]
    for i in range(1, len(path0)):
        final.append( [ [path0[i][0], path1[i][0], path2[i][0]], [path0[i][1], path1[i][1], path2[i][1]] ] )
    return final

def limit_tickets_reached(available_tickets, tickets0, tickets1, tickets2):
    for i in range(len(available_tickets)):
        if available_tickets[i] < available_tickets[i]*3 - tickets0[i] - tickets1[i] - tickets2[i]:
            return True

    return False

def valid_combination_3_lim(Paths, available_tickets, valid_path):
    for path0 in Paths[0]:
        for path1 in Paths[1]:
            for path2 in Paths[2]:
                valid = True
                if limit_tickets_reached(available_tickets, path0[-1], path1[-1], path2[-1]):
                    continue

                for i in range(1, len(path0)-1):
                    if path0[i][1] == path1[i][1] or path0[i][1] == path2[i][1] or path1[i][1] == path2[i][1]:
                        valid = False
                        break
                if valid:
                    valid_path[0] = [path0[:-1], path1[:-1], path2[:-1]]
                    return True
    return False

def IDA_star_3_lim(model, prev_path, init, goal, tickets, limitexp, depth):
    required_depth = depth
    if required_depth == 1:
        possible_finish = []
        for transport in h_trans[init][goal]:
            if has_ticket(tickets, transport):
                new_tickets = tickets.copy()
                new_tickets[transport] += -1
                possible_finish.append( prev_path + [ [ transport , goal] ] + [new_tickets])
        return possible_finish

    paths = []
    for option in model[init]:
        transport = option[0]
        new_step = option[1]
        if heuristic[new_step][goal] <= required_depth - 1 and has_ticket(tickets, transport):
            new_tickets = tickets.copy()
            new_tickets[transport] += -1
            paths += IDA_star_3_lim(model, prev_path + [ [transport, new_step] ], new_step, goal, new_tickets, limitexp, required_depth - 1)

    return paths

def search_3agent_lim(self, goals, init, tickets, limitexp, limitdepth):
    myMap = self._model
    model = self._model

    worst_depth = max( heuristic[init[agent]][goals[agent]] for agent in range(3) )

    Paths = {}
    for agent in range(3):
        Paths[agent] = IDA_star_3_lim(myMap, [[[],init[agent]]], init[agent], goals[agent], tickets, limitexp, worst_depth)

    valid_path = {}
    while any( [ p == [] for p in Paths.values() ] ) or not valid_combination_3_lim(Paths, tickets, valid_path):
        worst_depth += 1
        for agent in range(3):
            Paths[agent] = IDA_star_3_lim(myMap, [[[],init[agent]]], init[agent], goals[agent], tickets, limitexp, worst_depth)

    
    final = translate_path_lim(valid_path[0][0], valid_path[0][1], valid_path[0][2])

    #print("final4: {}".format(final))

    return final

#
# Exercise 5 - Three agents, tickets limited
#


def search_3agent_lim_anyorder(self, init, tickets, limitexp, limitdepth):
    myMap = self._model
    goals = self._goal
    model = self._model

    possible_goals = [ [a, b, c] for a in goals for b in goals for c in goals if a != b != c != a ]

    all_final = [ search_3agent_lim(self, goals, init, tickets, limitexp, limitdepth) for goals in possible_goals ]

    all_final = list(filter(lambda f : f != [], all_final))

    if all_final == []:
        return []

    final = all_final[0]
    for i in range(1, len(all_final)):
        if len(final) > len(all_final[i]):
            final = all_final[i]

    #print("final5: {}".format(final))

    return final