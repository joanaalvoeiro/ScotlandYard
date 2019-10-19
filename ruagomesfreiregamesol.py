import math
import pickle
import time

  
class SearchProblem:

  def __init__(self, goal, model, auxheur = []):
    self._goal = goal
    self._model = model
    self._auxheur = auxheur
    pass

  def search(self, init, limitexp = 2000, limitdepth = 10, tickets = [math.inf,math.inf,math.inf]):
    if len(self._goal) == 1 and tickets == [math.inf,math.inf,math.inf]:
      final = search_1agent_nolim(init, self)

    elif len(self._goal) == 1:
      final = search_1agent_lim(init, tickets, self)

    else:
      return []

    return final

def has_ticket(tickets, type):
  return tickets[type] > 0

def search_1agent_nolim(init, self):
    myMap = self._model
    goal = self._goal[0]
    mapSize = len(myMap)
    visited = set()
    queue = []
    transport = [[]]
    done = False

    print('goal is ' + str(goal))

    queue = [[init[0]]]

    while queue and not done:
      currPath = queue.pop(0)
      currTrans = transport.pop(0)
      currVertex = currPath[-1]
      print('visiting ' + str(currVertex))

      if currVertex not in visited:
        for i in myMap[currVertex]:
          if i[1] == goal:
            done = True
            visited.add(i[1])

            newPath = list(currPath)
            newPath.append(i[1])

            newTrans = list(currTrans)
            newTrans.append(i[0])
            print('done w path ' + str(newPath))
            print('transport: ' + str(newTrans))
            break

          newPath = list(currPath)
          newPath.append(i[1])
          queue.append(newPath)

          newTrans = list(currTrans)
          newTrans.append(i[0])
          transport.append(newTrans)

        visited.add(currVertex)
    
    path = newPath[1:]
    pathLen = len(path)

    final = [[[], init]]

    i = 0

    while(i<pathLen):
      final.append([[newTrans[i]], [path[i]]])
      i += 1

    print(str(final))

    return final


def search_1agent_lim(init, tickets, self):
    myMap = self._model
    goal = self._goal[0]
    mapSize = len(myMap)
    myTickets = [tickets.copy()]

    #print(str(myMap))

    visited = set()
    queue = []
    transport = [[]]
    done = False

    print('goal is ' + str(goal))
    print('available tickets: ' + str(tickets))

    queue = [[init[0]]]

    while queue and not done:
      currPath = queue.pop(0)
      currTrans = transport.pop(0)
      currTickets = myTickets.pop(0)
      currVertex = currPath[-1]
      print('visiting ' + str(currVertex))

      if currVertex not in visited:
        for i in myMap[currVertex]: #para cada par transporte/destino do vertice atual
          if has_ticket(currTickets, i[0]):#se temos bilhetes desse transporte
            if i[1] == goal:#se o destino e o goal
              done = True
              visited.add(i[1])

              newPath = list(currPath)
              newPath.append(i[1])
              queue.append(newPath)

              newTrans = list(currTrans)
              newTrans.append(i[0])
              transport.append(newTrans)

              newTickets = currTickets.copy()
              newTickets[i[0]] -=1

              print('done w path ' + str(newPath))
              print('transport: ' + str(newTrans))
              break

            newPath = list(currPath)
            newPath.append(i[1])
            queue.append(newPath)

            newTrans = list(currTrans)
            newTrans.append(i[0])
            transport.append(newTrans)

            newTickets = currTickets.copy()
            newTickets[i[0]] -=1
            myTickets.append(newTickets)


        visited.add(currVertex)
    
    path = newPath[1:]
    pathLen = len(path)

    final = [[[], init]]

    i = 0

    while(i<pathLen):
      final.append([[newTrans[i]], [path[i]]])
      i += 1

    print(str(final))

    return final