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

        elif tickets == [math.inf,math.inf,math.inf]:
            final = search_3agent_nolim(init, self)

        else:
            final = []

        return final

def has_ticket(tickets, type):
  return tickets[type] > 0

def search_1agent_nolim(init, self):
    myMap = self._model
    goal = self._goal[0]
    mapSize = len(myMap)
    visited = set()
    transport = [[]]
    done = False

    #print('goal is ' + str(goal))

    queue = [[init[0]]]

    while queue and not done:
      currPath = queue.pop(0)
      currTrans = transport.pop(0)
      currVertex = currPath[-1]
      #print('visiting ' + str(currVertex))

      if currVertex not in visited:
        for i in myMap[currVertex]:
          if i[1] == goal:
            done = True
            visited.add(i[1])

            newPath = list(currPath)
            newPath.append(i[1])

            newTrans = list(currTrans)
            newTrans.append(i[0])
            #print('done w path ' + str(newPath))
            #print('transport: ' + str(newTrans))
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

    #print(str(final))

    return final


def search_1agent_lim(init, tickets, self):
    myMap = self._model
    goal = self._goal[0]
    mapSize = len(myMap)
    myTickets = [tickets.copy()]

    print(str(myMap))

    visited = set()
    queue = []
    transport = [[]]
    done = False

    #print('goal is ' + str(goal))
    #print('available tickets: ' + str(tickets))

    queue = [[init[0]]]

    while queue and not done:
      currPath = queue.pop(0)
      currTrans = transport.pop(0)
      currTickets = myTickets.pop(0)
      currVertex = currPath[-1]
      #print('visiting ' + str(currVertex))

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

              #print('done w path ' + str(newPath))
              #print('transport: ' + str(newTrans))
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

    while ( i < pathLen ):
      final.append([[newTrans[i]], [path[i]]])
      i += 1

    #print(str(final))

    return final

def search_3agent_nolim(init, self):
    myMap = self._model
    goals = self._goal
    mapSize = len(myMap)
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
                print(currVertex)
                for i in myMap[currVertex]:


                    if i[1] == goals[agent]:
                        done[agent] = True
                        visited[agent].add(i[1])

                        newPath = list(currPath)
                        newPath.append(i[1])

                        newTrans = list(currTrans)
                        newTrans.append(i[0])

                        finals[agent] = newPath

                        print('done agent ' + str(agent) + ' with path ' + str(newPath))
                        #print('done w path ' + str(newPath))
                        #print('transport: ' + str(newTrans))
                        break

                    newPath = list(currPath)
                    newPath.append(i[1])
                    print(newPath)
                    queues[agent].append(newPath)

                    newTrans = list(currTrans)
                    newTrans.append(i[0])
                    transport[agent].append(newTrans)

                    visited[agent].add(currVertex)

        

    print(finals)

    return ['final']
