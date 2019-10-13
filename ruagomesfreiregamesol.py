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



    
