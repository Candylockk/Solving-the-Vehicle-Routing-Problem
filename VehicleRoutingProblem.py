import numpy as np

## Define helper classes
class Order:
    def __init__(self, order): 
        self.x = order[0]
        self.y = order[1]

    def getDistance(self, order): 
        distance = np.sqrt(((self.x - order.x) ** 2) + ((self.y - order.y) ** 2))
        return distance

class Route:
    def __init__(self, route):
        self.route = route
        self.distance = 0

    def routeDistance(self):
        if self.distance == 0:
            pathDistance = 0
            for i in range(0, len(self.route)):
                previous_order = self.route[i]
                next_order = None
                if i + 1 < len(self.route):
                    next_order = self.route[i + 1]
                else:
                    next_order = self.route[0] 
                pathDistance += orderDictionary[previous_order].getDistance(orderDictionary[next_order])
            self.distance = pathDistance
        return self.distance

## Define helper functions
def listToDict(lst):
    d = {}
    for item in lst:
        d[item[0]] = Order([float(item[1]), float(item[2])]) 
    return d

def getTheta(orders):
    d = {}
    for item in orders:
        d[item[0]] = np.arctan2(float(item[1]), float(item[2]))
    return d

def eliminateCrossing(route, orderDictionary):
    improvement = True
    while improvement:
        improvement = False
        for i in range(len(route)):
            for j in range(i + 2, len(route) - 1):
                alt_path_1 = orderDictionary[route[i]].getDistance(orderDictionary[route[j]])
                alt_path_2 = orderDictionary[route[i + 1]].getDistance(orderDictionary[route[j + 1]])
                alternative_path = alt_path_1 + alt_path_2
                origin_path = orderDictionary[route[i]].getDistance(orderDictionary[route[i+1]])+ orderDictionary[route[j]].getDistance(orderDictionary[route[j+1]])
                if alternative_path < origin_path:
                    route[i + 1:j + 1] = route[i + 1:j + 1][::-1]
                    improvement = True
    return route

def getfarthestRoute(vehiclePaths):
    d = {}
    for i in range(0, len(vehiclePaths)):
        d[i] = Route(vehiclePaths[i]).routeDistance()
    return sorted(d.items(), key = lambda x: x[1], reverse=True)[0][0]

def optimizeRoute(vehiclePaths, numberOfvehicles):
    if numberOfvehicles == 1:
        return vehiclePaths
    else:
        farthestDistArr = []
        while not ((len(farthestDistArr) > 5) and farthestDistArr[-5:] == farthestDistArr[-1:-6:-1]):
            leftvehicle = None
            rightvehicle = None
            farthestvehicleRoute = getfarthestRoute(vehiclePaths)
            for i in range(1, len(vehiclePaths[farthestvehicleRoute])):
                if i >= len(vehiclePaths[farthestvehicleRoute]) - 2:
                    break
                if farthestvehicleRoute == len(vehiclePaths) - 1:
                    leftvehicle = farthestvehicleRoute - 1
                    rightvehicle = 0
                elif farthestvehicleRoute == 0:
                    leftvehicle = len(vehiclePaths) - 1
                    rightvehicle = farthestvehicleRoute + 1
                else:
                    leftvehicle = farthestvehicleRoute - 1
                    rightvehicle = farthestvehicleRoute + 1

                farthestRoute = vehiclePaths[farthestvehicleRoute][:]
                leftRoute = vehiclePaths[leftvehicle][:]
                rightRoute = vehiclePaths[rightvehicle][:]
                farthestRoute_dist = Route(farthestRoute).routeDistance()

                bad_order = farthestRoute.pop(i)
                leftRoute.insert(-1, bad_order)
                rightRoute.insert(-1, bad_order)
                farthestRoute_new = eliminateCrossing(farthestRoute, orderDictionary)
                leftRoute_new = eliminateCrossing(leftRoute, orderDictionary)
                rightRoute_new = eliminateCrossing(rightRoute, orderDictionary)
                farthestRoute_new_dist = Route(farthestRoute_new).routeDistance()
                leftRoute_new_dist = Route(leftRoute_new).routeDistance()
                rightRoute_new_dist = Route(rightRoute_new).routeDistance()

                if (farthestRoute_new_dist < farthestRoute_dist) and (min(leftRoute_new_dist, rightRoute_new_dist) < farthestRoute_new_dist):
                    if leftRoute_new_dist < rightRoute_new_dist:
                        vehiclePaths[farthestvehicleRoute] = farthestRoute_new
                        vehiclePaths[leftvehicle] = leftRoute_new
                    else:
                        vehiclePaths[farthestvehicleRoute] = farthestRoute_new
                        vehiclePaths[rightvehicle] = rightRoute_new
            farthestDistArr.append(farthestRoute_new_dist)

        vehiclePaths = [i for i in vehiclePaths if len(i) != 0]
        return vehiclePaths


## Begin code
def solveVRP(orders, numberOfVehicles):
    global orderDictionary
    vehiclePaths = [[] for _ in range(numberOfVehicles)]
    orderDictionary = listToDict(orders)
    theta_orders = getTheta(orders)
    for order in orders: 
        index = int(np.floor(theta_orders[order[0]] / (2 * np.pi / numberOfVehicles)))
        vehiclePaths[index].append(order[0])
    for route in vehiclePaths:
        eliminateCrossing(route, orderDictionary)
    return optimizeRoute(vehiclePaths, numberOfVehicles)
