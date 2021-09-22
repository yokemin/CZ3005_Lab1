import json
from queue import PriorityQueue
from math import sqrt
import time

class Graph:

    # Load G.json file
    with open('G.json') as json_file:
        graph = json.load(json_file)

    # Load Dist.json file
    with open('Dist.json') as json_file:
        dists = json.load(json_file)

    # Load Cost.json file
    with open('Cost.json') as json_file:
        costs = json.load(json_file)

    # Load Coord.json file
    with open('Coord.json') as json_file:
        coords = json.load(json_file)


    # Task 1: Using UCS to solve relaxed version of the NYC instance
    def UCS(self, source, target):

        # Start time
        tic = time.perf_counter()

        # Create a priority queue for UCS
        queue = PriorityQueue()

        # Create a dict to store key:value = node:prev_node
        previous_nodes = {}

        # Put source into previous_nodes
        previous_nodes[source] = source

        # Enqueue the source node's neighbours with their dist
        for neighbour in self.graph[source]:
            if neighbour not in previous_nodes.keys():
                edge = source + "," + neighbour
                edge_dist = self.dists[edge]
                queue.put((edge_dist, (neighbour, source)))

        # UCS
        while queue:

            # Choose a node with the smallest dist:
            dist, (current, previous) = queue.get()

            if current not in previous_nodes.keys():
                # Add current node and its previous node into previous_nodes
                previous_nodes[current] = previous

                if current == target:
                    break

                # Get all adjacent vertices of the de-queued node current. If a adjacent
                # has not been visited, add its (dist,(neighbour,current)) into queue
                for neighbour in self.graph[current]:
                    if neighbour not in previous_nodes.keys():
                        edge = current + "," + neighbour
                        edge_dist = self.dists[edge]
                        total_dist = dist + edge_dist
                        queue.put((total_dist, (neighbour, current)))

        # End time
        toc = time.perf_counter()

        # Print time taken
        print(f"Completed search in: {toc - tic:0.4f} seconds")

        self.print_path(source, target, previous_nodes)

    # Task 2: Using UCS to solve the NYC instance, with the energy budget set to be 287932
    def modified_UCS(self, source, target):

        # Start time
        tic = time.perf_counter()

        # Create a priority queue for UCS
        queue = PriorityQueue()

        # Create a dict to store key:value = node:prev_node and another to keep track of cost accumulated
        previous_nodes = {}
        previous_nodes_with_cost_accumulated = {}

        # Put source into previous_nodes
        previous_nodes[source] = source
        previous_nodes_with_cost_accumulated[source] = [source, 0]

        # Enqueue the source node's neighbours with their dist and cost
        for neighbour in self.graph[source]:
            # Calculate the new accumulated cost with the new neighbour node added
            edge = source + "," + neighbour
            new_accumulated_cost = self.costs[edge]
            if neighbour not in previous_nodes.keys() and (new_accumulated_cost <= 287932):
                edge_dist = self.dists[edge]
                queue.put((edge_dist, (neighbour, source)))

        # UCS, taking note of energy cost
        while queue:

            # Choose a node with the smallest dist:
            dist, (current, previous) = queue.get()

            if current not in previous_nodes.keys():
                # Add current node and its previous node into previous_nodes
                previous_nodes[current] = previous

                # Add current node and its accumulated_cost into previous_nodes_with_cost_accumulated
                accumulated_cost = previous_nodes_with_cost_accumulated[previous][1]
                edge = previous + "," + current
                new_accumulated_cost = accumulated_cost + self.costs[edge]
                previous_nodes_with_cost_accumulated[current] = [previous, new_accumulated_cost]

                if current == target:
                    break

                # Get all adjacent vertices of the de-queued node. If a adjacent
                # has not been visited AND accumulated_cost with neighbour node include does not exceed 287932,
                # add its (dist,(neighbour,current)) into queue.
                for neighbour in self.graph[current]:
                    accumulated_cost = previous_nodes_with_cost_accumulated[current][1]
                    edge = current + "," + neighbour
                    new_accumulated_cost = accumulated_cost + self.costs[edge]
                    if neighbour not in previous_nodes.keys() and (new_accumulated_cost <= 287932):
                        edge_dist = self.dists[edge]
                        total_dist = dist + edge_dist
                        queue.put((total_dist, (neighbour, current)))

        # End time
        toc = time.perf_counter()

        # Print time taken
        print(f"Completed search in: {toc - tic:0.4f} seconds")

        self.print_path(source, target, previous_nodes)

    # Task 3: A* search algorithm to solve the NYC instance
    # Heuristic Function: Using Euclidean dist (Pythagoras) of current node to target node, since we are given the coords
    # E.g. Source/Current Node X(x1,y1) and Target Node B(x2,y2). Then h(n) = sqrt((x2 – x1)^2 + (y2 – y1)^2).
    # f(n) = g(n) + h(n), where g(n) is UCS
    def astar(self, source, target):

        # Start time
        tic = time.perf_counter()

        # Create a priority queue for astar
        queue = PriorityQueue()

        # Create a dict to store key:value = node:prev_node and another to keep track of cost accumulated
        previous_nodes = {}
        previous_nodes_with_cost_accumulated = {}

        # Put source into previous_nodes
        previous_nodes[source] = source
        previous_nodes_with_cost_accumulated[source] = [source, 0]

        # Enqueue the source node's neighbours with their dist and cost
        for neighbour in self.graph[source]:
            # Calculate the new accumulated cost with the new neighbour node added
            edge = source + "," + neighbour
            new_accumulated_cost = self.costs[edge]
            if neighbour not in previous_nodes.keys() and (new_accumulated_cost <= 287932):
                # Calculate heuristic function to use to determine priority in queue
                gn = self.dists[edge]
                fn = gn + self.get_heuristic_function(neighbour, target)
                queue.put((fn, (neighbour, source)))

        # Astar, while taking note of energy cost
        while queue:

            # Choose a node with the smallest fn:
            current_fn, (current, previous) = queue.get()

            if current not in previous_nodes.keys():
                # Add current node and its previous node into previous_nodes
                previous_nodes[current] = previous

                # Add current node and its accumulated_cost into previous_nodes_with_cost_accumulated
                accumulated_cost = previous_nodes_with_cost_accumulated[previous][1]
                edge = previous + "," + current
                new_accumulated_cost = accumulated_cost + self.costs[edge]
                previous_nodes_with_cost_accumulated[current] = [previous, new_accumulated_cost]

                if current == target:
                    break

                # Get all adjacent vertices of the de-queued node. If a adjacent
                # has not been visited AND accumulated_cost with neighbour node include does not exceed 287932,
                # add its (dist,(neighbour,current)) into queue.
                for neighbour in self.graph[current]:
                    accumulated_cost = previous_nodes_with_cost_accumulated[current][1]
                    edge = current + "," + neighbour
                    new_accumulated_cost = accumulated_cost + self.costs[edge]
                    if neighbour not in previous_nodes.keys() and (new_accumulated_cost <= 287932):
                        # Calculate heuristic function to use to determine priority in queue
                        gn = self.dists[edge]
                        new_fn = (current_fn - self.get_heuristic_function(current, target)) + gn + self.get_heuristic_function(neighbour, target)
                        queue.put((new_fn, (neighbour, current)))

        # End time
        toc = time.perf_counter()

        # Print time taken
        print(f"Completed search in: {toc - tic:0.4f} seconds")

        self.print_path(source, target, previous_nodes)
    # maybe heuristic funct can weigh more on energy

    def get_heuristic_function(self, neighbour, target):
        x1 = self.coords[neighbour][0]
        y1 = self.coords[neighbour][1]
        x2 = self.coords[target][0]
        y2 = self.coords[target][1]
        hn = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))
        return hn


    # Function to print shortest path found
    def print_path(self, source, target, previous_nodes):

        # Printing out the path
        if target not in previous_nodes.keys():
            print("No route found!")
        else:
            route = []
            current_node = target
            previous_node = previous_nodes[target]
            route.append(target)
            while previous_node != current_node:
                route.append(previous_node)
                current_node = previous_node
                previous_node = previous_nodes[current_node]

            # print("Route: " + str(route))

            path = str(source)
            distance = 0
            energy_cost = 0
            no_of_nodes = len(route)
            removed_node = route.pop() # remove starting node
            while len(route) != 0:
                current_node = route.pop()
                edge = removed_node + "," + current_node

                distance += self.dists[edge]
                energy_cost += self.costs[edge]
                path += "->" + current_node
                removed_node = current_node

            print("No of nodes in path: " + str(no_of_nodes) + ".")
            print("Number of nodes expanded: " + str(len(previous_nodes)))
            print("Shortest path: " + path + ".")
            print("Shortest distance: " + str(distance) + ".")
            print("Total energy cost: " + str(energy_cost) + ".")

    # Additional (Part of the approach to solving the shortest path problem)
    def BFS(self, source, target):

        # Start time
        tic = time.perf_counter()

        # Create a queue for BFS
        queue = []

        # Create a dict to store key:value = node:prev_node
        previous_nodes = {}

        # Put source into previous_nodes
        previous_nodes[source] = source

        # Enqueue the source node
        queue.append(source)

        # BFS
        while queue:

            # Dequeue a node from queue and print it
            current = queue.pop(0)

            # Get all adjacent vertices of the de-queued node current. If a adjacent
            # has not been visited, then enqueue it
            for neighbour in self.graph[current]:
                if neighbour not in previous_nodes.keys():
                    queue.append(neighbour)
                    previous_nodes[neighbour] = current

        # End time
        toc = time.perf_counter()

        # Print time taken
        print(f"Completed search in: {toc - tic:0.4f} seconds")

        self.print_path(source, target, previous_nodes)


    def modified_BFS(self, source, target):

        # Start time
        tic = time.perf_counter()

        # Create a queue for BFS
        queue = []

        # Create a dict to store key:value = node:prev_node
        previous_nodes = {}
        previous_nodes_with_cost_accumulated = {}

        # Put source into previous_nodes
        previous_nodes[source] = source
        previous_nodes_with_cost_accumulated[source] = [source, 0]

        # Enqueue the source node
        queue.append(source)

        # BFS, taking note of energy cost
        while queue:

            # Dequeue a node from queue and print it
            current = queue.pop(0)

            # Get all adjacent vertices of the de-queued node current. If a adjacent
            # has not been visited AND cost accumulated with the neighbour node included is
            # less than 287932, then enqueue it.
            for neighbour in self.graph[current]:
                accumulated_cost = previous_nodes_with_cost_accumulated[current][1]
                # Calculate the new accumulated cost with the new neighbour node added
                edge = current + "," + neighbour
                new_accumulated_cost = accumulated_cost + self.costs[edge]
                if (neighbour not in previous_nodes.keys()) and (new_accumulated_cost <= 287932):
                    queue.append(neighbour)
                    previous_nodes[neighbour] = current
                    previous_nodes_with_cost_accumulated[neighbour] = [current, new_accumulated_cost]

        # End time
        toc = time.perf_counter()

        # Print time taken
        print(f"Completed search in: {toc - tic:0.4f} seconds")

        self.print_path(source, target, previous_nodes)

# Driver code
task_no = input("------------------ Options ------------------\n1: UCS\n" +
                "2: UCS limited by energy cost\n" +
                "3: ASTAR limited by energy cost\n" +
                "(Additional) 4: BFS\n" +
                "(Additional) 5: BFS limited by energy cost\n" +
                "6: Quit\n" +
                "Enter task number: ")

g = Graph()

while (task_no != "6"):
    if task_no == "1":
        source = input("Enter starting node: ")
        target = input("Enter destination node: ")
        print("--- TASK 1 - USING UCS: ")
        g.UCS(source, target)
    elif task_no == "2":
        source = input("Enter starting node: ")
        target = input("Enter destination node: ")
        print("--- TASK 2 - USING UCS LIMITED BY ENERGY COST: ")
        g.modified_UCS(source, target)
    elif task_no == "3":
        source = input("Enter starting node: ")
        target = input("Enter destination node: ")
        print("--- TASK 3 - USING ASTAR: ")
        g.astar(source, target)
    elif task_no == "4":
        source = input("Enter starting node: ")
        target = input("Enter destination node: ")
        print("--- USING BFS: ")
        g.BFS(source, target)
    elif task_no == "5":
        source = input("Enter starting node: ")
        target = input("Enter destination node: ")
        print("--- USING BFS LIMITED BY ENERGY COST: ")
        g.modified_BFS(source, target)
    else:
        print("Invalid task number!")

    print("---------------------------------------------------------------")
    task_no = input("------------------ Options ------------------\n1: UCS\n" +
                    "2: UCS limited by energy cost\n" +
                    "3: ASTAR limited by energy cost\n" +
                    "(Additional) 4: BFS\n" +
                    "(Additional) 5: BFS limited by energy cost\n" +
                    "6: Quit\n" +
                    "Enter task number: ")


