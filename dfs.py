from pprint import pprint
import copy
from collections import deque
from random import randint, shuffle
import time
import sys

class Node:

    def __init__(self, state, space, parent, depth, step, heuristic):
        self.state = state  #The curremt configuration of tiles
        self.space = space  #the position of the blank tile
        self.parent = parent #pointer to the parent node
        self.depth = depth #number of steps taken to reach this node, depth of node in search treee
        self.step = step # direction of the last step needed to reach this node
        self.heuristic = heuristic # holds tbe value of manhattan distance to goal

    # Given the direction of movement of the empty tile, The move function returns the respective node of the successor node
    def move(self,direction): #right 0, up 1, left 2, down 3
        if direction == 0: #right
            if self.space[1] == 3:
                return "Invalid Move!"
            n_space = [self.space[0], self.space[1]+1]
        elif direction == 1: #up
            if self.space[0] == 0:
                return "Invalid Move!"
            n_space = [self.space[0]-1, self.space[1]]
        elif direction == 2: #left
            if self.space[1] == 0:
                return "Invalid Move!"
            n_space = [self.space[0], self.space[1]-1]
        elif direction == 3: #down
            if self.space[0] == 3:
                return "Invalid Move!"
            n_space = [self.space[0]+1, self.space[1]]
        n_state = copy.deepcopy(self.state)
        n_state[self.space[0]][self.space[1]] = n_state[n_space[0]][n_space[1]]
        n_state[n_space[0]][n_space[1]] = '#'
        new = Node(n_state, n_space, self, self.depth + 1, direction,0)
        new.heuristic = new.manhattan()
        return new

    # Determines whther the node is the goal state, based on the positions of the A,B,C tiles
    def is_goal_state(self):
        if (self.state[1][1] == 'A' and self.state[2][1] == 'B' and self.state[3][1] == 'C'):
            return True
        return False

    #used as the hueristic, evaluation function for the A* algorithm
    def manhattan(self):
        for i,lst in enumerate(self.state):
            for j, tile in enumerate(lst):
                if tile == 'A':
                    ai = i
                    aj = j
                    continue
                if tile == 'B':
                    bi = i
                    bj = j
                    continue
                if tile == 'C':
                    ci = i
                    cj = j
                    continue
        return (abs(ai - 1) + abs(aj - 1) + abs(bi - 2) + abs (bj - 1) + abs(ci - 3) + abs (cj - 1))

    # Go over the predeceesors of the current node to check it its state is repeated. This is needed to make searches faster and to avoid redundancy and avoid expanding unnecessary nodes
    def is_repeated(self):
        node = self
        while True:
            if node.parent == None:
                return False
            node = node.parent
            if self.state == node.state:
                return True
        pass

    #Get all the valid moves of the current tile, and returns them as list of integers
    def get_validmoves(self):
        moves = []
        if self.space[1] != 3: #right is possible
            moves.append(0)
        if self.space[0] != 0: #up is possible
            moves.append(1)
        if self.space[1] != 0: #left is possible
            moves.append(2)
        if self.space[0] != 3: #down is possible
            moves.append(3)
        return moves

    #prints the step direction as a string instead of integer
    def print_step(self):
        steps = {-1:'start', 0:'right', 1:'up', 2:'left', 3:'down'}
        pprint(steps[self.step])
        pass
    #prints the step direction as a string instead of integer, but in the file
    def print_step_file(self,f):
        steps = {-1:'start', 0:'right', 1:'up', 2:'left', 3:'down'}
        f.write(str(steps[self.step])+'\n')
        pass

    #prints the path taken from start_node to current node, with states, steps, and depth
    def print_path(self):
        node = self
        path_list = []
        path_list.append(node)
        while True:
            node = node.parent
            path_list.append(node)
            if node.parent == None:
                break
        while (len(path_list)):
            node = path_list.pop()
            node.print_step()
            pprint(node.state)
        print('Depth\t',self.depth)
        pass

    def print_path_file(self,f):
        node = self
        path_list = []
        path_list.append(node)
        while True:
            node = node.parent
            path_list.append(node)
            if node.parent == None:
                break
        while (len(path_list)):
            node = path_list.pop()
            node.print_step_file(f)
            f.write(str(node.state)+'\n')
        f.write('Depth\t'+str(self.depth)+"\n")
        pass

    # Given a node (usually goal_state), and desired depth d, this function generates a board state that is at most d steps away
    def generate_start(self,depth):
        while True:
            fails = 0
            failed = False
            node = copy.deepcopy(self)
            for n in range(0,depth):
                moves = node.get_validmoves()
                while True:
                    direction =moves[randint(0,len(moves)-1)]
                    tmp = node.move(direction)
                    if (tmp.is_repeated() == False and tmp.is_goal_state() == False): #to avoid unnecessary moves that repeats previous states or gives a goal state
                        del tmp
                        node = node.move(direction)
                        fails = 0
                        break
                    fails += 1
                    if fails == 100: #if this function gets stuck (sometimes happens), restart the function again
                        print('Resetiing')
                        failed = True
                        break
                if failed:
                    break
            if failed == False:
                break
        node.parent = None
        node.depth = 0
        node.step = -1
        return node
#performs Astar search
def Astar(start_Node):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    #____
    stats = dict()
    filename = 'smarter_astar_output.txt'
    f = open(filename,'w')
    fringe = [start_Node]
    while (len(fringe) >0): #while fringe not empty
        fringe.sort(key=lambda x: x.depth + x.heuristic, reverse = True) #sort fringe by evaluation function
        node = fringe.pop() #get the fittest node from the fringe
        if node.is_goal_state(): #if node is goal state, solution is found and print it
            solution_found = True
            node.print_path_file(f)
            break
        if node.is_repeated(): #if node has been seen before, ignore
            continue
        moves = node.get_validmoves() #find successor nodes
        nodes_expanded += 1
        if node.depth in stats:
            stats[node.depth] += 1
        else:
            stats[node.depth] = 1
        for i in moves:
            fringe.append(node.move(i))
            nodes_added += 1
    f.write('Found?\t'+str(solution_found)+'\n')
    f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
    f.write('Nodes Added\t'+str(nodes_added)+'\n')
    for k, v in stats.items():
        f.write(str(k)+' '+str(v)+'\n')
    f.close()
    return solution_found, nodes_expanded, stats, nodes_added
#performs DFS, either in a way that checks for repeated board states or not
def Depth_First(start_Node, is_smart):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    stats = dict()
    filename ='depth_output.txt'
    if is_smart:
        filename = 'smarter_'+filename
    f = open(filename,'w')
    #create fringe as a stack structure
    fringe = [start_Node]
    test = time.time()
    while (len(fringe) >0 and len(fringe) < 2500000): #while fringe not empty, limit fringe size to avoid unlimited searching
        node = fringe.pop() #get last noded added to fringe, as DFS fringe is a stack
        if node.is_goal_state(): #if node is a goal state then end search amd print
            solution_found = True
            node.print_path_file(f)
            break
        if is_smart:
            if node.is_repeated(): #avoid repated states
                continue
        if node.depth in stats:
            stats[node.depth] += 1
        else:
            stats[node.depth] = 1
        if (time.time() - test ) > 10:
            break
        moves = node.get_validmoves()
        nodes_expanded += 1
        shuffle(moves) #needed to randomize order of successors added to the fringe
        for i in moves:
            fringe.append(node.move(i))
            nodes_added += 1
    if (solution_found == False):
         print('Last Node Expanded')
         #node.print_path_file(f)
    for k, v in stats.items():
        f.write(str(k)+' '+str(v)+'\n')
    f.write('Found?\t'+str(solution_found)+'\n')
    f.write('Depth?\t'+str(node.depth)+'\n')
    f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
    f.write('Nodes Added\t'+str(nodes_added)+'\n')
    f.close()
    return solution_found, nodes_expanded, nodes_added, node.depth

#performs DLS, either in a way that checks for repeated board states or not
def Depth_Limited(start_Node, depth_limit, itr):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    stats = dict()
    if itr == False:
        filename = 'depth_limited_output.txt'
    else:
        filename = 'iterative_deepening_output.txt'
    filename = 'smarter_'+filename
    f = open(filename,'w')
    #create fringe as a stack structure
    fringe = [start_Node]
    while (len(fringe) >0 and len(fringe) < 2500000): #while fringe not empty
        node = fringe.pop()
        if node.is_goal_state():
            solution_found = True
            node.print_path_file(f)
            break
        if node.is_repeated():
            continue
        if node.depth in stats:
            stats[node.depth] += 1
        else:
            stats[node.depth] = 1
        if node.depth == depth_limit:
            continue
        moves = node.get_validmoves()
        nodes_expanded += 1
        shuffle(moves)
        for i in moves:
            fringe.append(node.move(i))
            nodes_added += 1
    if (solution_found == True):
        f.write('Found?\t'+str(solution_found)+'\n')
        f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
        f.write('Nodes Added\t'+str(nodes_added)+'\n')
        for k, v in stats.items():
            f.write(str(k)+' '+str(v)+'\n')
    f.close()
    return solution_found, nodes_expanded, stats, nodes_added, node.depth

#performs DLS in increasing depth limits
def Iterative_Deepening(start_Node, depth_limit):
    c = 0
    solution_foud = False
    while c<=depth_limit:
        solution_found, nodes_expanded, stats, nodes_added, depth = Depth_Limited(start_Node,c,True)
        c+=1
        if solution_found:
            break
    return solution_found, nodes_expanded, stats, nodes_added, depth

#performs bfs
def Breadth_First(start_Node, is_smart):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    stats = [0] * 20
    filename = 'breadth_output.txt'
    if is_smart:
         filename = 'smarter_'+filename
    f = open(filename,'w')
    #create fringe as a queue structure
    tmp = [start_Node]
    fringe = deque(tmp)
    del tmp
    while (len(fringe) >0 and len(fringe) < 500000): #while fringe not empty
        node = fringe.popleft() #gets the first node added to the fringe (FIFO queue)
        stats[node.depth] += 1
        if node.is_goal_state():
            solution_found = True
            node.print_path_file(f)
            break
        if is_smart:
            if node.is_repeated():
                continue
        moves = node.get_validmoves()
        nodes_expanded += 1
        for i in moves:
            fringe.append(node.move(i))
            nodes_added += 1
    if (solution_found == False):
         print('Last Node Expanded')
    f.write('Found?\t'+str(solution_found)+'\n')
    f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
    f.write('Nodes Added\t'+str(nodes_added)+'\n')
    c = 0
    for x in stats:
        if x == 0:
            break
        f.write(str(c)+' '+str(x)+'\n')
        c +=1
    f.close
    return solution_found, nodes_expanded, stats, nodes_added

start_state = [['_', '_', '_', '_'],\
                ['_', '_', '_', '_'],\
                ['_', '_', '_', '_'],\
                ['A','B','C','#']]

goal_state = [['_', '_', '_', '_'],\
            ['_', 'A', '_', '_'],\
            ['_', 'B', '_', '_'],\
            ['_','C','#','_']]
start_Node = Node(start_state, [3,3], None, 0,-1,0)
start_Node.heuristic = start_Node.manhattan()
goal_Node = Node(goal_state, [3,2], None, 0,-1,0)

trials = int(1)
testing_depth = int(sys.argv[1])
t = 0
d = testing_depth -1
simple_stats = {'solution_found':0, 'nodes_expanded':0, 'nodes_added': 0, 'run_time': 0, "runs":0, 'depth':0}
depth_list = list()
for i in range(0,testing_depth+1):
    depth_list.append(copy.deepcopy(simple_stats))
Statistics = list()
for i in range(0,5):
    Statistics.append(copy.deepcopy(depth_list))
while d < testing_depth:
    d += 1
    print('Depth '+str(d))
    t = 0
    while t < trials:
        t += 1
        print('Trial '+str(t))
        gen_Node = goal_Node.generate_start(d)
        #gen_Node.print_step()
        #pprint(gen_Node.state)
        if gen_Node.is_goal_state():
            print('Start Node is goal state')
        else:
            #start_Node.print_step()
            #pprint(start_Node.state)
            # input('Press enter for smarter iterative deepening')
            print('Smart Iterative deepening')
            t0 = time.time()
            SIDsolution_found, SIDnodes_expanded, SIDstats, SIDnodes_added, found_depth = Iterative_Deepening(gen_Node,d)
            t1 = time.time()
            SIDrun_time = t1-t0
            Statistics[0][found_depth]['solution_found'] += SIDsolution_found
            Statistics[0][found_depth]['nodes_expanded'] += SIDnodes_expanded
            Statistics[0][found_depth]['nodes_added'] += SIDnodes_added
            Statistics[0][found_depth]['run_time'] += SIDrun_time
            Statistics[0][found_depth]['runs'] += 1
            print('Smart A*')
            t0 = time.time()
            Asolution_found, Anodes_expanded, Astats, Anodes_added = Astar(gen_Node)
            t1 = time.time()
            Arun_time = t1-t0
            Statistics[1][found_depth]['solution_found'] += Asolution_found
            Statistics[1][found_depth]['nodes_expanded'] += Anodes_expanded
            Statistics[1][found_depth]['nodes_added'] += Anodes_added
            Statistics[1][found_depth]['run_time'] += Arun_time
            Statistics[1][found_depth]['runs'] += 1
            print('Smart BFS')
            t0 = time.time()
            SBsolution_found, SBnodes_expanded, SBstats, SBnodes_added = Breadth_First(gen_Node, True)
            t1 = time.time()
            SBrun_time = t1-t0
            Statistics[2][found_depth]['solution_found'] += SBsolution_found
            Statistics[2][found_depth]['nodes_expanded'] += SBnodes_expanded
            Statistics[2][found_depth]['nodes_added'] += SBnodes_added
            Statistics[2][found_depth]['run_time'] += SBrun_time
            Statistics[2][found_depth]['runs'] += 1
            print('DFS')
            t0 = time.time()
            Dsolution_found, Dnodes_expanded, Dnodes_added, dfs_depth = Depth_First(gen_Node, False)
            t1 = time.time()
            Drun_time = t1-t0
            Statistics[3][found_depth]['solution_found'] += Dsolution_found
            Statistics[3][found_depth]['nodes_expanded'] += Dnodes_expanded
            Statistics[3][found_depth]['nodes_added'] += Dnodes_added
            if Dsolution_found:
                Statistics[3][found_depth]['run_time'] += Drun_time
            Statistics[3][found_depth]['runs'] += 1
            Statistics[3][found_depth]['depth'] += dfs_depth
            print('Smarter DFS')
            t0 = time.time()
            SDsolution_found, SDnodes_expanded, SDnodes_added, sdfs_depth = Depth_First(gen_Node, True)
            t1 = time.time()
            SDrun_time = t1-t0
            Statistics[4][found_depth]['solution_found'] += SDsolution_found
            Statistics[4][found_depth]['nodes_expanded'] += SDnodes_expanded
            Statistics[4][found_depth]['nodes_added'] += SDnodes_added
            if SDsolution_found:
                Statistics[4][found_depth]['run_time'] += SDrun_time
            Statistics[4][found_depth]['runs'] += 1
            Statistics[4][found_depth]['depth'] += sdfs_depth
method_names = ['IDS,', 'A*,', 'BFS', 'DFS,', 'SDFS,']
for i in range(0,5):
    print(method_names[i])
    for d in range(0,testing_depth):
        print('\tDepth, ',d+1,',')
        if Statistics[i][d+1]['runs'] == 0:
            continue
        averagefound = Statistics[i][d+1]['solution_found']/Statistics[i][d+1]['runs']
        averagetime = Statistics[i][d+1]['run_time']/Statistics[i][d+1]['solution_found']
        averageexpanded = Statistics[i][d+1]['nodes_expanded']/Statistics[i][d+1]['runs']
        averageadded = Statistics[i][d+1]['nodes_added']/Statistics[i][d+1]['runs']
        averagedepth = Statistics[i][d+1]['depth']/Statistics[i][d+1]['runs']
        print('\t\t Avg Solutions, ',averagefound,',')
        print('\t\t Avg Time, ',averagetime,',')
        print('\t\t Avg Expanded, ', averageexpanded,',')
        print('\t\t Avg Added, ', averageadded,',')
        print('\t\t Avg Depth, ', averagedepth,',')
        print('\t\t Runs, ', Statistics[i][d+1]['runs'],',')
