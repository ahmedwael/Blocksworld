from pprint import pprint
import copy
from collections import deque
from random import randint, shuffle
import time
import sys

class Node:

    def __init__(self, state, space, parent, depth, step, heuristic):
        self.state = state
        self.space = space
        self.parent = parent
        self.depth = depth
        self.step = step
        self.heuristic = heuristic

    #also can be seen as the expand function
    def move(self,direction): #right 0, up 1, left 2, down 3
        if direction == 0: #right
            #print('right')
            if self.space[1] == 3:
                return "Invalid Move!"
            n_space = [self.space[0], self.space[1]+1]
        elif direction == 1: #up
            #print('up')
            if self.space[0] == 0:
                return "Invalid Move!"
            n_space = [self.space[0]-1, self.space[1]]
        elif direction == 2: #left
            #print('left')
            if self.space[1] == 0:
                return "Invalid Move!"
            n_space = [self.space[0], self.space[1]-1]
        elif direction == 3: #down
            #print('down')
            if self.space[0] == 3:
                return "Invalid Move!"
            n_space = [self.space[0]+1, self.space[1]]
        n_state = copy.deepcopy(self.state)
        n_state[self.space[0]][self.space[1]] = n_state[n_space[0]][n_space[1]]
        n_state[n_space[0]][n_space[1]] = '#'
        new = Node(n_state, n_space, self, self.depth + 1, direction,0)
        new.heuristic = new.manhattan()
        return new

    def is_goal_state(self):
        if (self.state[1][1] == 'A' and self.state[2][1] == 'B' and self.state[3][1] == 'C'):
            return True
        return False

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

    def is_repeated(self):
        node = self
        while True:
            if node.parent == None:
                return False
            node = node.parent
            if self.state == node.state:
                return True
        pass

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

    def print_step(self):
        steps = {-1:'start', 0:'right', 1:'up', 2:'left', 3:'down'}
        pprint(steps[self.step])
        pass

    def print_step_file(self,f):
        steps = {-1:'start', 0:'right', 1:'up', 2:'left', 3:'down'}
        f.write(str(steps[self.step])+'\n')
        pass

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
        print('Depth\t',self.depth)
        f.write('Depth\t'+str(self.depth)+"\n")
        pass

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
                    if (tmp.is_repeated() == False and tmp.is_goal_state() == False):
                        del tmp
                        node = node.move(direction)
                        fails = 0
                        break
                    fails += 1
                    if fails == 100:
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
        #pprint('---------------Generated-----------------')
        return node

def Astar(start_Node, is_smart):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    stats = dict()
    # filename = 'astar_output.txt'
    # if is_smart:
    #     filename = 'smarter_'+filename
    # f = open(filename,'w')
    fringe = [start_Node]
    while (len(fringe) >0 and len(fringe) < 2500000): #while fringe not empty
        fringe.sort(key=lambda x: x.depth + x.heuristic, reverse = True)
        node = fringe.pop()
        #input('Min')
        #pprint(node.depth + node.heuristic)
        if node.is_goal_state():
            solution_found = True
            # node.print_path_file(f)
            break
        if is_smart:
            if node.is_repeated():
                continue
        moves = node.get_validmoves()
        nodes_expanded += 1
        if node.depth in stats:
            stats[node.depth] += 1
        else:
            stats[node.depth] = 1
        for i in moves:
            fringe.append(node.move(i))
            nodes_added += 1
    # if (solution_found == False):
         # print('Last Node Expanded')
         # node.print_path()
    # f.write('Found?\t'+str(solution_found)+'\n')
    # f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
    # f.write('Nodes Added\t'+str(nodes_added)+'\n')
    # print('Found?\t',solution_found)
    # print('Nodes Expanded\t',nodes_expanded)
    # print('Nodes Added\t',nodes_added)
    # for k, v in stats.items():
    #     f.write(str(k)+' '+str(v)+'\n')
    # f.close()
    return solution_found, nodes_expanded, stats, nodes_added

def Depth_First(start_Node, is_smart):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    # filename ='depth_output.txt'
    # if is_smart:
    #     filename = 'smarter_'+filename
    # f = open(filename,'w')
    #create fringe as a stack structure
    fringe = [start_Node]
    while (len(fringe) >0):# and len(fringe) < 2500000): #while fringe not empty
        node = fringe.pop()
        if node.is_goal_state():
            solution_found = True
            # node.print_path_file(f)
            break
        if is_smart:
            if node.is_repeated():
                continue
        moves = node.get_validmoves()
        nodes_expanded += 1
        shuffle(moves)
        for i in moves:
            fringe.append(node.move(i))
            nodes_added += 1
    # if (solution_found == False):
         # print('Last Node Expanded')
         # node.print_path()
    # f.write('Found?\t'+str(solution_found)+'\n')
    # f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
    # f.write('Nodes Added\t'+str(nodes_added)+'\n')
    # print('Found?\t',solution_found)
    # print('Nodes Expanded\t',nodes_expanded)
    # print('Nodes Added\t',nodes_added)
    # f.close()
    return solution_found, nodes_expanded, nodes_added


def Depth_Limited(start_Node, depth_limit, is_smart, itr):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    stats = dict()
    # if itr == False:
    #     filename = 'depth_limited_output.txt'
    #     if is_smart:
    #         filename = 'smarter_'+filename
    # else:
    #     filename = 'iterative_deepening_output.txt'
    # if is_smart:
    #     filename = 'smarter_'+filename
    # f = open(filename,'w')
    #create fringe as a stack structure
    fringe = [start_Node]
    while (len(fringe) >0 and len(fringe) < 2500000): #while fringe not empty
        node = fringe.pop()
        if node.is_goal_state():
            solution_found = True
            # node.print_path_file(f)
            break
        if is_smart:
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
    #if (solution_found == False):
    #     print('Last Node Expanded')
    #     node.print_path()
    # if (solution_found == True):
    #     f.write('Found?\t'+str(solution_found)+'\n')
    #     f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
    #     f.write('Nodes Added\t'+str(nodes_added)+'\n')
    #     print('Found?\t',solution_found)
    #     print('Nodes Expanded\t',nodes_expanded)
    #     print('Nodes Added\t',nodes_added)
    #     for k, v in stats.items():
    #         f.write(str(k)+' '+str(v)+'\n')
    # f.close()
    return solution_found, nodes_expanded, stats, nodes_added, node.depth

def Iterative_Deepening(start_Node, depth_limit, is_smart):
    c = 0
    solution_foud = False
    while c<=depth_limit:
        solution_found, nodes_expanded, stats, nodes_added, depth = Depth_Limited(start_Node,c, is_smart,True)
        c+=1
        if solution_found:
            break
        #print('Solution not found', c-1)
    return solution_found, nodes_expanded, stats, nodes_added, depth

def Breadth_First(start_Node,depth_limit, is_smart):
    #counters and flags
    solution_found = False
    nodes_expanded = 0
    nodes_added = 1
    stats = [0] * 20
    # filename = 'breadth_output.txt'
    # if is_smart:
    #     filename = 'smarter_'+filename
    # f = open(filename,'w')
    #create fringe as a queue structure
    tmp = [start_Node]
    fringe = deque(tmp)
    del tmp
    while (len(fringe) >0 and len(fringe) < 500000): #while fringe not empty
        node = fringe.popleft()
        if node.depth>depth_limit:
            break
        stats[node.depth] += 1
        if node.is_goal_state():
            solution_found = True
            # node.print_path_file(f)
            break
        if is_smart:
            if node.is_repeated():
                continue
        moves = node.get_validmoves()
        nodes_expanded += 1
        for i in moves:
            fringe.append(node.move(i))
            nodes_added += 1
    # if (solution_found == False):
    #     print('Last Node Expanded')
    # f.write('Found?\t'+str(solution_found)+'\n')
    # f.write('Nodes Expanded\t'+str(nodes_expanded)+'\n')
    # f.write('Nodes Added\t'+str(nodes_added)+'\n')
    # print('Found?\t',solution_found)
    # print('Nodes Expanded\t',nodes_expanded)
    # print('Nodes Added\t',nodes_added)
    # c = 0
    # for x in stats:
    #     if x == 0:
    #         break
    #     f.write(str(c)+' '+str(x)+'\n')
    #     c +=1
    # f.close
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

trials = int(sys.argv[1])
testing_depth = int(sys.argv[2])
t = 0
d = 0
simple_stats = {'solution_found':0, 'nodes_expanded':0, 'nodes_added': 0, 'run_time': 0, "runs":0}
depth_list = list()
for i in range(0,testing_depth+1):
    depth_list.append(copy.deepcopy(simple_stats))
Statistics = list()
for i in range(0,3):
    Statistics.append(copy.deepcopy(depth_list))
#method_list[1]['solution_found'] = 1
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
            SIDsolution_found, SIDnodes_expanded, SIDstats, SIDnodes_added, found_depth = Iterative_Deepening(gen_Node,d,True)
            t1 = time.time()
            SIDrun_time = t1-t0
            Statistics[0][found_depth]['solution_found'] += SIDsolution_found
            Statistics[0][found_depth]['nodes_expanded'] += SIDnodes_expanded
            Statistics[0][found_depth]['nodes_added'] += SIDnodes_added
            Statistics[0][found_depth]['run_time'] += SIDrun_time
            Statistics[0][found_depth]['runs'] += 1
            # print(t1-t0)
            print('Smart A*')
            # input('Press enter for smarter A*')
            t0 = time.time()
            Asolution_found, Anodes_expanded, Astats, Anodes_added = Astar(gen_Node,True)
            t1 = time.time()
            Arun_time = t1-t0
            Statistics[1][found_depth]['solution_found'] += Asolution_found
            Statistics[1][found_depth]['nodes_expanded'] += Anodes_expanded
            Statistics[1][found_depth]['nodes_added'] += Anodes_added
            Statistics[1][found_depth]['run_time'] += Arun_time
            Statistics[1][found_depth]['runs'] += 1
            # print(t1-t0)
            # input('Press enter for A*')
            # t0 = time.time()
            # Asolution_found, Anodes_expanded, Astats, Anodes_added = Astar(gen_Node, False)
            # t1 = time.time()
            # print(t1-t0)
            # input('Press enter for iterative deepening')
            # t0 = time.time()
            # IDsolution_found, IDnodes_expanded, IDstats, IDnodes_added = Iterative_Deepening(gen_Node,100,False)
            # t1 = time.time()
            # print(t1-t0)
            # input('Press enter for smarter depth limited')
            # t0 = time.time()
            # SDLsolution_found, SDLnodes_expanded, SDLstats, SDLnodes_added = Depth_Limited(start_Node,14,True,False)
            # t1 = time.time()
            # print(t1-t0)
            # input('Press enter for depth limited')
            # t0 = time.time()
            # DLsolution_found, DLnodes_expanded, DLstats, DLnodes_added = Depth_Limited(start_Node,14, False, False)
            # t1 = time.time()
            # print(t1-t0)
            # input('Press enter for smarter breadth first')
            if found_depth > 100:
                continue
            print('Smart BFS')
            t0 = time.time()
            SBsolution_found, SBnodes_expanded, SBstats, SBnodes_added = Breadth_First(gen_Node,100,True)
            t1 = time.time()
            SBrun_time = t1-t0
            Statistics[2][found_depth]['solution_found'] += SBsolution_found
            Statistics[2][found_depth]['nodes_expanded'] += SBnodes_expanded
            Statistics[2][found_depth]['nodes_added'] += SBnodes_added
            Statistics[2][found_depth]['run_time'] += SBrun_time
            Statistics[2][found_depth]['runs'] += 1
            # print(t1-t0)
            # input('Press enter for breadth first')
            # print('\nBFS')
            # t0 = time.time()
            # Bsolution_found, Bnodes_expanded, Bstats, Bnodes_added = Breadth_First(gen_Node,100,False)
            # t1 = time.time()
            # Bruntime = t1-t0
            # print(t1-t0)
            # print('\nSmart DFS')
            # input('Press enter for smarter depth first')
            # t0 = time.time()
            # SDsolution_found, SDnodes_expanded, SDnodes_added = Depth_First(gen_Node, True)
            # t1 = time.time()
            # SDruntime = t1-t0
            # print(t1-t0)
            # input('Press enter for depth first')
            # t0 = time.time()
            # Dsolution_found, Dnodes_expanded, Dnodes_added = Depth_First(gen_Node, False)
            # t1 = time.time()
            # print(t1-t0)
#pprint(Statistics)
method_names = ['IDS', 'A*', 'BFS']
for i in range(0,3):
    print(method_names[i])
    for d in range(0,testing_depth):
        print('\tDepth ',d+1)
        if Statistics[i][d+1]['runs'] == 0:
            continue
        averagetime = Statistics[i][d+1]['run_time']/Statistics[i][d+1]['runs']
        averageexpanded = Statistics[i][d+1]['nodes_expanded']/Statistics[i][d+1]['runs']
        averageadded = Statistics[i][d+1]['nodes_added']/Statistics[i][d+1]['runs']
        print('\t\t Avg Time ',averagetime)
        print('\t\t Avg Expanded ', averageexpanded)
        print('\t\t Avg Added ', averageadded)
        print('\t\t Runs ', Statistics[i][d+1]['runs'])
