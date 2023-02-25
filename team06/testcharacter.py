# This is necessary to find the main code
import sys
import numpy as np
from queue import PriorityQueue
import math
import csv
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

def find_next(wrld):
    x = 0
    y = 0
    # Weigh each node based on A* to goal and distance from enemy, if tile we are trying to go to is within 1 of enemy, make it 0?
    return (x,y)

def magnitude(vector):
            return math.sqrt(sum(pow(element, 2) for element in vector))

def manhattan_distance(self,startx,starty,targetx,targety):
    return math.abs(targetx - startx) + math.abs(targety - starty)

def eights_distance(self, startx, starty, targetx, targety):
    return max(math.abs(targetx - startx), math.abs(targety - starty))

moves = [[-1,-1,0],[-1,1,0],[1,-1,0],[1,1,0],[-1,0,0],[0,-1,0],[1,0,0],[0,1,0],[0,0,0],[0,0,1]]

class TestCharacter(CharacterEntity):

    def __init__(self,name,avatar,x,y):
        super().__init__(name, avatar, x, y)

        # TODO ANDY : PULL FROM CSV RIGHT HERE
        self.weights = self.read_from_csv

        """
        List of Rewards Below
        """
        self.reward_win = 500
        self.death = -500
        self.cost_of_living = -1
        self.wall_demo = 10
        self.monster_kill = 50
        self.character_kill = 100

    def __init__(self,name,avatar,x,y,weight_list):
        super().__init__(name, avatar, x, y)
        self.weights = weight_list
        # TODO ANDY : PULL FROM CSV RIGHT HERE, ALSO FIGURE OUT WHERE TO SAVE TO CSV, B I T C H
        self.weights = self.read_from_csv

        """
        List of Rewards Below
        """
        self.reward_win = 500
        self.death = -500
        self.cost_of_living = -1
        self.wall_demo = 10
        self.monster_kill = 50
        self.character_kill = 100

    def do(self, wrld):
        self.q_learning(wrld)
        pass

    def q_learning(self,wrld):
        alpha = 0.1
        gamma = 0.9
        num_actions = 10
        num_f = 4
        (exit, exit_x, exit_y) = self.find_exit(wrld)
        monsters_position = self.find_monsters(wrld)

        best_action = [self.x,self.y]
        best_q_sa = 0

        q_sa_prime_max = 0
        q_sa = []
        actions = []

        f_values = [[]] * num_actions

        # start of q_sa
        for i in range(10): #calcualte q(s,a)
            action_position_x = self.x + moves[i][0]
            action_position_y = self.y + moves[i][1]

            #calculate all the f values needed for q(s,a)

            path_to_exit = self.a_star(wrld, action_position_x, action_position_y, exit_x, exit_y)
            character_direction

            f_direction = 0

            # TODO : Pretty sure these lists CANNOT be multiplies / divided, so make them numpy arrays probably
            if(len(path_to_exit) > 0): # Currently using Unit Vectors
                character_direction = np.array(path_to_exit[0]) / magnitude(np.array(path_to_exit[0]))
                # character_direction = np.array(path_to_exit[0]*len(path_to_exit))
                pass
            else:
                path_to_exit = [exit_x, exit_y]
                character_direction = np.array([action_position_x - path_to_exit[0],action_position_y - path_to_exit[1]]) / math.sqrt((action_position_x - path_to_exit[0])^2 + (action_position_y - path_to_exit[1])^2)
                # character_direction = np.array([action_position_x - path_to_exit[0],action_position_y - path_to_exit[1]])

                pass

            monster_direction
            for monster in monsters_position:
                path_to_mon = self.a_star(wrld, monster[0], monster[1], exit_x, exit_y)
                if(len(path_to_mon) > 0):
                    monster_direction = np.array(path_to_mon[0]) / magnitude(np.array(path_to_mon[0]))
                    # monster_direction = np.array(path_to_mon[0] * len(path_to_mon))
                    #there is path do something
                    pass
                else:
                    monster_direction = np.array([action_position_x - monster[0],action_position_y - monster[1]]) / math.sqrt((action_position_x - monster[0])^2 + (action_position_y - monster[1])^2)
                    # monster_direction = np.array([action_position_x - monster[0],action_position_y - monster[1]])
                    #there is no path, panic
                    pass

                # TODO : Right here is where we dot product character_direction*(-monster_direction) # monster erection
                f_direction += np.dot(character_direction,-monster_direction)

            astar_dist_exit = 1 / len(path_to_exit)

            """
            For the f_direction, which is just the weight or something for the direction we want to go in
            Dot product of the position we want to go to next, with the unit vector of the monster
            The position we want to go to next is given by:
            A* if there is a path that exists
            or
            The unit vector of the chracter to the goal
            
            The monster unit vector can either be
            the A* to the monster's next move
            Or
            The unit vector to the monster if the path does not exist
            Monster's unit vector might also be [1,1] - [the above] since we DONT want to go torwards the monster
            Actually, we will probably do the dot product of direction we want to go with the -dot product of the monster
            
            If multiple monsters, the result is additive
            
            Do we want A* of path length and distance from monster magnitude?
            Do we actually just want the vector to the monster, and does that mean that for the distance from goal,
            would that be a vector as well, so for A* it would be distance * next move?
            
            We want a smaller number the closer the monster is to us, so would it be nextmovevector - 1/monstervector?
            """

            #dist from bomb
            # check if bomb is in play, if in play, calculate distance - bomb in play IF len(wrld.bombs.value()) > 0 probably? Gotta debug that
            f_bomb_x = 0 # Large if close, small if far
            f_bomb_y = 0
            bombs = self.find_bomb(wrld)
            if bombs > 0:
                for bomb in bombs:
                    f_bomb_x += 1 / abs(action_position_x - bomb[0] + 0.001)
                    f_bomb_y += 1 / abs(action_position_y - bomb[1] + 0.001)

            # TODO : Copy above loop, but take into account the WORST (minimax) possible move the monster can make (Smallest A* to monster)
            # Get max of inner for loop, and then get the delta with the max
            f_values[i].append(f_direction)
            f_values[i].append(f_bomb_x)
            f_values[i].append(f_bomb_y)
            f_values[i].append(moves[i][2])
            # f_values[i+3] = 1

            q_sa.append(self.q_function(f_values[i]))
            place_bomb = moves[i][2]
            actions.append([action_position_x,action_position_y,place_bomb])
            """
            Have q_sa
            We now want q_sa_prime
            """

        # Now we have selected our action, so update the weights based on the rewards and q_sa_prime_max for that action
        max_q_sa = max(q_sa)
        index_best = q_sa.index(max_q_sa)
        action = actions[index_best]
        q_sa_prime = []
        f_values_prime = [[]] * num_actions

        for j in range(10):  # calcualte q(s_prime,a_prime)
            s_prime_position_x = action[0] + moves[j][0]
            s_prime_position_y = action[1] + moves[j][1]

            # calculate all the f values needed for q(s,a)

            path_to_exit = self.a_star(wrld, s_prime_position_x, s_prime_position_y, exit_x, exit_y)
            character_direction

            f_direction = 0

            if (len(path_to_exit) > 0):  # Currently using Unit Vectors
                character_direction = np.array(path_to_exit[0]) / magnitude(np.array(path_to_exit[0]))
                # character_direction = np.array(path_to_exit[0]*len(path_to_exit))
                pass
            else:
                path_to_exit = [exit_x, exit_y]
                character_direction = np.array(
                    [s_prime_position_x - path_to_exit[0], s_prime_position_y - path_to_exit[1]]) / math.sqrt(
                    (s_prime_position_x - path_to_exit[0]) ^ 2 + (s_prime_position_y - path_to_exit[1]) ^ 2)
                # character_direction = np.array([s_prime_position_x - path_to_exit[0],s_prime_position_y - path_to_exit[1]])

                pass

            monster_direction
            # probability_moves = [[]] * len(monsters_position)
            for monster in monsters_position:
                worst_move = [monster[0],monster[1]] # For us
                min_dist = 100
                monster_moves = self.look_for_empty_cell_monster(wrld, monster[0], monster[1])
                for move in monster_moves:  # 1 / len(monster_moves)
                    man_monster_dist = self.manhattan_distance(s_prime_position_x,s_prime_position_y,move[0],move[1])
                    if man_monster_dist < min_dist:
                        min_dist = man_monster_dist
                        worst_move = [move[0],move[1]]
                    # probability_moves[monsters_position.index(monster)].append((1 / len(monster_moves), move[0], move[1])) # Do we need this?

                path_to_mon = self.a_star(wrld, worst_move[0], worst_move[1], exit_x, exit_y)
                if (len(path_to_mon) > 0):
                    monster_direction = np.array(path_to_mon[0]) / magnitude(np.array(path_to_mon[0]))
                    # monster_direction = np.array(path_to_mon[0] * len(path_to_mon))
                    # there is path do something
                    pass
                else:
                    monster_direction = np.array(
                        [s_prime_position_x - worst_move[0], s_prime_position_y - worst_move[1]]) / math.sqrt(
                        (s_prime_position_x - worst_move[0]) ^ 2 + (s_prime_position_y - worst_move[1]) ^ 2)
                    # monster_direction = np.array([s_prime_position_x - monster[0],s_prime_position_y - monster[1]])
                    # there is no path, panic
                    pass

                # TODO : Right here is where we dot product character_direction*(-monster_direction) # monster erection
                f_direction += np.dot(character_direction, -monster_direction)

            # astar_dist_exit = 1 / len(path_to_exit)

            # dist from bomb
            # check if bomb is in play, if in play, calculate distance - bomb in play IF len(wrld.bombs.value()) > 0 probably? Gotta debug that
            f_bomb_x = 0  # Large if close, small if far
            f_bomb_y = 0
            if action[2] == 1:
                # TODO Pretend there's a bomb places
                bomb = [action[0],action[1]]
                f_bomb_x += 1 / abs(s_prime_position_x - bomb[0] + 0.001)
                f_bomb_y += 1 / abs(s_prime_position_y - bomb[1] + 0.001)
            if bombs > 0:
                for bomb in bombs:
                    f_bomb_x += 1 / abs(s_prime_position_x - bomb[0] + 0.001)
                    f_bomb_y += 1 / abs(s_prime_position_y - bomb[1] + 0.001)


            # TODO : Copy above loop, but take into account the WORST (minimax) possible move the monster can make (Smallest A* to monster)
            f_values_prime[j].append(f_direction)
            f_values_prime[j].append(f_bomb_x)
            f_values_prime[j].append(f_bomb_y)
            f_values_prime[j].append(moves[j][2])

            q_sa_prime.append(self.q_function(f_values_prime[j]))

        """
        Now we maximize q_sa_prime
        """
        q_sa_prime_max = max(q_sa_prime)
        """
        And now here's our rewards
        """
        reward = self.identify_rewards(wrld,[action[0],action[1]])
        # TODO : Delta function here
        delta = reward + gamma*q_sa_prime_max - q_sa
        for i in range(len(f_values[index_best])):
            self.weights += alpha*delta*f_values[index_best][i] #update the weights
            self.save_to_csv(self.weights) #save the list of weights to weights.csv

        if action[2] == 1:
            self.place_bomb()
            return action
        self.move(action[0] - self.x, action[1] - self.y)
        return action

    def weight_delta(self):
        pass
    
    def q_function(self,f_values):
        value = 0
        for i in range(len(f_values)):
            value += f_values[i] * self.weights[i]
            # We need to figure out if there are multiple monsters
            # TODO : Figure out how to do that
        return value

    def identify_rewards(self,wrld,next_position):
        reward = -1 #cost of living
        # If we predict our character dying (within range of monster or bomb has 1 left and either x and y distance from bomb is 0
        monsters_position = self.find_monsters(wrld)

        for monster in monsters_position:
            dist_to_monster = self.eights_distance(next_position[0],next_position[1],monster[0],monster[1])
            if dist_to_monster <= 1:
                reward -= 500 # we committed Foisie jump

        # If the bomb blows up and breaks a wall, add 10
        for bomb in self.get_bombs(wrld):
            for i in range(4):
                if wrld.monsters_at(min(max(0,bomb[0] + i),wrld.width),bomb[1]):
                    reward += 50
                if wrld.wall_at(min(max(0, bomb[0] + i), wrld.width), bomb[1]):
                    reward += 10
                if next_position[0] == min(max(0, bomb[0] + i), wrld.width) and next_position[1] == bomb[1]:
                    reward -= 500
                if wrld.monsters_at(min(max(0,bomb[0] - i),wrld.width),bomb[1]):
                    reward += 50
                if wrld.wall_at(min(max(0, bomb[0] - i), wrld.width), bomb[1]):
                    reward += 10
                if next_position[0] == min(max(0, bomb[0] - i), wrld.width) and next_position[1] == bomb[1]:
                    reward -= 500
                if wrld.monsters_at(bomb[0],min(max(0,bomb[1] + i),wrld.height)):
                    reward += 50
                if wrld.wall_at(bomb[0], min(max(0,bomb[1] + i),wrld.height)):
                    reward += 10
                if next_position[1] == min(max(0, bomb[1] + i), wrld.width) and next_position[0] == bomb[0]:
                    reward -= 500
                if wrld.monsters_at(bomb[0],min(max(0,bomb[1] - i),wrld.height)):
                    reward += 50
                if wrld.wall_at(bomb[0], min(max(0,bomb[1] - i),wrld.height)):
                    reward += 10
                if next_position[1] == min(max(0, bomb[1] - i), wrld.width) and next_position[0] == bomb[0]:
                    reward -= 500

        (exit, exit_x, exit_y) = self.find_exit(wrld)
        if next_position[0] == exit_x and next_position[1] == exit_y:
            reward += 500

        return reward


    def save_to_csv(self, weights): #saves a list of numbers to weights.csv
        # Save the weights in order to a CSV

        # field names
        fields = ['Weights']

        # data rows of csv file
        rows = []
        for w in weights:
                rows.append([w])

        with open('weights.csv', 'w',  newline='') as f:
            # using csv.writer method from CSV package
            write = csv.writer(f)

            write.writerow(fields)
            write.writerows(rows)
        pass

    def read_from_csv(self): #returns a list of numbers saved in weights.csv
        file = open('weights.csv')
        type(file)
        csvreader = csv.reader(file)
        header = []
        header = next(csvreader)
        rows = []
        for row in csvreader:
            print(row)
            rows.append(float(row[0]))
        return rows

    # TODO : A* for a certain target in a given world from a certain position
    def a_star(self,wrld,startingx,startingy,targetx,targety):
        start = (startingx, startingy)
        goal = (targetx, targety)
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}
        cost = {}
        came_from[start] = None
        cost[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            neighbors = self.look_for_empty_cell_monster(wrld, current[0], current[1])
            for n in neighbors:
                n_cost = self.get_hypotnuse(current,n)
                new_cost = cost[current] + 1
                if n not in cost or new_cost < cost[n]:
                    cost[n] = new_cost
                    priority = new_cost + n_cost
                    frontier.put(n,priority)
                    came_from[n] = current
        path = []
        try:
            current_cell = goal
            while not current_cell == None:
                path.append(current_cell)
                current_cell = came_from[current_cell]

            path = list(reversed(path))
            return path # TODO :  We might need a try catch here if there is no path
        except:
            return path

    def look_for_empty_cell(self, wrld):
        # List of empty cells
        cells = []
        # Go through neighboring cells
        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((self.x + dx >= 0) and (self.x + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((self.y + dy >= 0) and (self.y + dy < wrld.height())):
                        # Is this cell safe?
                        if(wrld.exit_at(self.x + dx, self.y + dy) or
                           wrld.empty_at(self.x + dx, self.y + dy)):
                            # Yes
                            cells.append((dx, dy))
        # All done
        return cells

    def get_hypotnuse(self,cell1,cell2):

        #takes 2 tuples (cell_x, cell_y) and calculates the distance between them
        return math.hypot(abs(cell1[0]-cell2[0]), abs(cell1[1]-cell2[1]))


    def expectimax_do(self,wrld):
        (enemy,ex,ey) = self.within_range(3,wrld)
        if enemy:
            step = self.expectimax(wrld)
            self.move(step[0] - self.x, step[1] - self.y)
            pass
        else:
            (exit, exit_x, exit_y) = self.find_exit(wrld)
            if exit:
                path = self.a_star(wrld, self.x, self.y, exit_x, exit_y)
                step = path[1]
                self.move(step[0] - self.x, step[1] - self.y)
            else:
                print("oh fuck there's no exit")
            pass
        pass

    #check for monster near the character within a certain range, return boolean and position of enemy
    def within_range(self, rnge, wrld):
        """
        We make the assumption we are looking for an enemy within range of us
        :param rnge: the range to search for an enemy
        :param wrld: the world which is being searched
        """
        for dx in range(-rnge, rnge + 1):
            # Avoid out-of-bounds access
            if ((self.x + dx >= 0) and (self.x + dx < wrld.width())):
                for dy in range(-rnge, rnge + 1):
                    # Avoid out-of-bounds access
                    if ((self.y + dy >= 0) and (self.y + dy < wrld.height())):
                        # Is a character at this position?
                        if (wrld.monsters_at(self.x + dx, self.y + dy)):
                            return (True, dx, dy)
        # Nothing found
        return (False, 0, 0)

    def chance_monster_value(self, wrld, monster_moves, move_x, move_y):
        """
        Takes in wrld, the list and probabilities of each monster move, and the move's x and y to figure out the value
        of taking this move with respect to monster, not taking into account anything else
        Returns a value, 0 if not a viable move (monster will kill us), above 0 if otherwise
        monster moves is a (probability, x, y)
        The higher the number is, the better for bomberman
        """
        value = 0
        for mmove in monster_moves:
            path = self.a_star(wrld, move_x, move_y, mmove[1], mmove[2])
            dist_from_monster = len(path)-1
            if(dist_from_monster <= 2): # is this 1 or 2
                value -= 2 * mmove[0] # TODO : Fine tune this value
            value += (dist_from_monster) * mmove[0]
        return value

    def expectimax(self, wrld):
        possible_moves = self.weigh_moves(wrld)
        max = 0
        max_move = None
        for move in possible_moves:
            if move[2] >= max:
                max = move[2]
                max_move = (move[0],move[1])
        return max_move


    def weigh_moves(self,wrld):
        moves = self.look_for_empty_cell_character(wrld)
        dists_from_goal = [None] * len(moves)
        monster_chance_values = [0] * len(moves) # Moves that we can make weighted with the moves monster can make
        (exit,exit_x,exit_y) = self.find_exit(wrld)
        monsters_at = self.find_monsters(wrld)

        #look for empty cells around monster to generate possible monster moves
        probability_moves = [[]] * len(monsters_at)
        for monster in monsters_at:
            monster_moves = self.look_for_empty_cell_monster(wrld,monster[0],monster[1])
            for move in monster_moves: # 1 / len(monster_moves)
                probability_moves[monsters_at.index(monster)].append((1/len(monster_moves),move[0],move[1])) # TODO : Jank
        # Set probability of monster doing that move

        max_path_length = 0
        for move in moves:
            index = moves.index(move)

            path = self.a_star(wrld, move[0], move[1], exit_x, exit_y)
            path_length = len(path)
            if max_path_length < path_length: max_path_length = path_length
            dists_from_goal[index] = path_length
            for i in range(len(monsters_at)):
                monster_chance_values[index] += self.chance_monster_value(wrld,probability_moves[i],move[0],move[1]) # TODO Jank



        # TODO : Using stuff from above, use equation below to weigh the viable neighbors
        # (max_distance - distance + 1) * (distance_from_monster - 1)
        move_utilities = []
        for i in range(len(moves)):
            # TODO : This definitely does not work, maybe have an external weight if we are super close to monter?
            big_scalar = 10
            utility = (max_path_length - dists_from_goal[i] + 1) * (monster_chance_values[i] * big_scalar)
            move_utilities.append((moves[i][0], moves[i][1], utility))

        return move_utilities

    def find_exit(self,wrld):
        for x in range(len(wrld.grid)):
            for y in range(len(wrld.grid[0])):
                if wrld.exit_at(x, y):
                    return (True, x, y)
        return (False, 0, 0)

    def find_monsters(self,wrld):
        monster_locations = []
        monsters = wrld.monsters.values()
        for monster in monsters:
            monster_locations.append((monster[0].x, monster[0].y))
        return monster_locations

    def find_bomb(self,wrld):
        bomb_locations = []
        bombs = wrld.bombs.values()
        for bomb in bombs:
            bomb_locations.append((bomb[0].x, bomb[0].y))
        return bomb_locations

    def look_for_empty_cell_character(self, wrld):
        # List of empty cells
        cells = []
        # Go through neighboring cells
        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((self.x + dx >= 0) and (self.x + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((self.y + dy >= 0) and (self.y + dy < wrld.height())):
                        # Is this cell safe?
                        if(wrld.exit_at(self.x + dx, self.y + dy) or
                           wrld.empty_at(self.x + dx, self.y + dy)):
                            # Yes
                            cells.append((self.x + dx, self.y + dy))
        # All done
        return cells

    def look_for_empty_cell_monster(self, wrld, monster_x, monster_y):
        # List of empty cells
        cells = []
        # Go through neighboring cells
        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((monster_x + dx >= 0) and (monster_x + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((monster_y + dy >= 0) and (monster_y + dy < wrld.height())):
                        # Is this cell safe?
                        if(wrld.exit_at(monster_x + dx, monster_y + dy) or
                           wrld.empty_at(monster_x + dx, monster_y + dy)):
                            # Yes
                            cells.append((monster_x + dx, monster_y + dy))
        # All done
        return cells

