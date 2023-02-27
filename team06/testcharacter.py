# This is necessary to find the main code
import sys
import numpy as np
from queue import PriorityQueue
import math
import time
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

def manhattan_distance(startx,starty,targetx,targety):
    return abs(targetx - startx) + abs(targety - starty)

def eights_distance(startx, starty, targetx, targety):
    return max(abs(targetx - startx), abs(targety - starty))

moves = [[-1,-1,0],[-1,1,0],[1,-1,0],[1,1,0],[-1,0,0],[0,-1,0],[1,0,0],[0,1,0],[0,0,0],[0,0,1]]

class TestCharacter(CharacterEntity):

    def __init__(self,name,avatar,x,y,weights):
        super().__init__(name, avatar, x, y)
        self.weights = weights

        """
        List of Rewards Below
        """
        self.reward_win = 500
        self.death = -200
        self.bomb_death = -500
        self.cost_of_living = -1
        self.wall_demo = 10
        self.monster_kill = 10
        self.character_kill = 100
        self.place_bomb_reward = 1

    def do(self, wrld):
        self.q_learning(wrld)
        time.sleep(0.1)
        pass

    def q_learning(self,wrld):
        alpha = 0.25
        gamma = 0.9
        num_actions = 10
        num_f = 4
        (exit, exit_x, exit_y) = self.find_exit(wrld)
        monsters_position = self.find_monsters(wrld)

        best_action = [self.x,self.y]
        best_q_sa = 0

        q_sa_prime_max = 0
        q_sa = [-1000000 for _ in range(num_actions)]
        actions = [[0,0,0] for _ in range(num_actions)]

        f_values = [[] for _ in range(num_actions)]

        start_to_exit = self.a_star(wrld,self.x, self.y, exit_x, exit_y)
        cell_to_exit = self.find_close_to_exit(wrld, self.x, self.y, exit_x,
                                                       exit_y)  # TODO : Make this the path from the current location, and then take the dot product with the vector we want to use
        start_to_cell = self.a_star(wrld, self.x, self.y, cell_to_exit[0],
                                           cell_to_exit[1])

        if len(start_to_exit) > 0: # TODO : Figure out if these are selecting the correct indices
            next_cell = start_to_exit[1]
            start_to_exit_vector = np.array([next_cell[0] - self.x, next_cell[1] - self.y]) / len(start_to_exit)
        else:
            try:
                next_cell = start_to_cell[1]
                start_to_exit_vector = np.array([next_cell[0] - self.x, next_cell[1] - self.y]) / len(cell_to_exit)
            except: # Something went wrong
                start_to_exit_vector = np.array([0,0])

        # start of q_sa
        for i in range(10): #calcualte q(s,a)

            action_position_x = self.x + moves[i][0]
            action_position_y = self.y + moves[i][1]
            if action_position_x < 0 or action_position_y < 0 or action_position_x >= wrld.width() or action_position_y >= wrld.height():
                continue

            if wrld.empty_at(action_position_x, action_position_y) or wrld.characters_at(action_position_x, action_position_y) or wrld.explosion_at(action_position_x, action_position_y) or wrld.exit_at(action_position_x,action_position_y):
                #calculate all the f values needed for q(s,a)

                path_to_exit = self.a_star(wrld, action_position_x, action_position_y, exit_x, exit_y)
                character_direction = np.array([0,0])

                f_closest_to_exit = 0
                cell_closest_to_exit = self.find_close_to_exit(wrld, action_position_x, action_position_y, exit_x, exit_y) # TODO : Make this the path from the current location, and then take the dot product with the vector we want to use
                path_closest_to_exit = self.a_star(wrld,action_position_x, action_position_y, cell_closest_to_exit[0], cell_closest_to_exit[1])

                move_vector = np.array([moves[i][0], moves[i][1]])

                f_direction = 0
                # TODO : Pretty sure these lists CANNOT be multiplies / divided, so make them numpy arrays probably
                if(len(path_to_exit) > 0): # Currently using Unit Vectors
                    # character_direction = np.array(path_to_exit[0]) / magnitude(np.array(path_to_exit[0]))
                    f_direction = np.dot(move_vector, start_to_exit_vector)/len(path_to_exit)
                    move_vector *= len(path_to_exit)
                else:
                    f_direction = np.dot(move_vector, start_to_exit_vector)/len(path_closest_to_exit)
                    move_vector *= len(path_closest_to_exit)
                    f_closest_to_exit = 1

                f_monster_direction = 0
                monster_direction = np.array([0,0])
                for monster in monsters_position:
                    # path_to_mon = self.a_star(wrld, monster[0], monster[1], exit_x, exit_y)
                    # if(len(path_to_mon) > 0):
                        # monster_direction = np.array(path_to_mon[0]) / magnitude(np.array(path_to_mon[0]))
                        # monster_direction = np.array(path_to_mon[0] * len(path_to_mon))
                        #there is path do something
                    # else:
                        # monster_direction = np.array([action_position_x - monster[0],action_position_y - monster[1]]) / math.sqrt((action_position_x - monster[0])^2 + (action_position_y - monster[1])^2)
                    monster_direction = np.array([monster[0] - action_position_x, monster[1] - action_position_y]) / math.sqrt((action_position_x - monster[0])**2 + (action_position_y - monster[1])**2 + 0.1)


                    # TODO : Right here is where we dot product character_direction*(-monster_direction) # monster erection
                    f_monster_direction += .1 * np.dot(move_vector,monster_direction)

                f_closest_to_exit = 2 * f_direction * f_closest_to_exit
                f_monster_direction = f_monster_direction * len(monsters_position)
                print(f_monster_direction)

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
                if len(bombs) > 0:
                    for bomb in bombs:
                        f_bomb_x += 1 / (abs(action_position_x - bomb[0]) + 0.1)
                        f_bomb_y += 1 / (abs(action_position_y - bomb[1]) + 0.1)

                # TODO : Copy above loop, but take into account the WORST (minimax) possible move the monster can make (Smallest A* to monster)
                # Get max of inner for loop, and then get the delta with the max
                f_values[i].append(f_direction)
                f_values[i].append(f_bomb_x)
                f_values[i].append(f_bomb_y)
                # if moves[i][2] == 1:
                #     print("We should be incentivized to place a bomb")
                f_values[i].append(moves[i][2])
                f_values[i].append(f_monster_direction)
                f_values[i].append(f_closest_to_exit)

                # f_values[i+3] = 1

                q_sa[i] = self.q_function(f_values[i])
                place_bomb = moves[i][2]
                actions[i] = [action_position_x,action_position_y,place_bomb]
                """
                Have q_sa
                             f_monster_direction = f_monster_direction * len(monsters_position)
   We now want q_sa_prime
                """

        # Now we have selected our action, so update the weights based on the rewards and q_sa_prime_max for that action
        max_q_sa = max(q_sa)
        index_best = q_sa.index(max_q_sa)
        action = actions[index_best]
        q_sa_prime = [-1000000 for _ in range(num_actions)]
        f_values_prime = [[] for _ in range(num_actions)]

        start_to_exit = self.a_star(wrld, action[0], action[1], exit_x, exit_y)
        cell_to_exit = self.find_close_to_exit(wrld, action[0], action[1], exit_x,
                                               exit_y)  # TODO : Make this the path from the current location, and then take the dot product with the vector we want to use
        start_to_cell = self.a_star(wrld, action[0], action[1], cell_to_exit[0],
                                    cell_to_exit[1])

        if len(start_to_exit) > 0:  # TODO : Figure out if these are selecting the correct indices
            try:
                next_cell = start_to_exit[1]
                start_to_exit_vector = np.array([next_cell[0] - action[0], next_cell[1] - action[1]]) / len(start_to_exit)
            except: # We are at the finish line so we don't need a q_sa_prime, but we are still going to run it
                next_cell = start_to_exit[0]
                start_to_exit_vector = np.array([next_cell[0] - action[0], next_cell[1] - action[1]])
        else:
            try:
                next_cell = start_to_cell[1]
                start_to_exit_vector = np.array([next_cell[0] - action[0], next_cell[1] - action[1]]) / len(cell_to_exit)
            except: # Problems happened, we might have been blocked off by an explosion
                start_to_exit_vector = np.array([0,0])

        for j in range(10):  # calcualte q(s_prime,a_prime)
            s_prime_position_x = action[0] + moves[j][0]
            s_prime_position_y = action[1] + moves[j][1]
            # print([wrld.width(), wrld.height(),s_prime_position_x, s_prime_position_y])
            if s_prime_position_x < 0 or s_prime_position_y < 0 or s_prime_position_x >= wrld.width() or s_prime_position_y >= wrld.height():
                # print("It doesn't think %d %d is a location we can go" % (s_prime_position_x,s_prime_position_y))
                continue

            # calculate all the f values needed for q(s,a)
            if wrld.empty_at(s_prime_position_x, s_prime_position_y) or wrld.characters_at(s_prime_position_x, s_prime_position_y) or wrld.explosion_at(s_prime_position_x, s_prime_position_y):

                f_direction = 0
                move_vector = np.array([moves[i][0], moves[i][1]])
                # TODO : Pretty sure these lists CANNOT be multiplies / divided, so make them numpy arrays probably
                if (len(path_to_exit) > 0):  # Currently using Unit Vectors
                    # character_direction = np.array(path_to_exit[0]) / magnitude(np.array(path_to_exit[0]))
                    f_direction = np.dot(move_vector, start_to_exit_vector) / len(path_to_exit)
                    move_vector *= len(path_to_exit)
                else:
                    f_direction = np.dot(move_vector , start_to_exit_vector) / len(path_closest_to_exit)
                    move_vector *= len(path_closest_to_exit)
                    f_closest_to_exit = 1

                f_monster_direction = 0
                monster_direction = np.array([0, 0])
                for monster in monsters_position:
                    # path_to_mon = self.a_star(wrld, monster[0], monster[1], exit_x, exit_y)
                    # if(len(path_to_mon) > 0):
                    # monster_direction = np.array(path_to_mon[0]) / magnitude(np.array(path_to_mon[0]))
                    # monster_direction = np.array(path_to_mon[0] * len(path_to_mon))
                    # there is path do something
                    # else:
                    # monster_direction = np.array([action_position_x - monster[0],action_position_y - monster[1]]) / math.sqrt((action_position_x - monster[0])^2 + (action_position_y - monster[1])^2)
                    monster_direction = np.array([monster[0] - action_position_x, monster[1] - action_position_y]) / math.sqrt((action_position_x - monster[0])**2 + (action_position_y - monster[1])**2 + 0.1)
                    # there is no path, panic

                    # TODO : Right here is where we dot product character_direction*(-monster_direction) # monster erection
                    f_monster_direction += np.dot(move_vector, monster_direction)


                f_closest_to_exit = 2 * f_direction * f_closest_to_exit

                # dist from bomb
                # check if bomb is in play, if in play, calculate distance - bomb in play IF len(wrld.bombs.value()) > 0 probably? Gotta debug that
                f_bomb_x = 0  # Large if close, small if far
                f_bomb_y = 0
                f_explosion = 0
                # if action[2] == 1:
                #     # TODO Pretend there's a bomb places
                #     bomb = [action[0],action[1]]
                #     f_bomb_x += 1 / abs(s_prime_position_x - bomb[0] + 0.1)
                #     f_bomb_y += 1 / abs(s_prime_position_y - bomb[1] + 0.1)
                    # """
                    #                 If we are placing a bomb, we want to run away from the explosion
                    #                 """
                    # for k in range(4):
                    #     f_explosion += 1 / math.sqrt(
                    #         (s_prime_position_x - bomb[0] + k) ** 2 + (s_prime_position_y - bomb[1]) ** 2)
                    #     f_explosion += 1 / math.sqrt(
                    #         (s_prime_position_x - bomb[0] - k) ** 2 + (s_prime_position_y - bomb[1]) ** 2)
                    #     f_explosion += 1 / math.sqrt(
                    #         (s_prime_position_x - bomb[0]) ** 2 + (s_prime_position_y - bomb[1] + k) ** 2)
                    #     f_explosion += 1 / math.sqrt(
                    #         (s_prime_position_x - bomb[0]) ** 2 + (s_prime_position_y - bomb[1] - k) ** 2)
                if len(bombs) > 0:
                    for bomb in bombs:
                        f_bomb_x += 1 / abs(s_prime_position_x - bomb[0] + 0.1)
                        f_bomb_y += 1 / abs(s_prime_position_y - bomb[1] + 0.1)


                # TODO : Copy above loop, but take into account the WORST (minimax) possible move the monster can make (Smallest A* to monster)
                f_values_prime[j].append(f_direction)
                f_values_prime[j].append(f_bomb_x)
                f_values_prime[j].append(f_bomb_y)
                f_values_prime[j].append(moves[j][2])
                f_values_prime[j].append(f_monster_direction)
                f_values_prime[j].append(f_closest_to_exit)

                q_sa_prime[j] = self.q_function(f_values_prime[j])

        """
        Now we maximize q_sa_prime
        """
        q_sa_prime_max = max(q_sa_prime)
        """
        And now here's our rewards
        """
        reward = self.identify_rewards(wrld,[action[0],action[1]],action[2])
        # print(reward)
        # TODO : Delta function here
        # print("Reward : %d" %reward)
        # print("max_q_sa : %f" % max_q_sa)
        # print("gamma q_sa_prime_max : %f" % (gamma*q_sa_prime_max))
        delta = reward + gamma*q_sa_prime_max - max_q_sa
        print(delta)
        for i in range(len(f_values[index_best])):
            self.weights[i] += alpha*delta*f_values[index_best][i] #update the weights

        print(self.weights)
        if action[2] == 1:
            self.place_bomb()
        #print("Action Selected")
        self.move(action[0] - self.x, action[1] - self.y)
        print([action[0], action[1], action[2]])
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

    def identify_rewards(self,wrld,next_position, bomb):
        reward = self.cost_of_living #cost of living
        # If we predict our character dying (within range of monster or bomb has 1 left and either x and y distance from bomb is 0
        monsters_position = self.find_monsters(wrld)

        for monster in monsters_position:
            dist_to_monster = eights_distance(next_position[0],next_position[1],monster[0],monster[1])
            if dist_to_monster <= 1:
                reward += self.death # we committed Foisie jump

        if bomb == 1:
            reward += self.place_bomb_reward

        # If the bomb blows up and breaks a wall, add 10
        for bomb in self.find_bomb(wrld):
            for i in range(4):
                if wrld.monsters_at(min(max(0,bomb[0] + i),wrld.width()-1),bomb[1]):
                    reward += self.monster_kill
                if wrld.wall_at(min(max(0, bomb[0] + i), wrld.width()-1), bomb[1]):
                    reward += self.wall_demo
                if next_position[0] == min(max(0, bomb[0] + i), wrld.width()-1) and next_position[1] == bomb[1]:
                    reward += self.bomb_death
                if wrld.monsters_at(min(max(0,bomb[0] - i),wrld.width()-1),bomb[1]):
                    reward += self.monster_kill
                if wrld.wall_at(min(max(0, bomb[0] - i), wrld.width()-1), bomb[1]):
                    reward += self.wall_demo
                if next_position[0] == min(max(0, bomb[0] - i), wrld.width()-1) and next_position[1] == bomb[1]:
                    reward += self.bomb_death
                if wrld.monsters_at(bomb[0],min(max(0,bomb[1] + i),wrld.height()-1)):
                    reward += self.monster_kill
                if wrld.wall_at(bomb[0], min(max(0,bomb[1] + i),wrld.height()-1)):
                    reward += self.wall_demo
                if next_position[1] == min(max(0, bomb[1] + i), wrld.height()-1) and next_position[0] == bomb[0]:
                    reward += self.bomb_death
                if wrld.monsters_at(bomb[0],min(max(0,bomb[1] - i),wrld.height()-1)):
                    reward += self.monster_kill
                if wrld.wall_at(bomb[0], min(max(0,bomb[1] - i),wrld.height()-1)):
                    reward += self.bomb_death
                if next_position[1] == min(max(0, bomb[1] - i), wrld.height()-1) and next_position[0] == bomb[0]:
                    reward += self.bomb_death
        for explosion in self.find_explosions(wrld):
            if explosion[0] == next_position[0] and explosion[1] == next_position[1]:
                reward += self.bomb_death

        (exit, exit_x, exit_y) = self.find_exit(wrld)
        if next_position[0] == exit_x and next_position[1] == exit_y:
            reward += self.reward_win

        return reward

        # TODO : A* for a certain target in a given world from a certain position
    def find_close_to_exit(self, wrld, startingx, startingy, targetx, targety):
        start = (startingx, startingy)
        goal = (targetx, targety)
        frontier = PriorityQueue()
        frontier.put(start, 0)
        cost = {}
        cost[start] = 0
        closest_cell = (startingx,startingy)
        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            neighbors = self.look_for_empty_cell_monster(wrld, current[0], current[1])
            for n in neighbors:
                n_cost = self.get_hypotnuse(current, n)
                new_cost = cost[current] + 1
                if n not in cost or new_cost < cost[n]:
                    cost[n] = new_cost
                    priority = new_cost + n_cost
                    frontier.put(n, priority)
                if self.get_hypotnuse(current, goal) <= self.get_hypotnuse(closest_cell, goal):
                    closest_cell = current

        return closest_cell




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
            return []



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
            monster_locations.append([monster[0].x, monster[0].y])
        return monster_locations

    def find_bomb(self,wrld):
        bomb_locations = []
        bombs = wrld.bombs.values()
        for bomb in bombs:
            bomb_locations.append((bomb.x, bomb.y))
        return bomb_locations
    def find_explosions(self,wrld):
        explosion_locations = []
        explosions = wrld.explosions.values()
        for explosion in explosions:
            explosion_locations.append((explosion.x, explosion.y))
        return explosion_locations

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

