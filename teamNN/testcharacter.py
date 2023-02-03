# This is necessary to find the main code
import sys
from queue import PriorityQueue
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
import math
from colorama import Fore, Back

def find_next(wrld):
    x = 0
    y = 0
    # Weigh each node based on A* to goal and distance from enemy, if tile we are trying to go to is within 1 of enemy, make it 0?
    return (x,y)


class TestCharacter(CharacterEntity):

    def __init__(self,name,avatar,x,y):
        super().__init__(name, avatar, x, y)

    def do(self, wrld):
        self.variant1_do(wrld)
        pass

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
        current_cell = goal
        while not current_cell == None:
            path.append(current_cell)
            current_cell = came_from[current_cell]

        path = reversed(path)
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

    def variant1_do(self,wrld):

        (exit, exit_x, exit_y) = self.find_exit(wrld)
        if exit:
            path = self.a_star(wrld,self.x, self.y, exit_x, exit_y)
            for step in path:
                self.move(step(0) - self.x , step(1) - self.y)
        else:
            print("oh fuck there's no exit")
        pass

    def variant2_do(self,wrld):
        (enemy,ex,ey) = self.within_range(3,wrld)
        if enemy:
            # do expectimax using ex and ey for whatever is around
            pass
        else:
            # do A*
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
                for dy in range(-self.rnge, self.rnge + 1):
                    # Avoid out-of-bounds access
                    if ((self.y + dy >= 0) and (self.y + dy < wrld.height())):
                        # Is a character at this position?
                        if (wrld.characters_at(self.x + dx, self.y + dy)):
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
            (dx, dy, dist_from_monster) = self.a_star(wrld, move_x, move_y, mmove[1], mmove[2])
            if(dist_from_monster <= 2): # is this 1 or 2
                value -= 2 * mmove[0] # TODO : Fine tune this value
            value += dist_from_monster * mmove[0]
        return value

    def weigh_moves(self,wrld):
        moves = self.look_for_empty_cell_character(wrld)
        dists_from_goal = [None] * len(moves)
        monster_chance_values = [None] * len(moves)
        (exit,exit_x,exit_y) = self.find_exit(wrld)
        monsters_at = self.find_monsters(wrld)

        #look for empty cells around monster to generate possible monster moves
        probability_moves = [[]] * len(monsters_at)
        for monster in monsters_at:
            monster_moves = self.look_for_empty_cell_monster(wrld,monster.x,monster.y)
            for move in monster_moves: # 1 / len(monster_moves)
                probability_moves[len(monsters_at)][move] = (1/len(monster_moves),move[0],move[1])
        # Set probability of monster doing that move

        max_path_length = 0
        for move in moves:
            index = moves.index(move)

            (dx, dy, path_length) = self.a_star(wrld, move[0], move[1], exit_x, exit_y)
            if max_path_length < path_length: max_path_length = path_length
            dists_from_goal[index] = path_length
            monster_chance_values[index] = self.chance_monster_value(wrld,monster_moves,move[0],move[1])



        # TODO : Using stuff from above, use equation below to weigh the viable neighbors
        # (max_distance - distance + 1) * (distance_from_monster - 1)
        move_utilities = []
        for i in range(len(moves)):
            # TODO : This definitely does not work, maybe have an external weight if we are super close to monter?
            big_scalar = 10
            utility = (max_path_length - dists_from_goal[i] + 1) * (monster_chance_values[i] * big_scalar)
            move_utilities.append(moves[i].x, moves[i].y, utility)

        return move_utilities

    def find_exit(self,wrld):
        for x in range(len(wrld.grid)):
            for y in range(len(wrld.grid[0])):
                if wrld.exit_at(x, y):
                    return (True, x, y)
        return (False, 0, 0)

    def find_monsters(self,wrld):
        monsters = []
        for x in range(wrld.height):
            for y in range(wrld.width):
                if wrld.monsters_at(x, y):
                     monsters.append((x, y))
        return monsters

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
                            cells.append((dx, dy))
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
                            cells.append((dx, dy))
        # All done
        return cells

