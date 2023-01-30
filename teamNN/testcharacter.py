# This is necessary to find the main code
import sys
from queue import PriorityQueue
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
import math
from colorama import Fore, Back

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here
        pass
<<<<<<< Updated upstream
=======

    # TODO : A* for a certain target in a given world from a certain position
    def a_star(self,wrld,startingx,startingy,targetx,targety):
        start = (startingx, startingy)
        goal = (targetx, targety)
        frontier = PriorityQueue(start,0)
        came_from = {}
        cost = {}
        came_from[start] = None
        cost[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            neighbors = self.look_for_empty_cell(wrld)
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

        # Pick best whatever
        return (dx,dy)
    def variant1_do(self,wrld):
        # TODO : Find Exit
        (exit_x,exit_y) = (0,0)
        (dx,dy) = self.a_star(wrld,)
        self.move(dx,dy)
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
        return math.hypot(abs(cell1(0)-cell2(0)), abs(cell1(1)-cell2(1)))

>>>>>>> Stashed changes
