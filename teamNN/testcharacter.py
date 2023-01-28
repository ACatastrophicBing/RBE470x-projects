# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
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
        # Your code here
        pass

    # TODO : A* for a certain target in a given world from a certain position
    def a_star(self,wrld,startingx,startingy,targetx,targety):
        """
        Takes in a starting and ending position and returns our next best move, and the length it will take us to reach there
        :param wrld: the world we are in
        :param startingx, startingy: where we are starting from
        :param targetx, targety: where we want to go
        """
        dx = 0
        dy = 0
        path_length = 0
        # Pick best whatever
        return (dx,dy,path_length)

    def variant1_do(self,wrld):

        (exit, exit_x, exit_y) = self.find_exit(wrld)
        if exit:
            # do A*
            (dx, dy) = self.a_star(wrld, )
            self.move(dx, dy)
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

    def weigh_moves(self,wrld,moves):
        dists_from_goal = [None] * len(moves)
        dists_from_monsters = [None] * len(moves)
        (exit,exit_x,exit_y) = self.find_exit(wrld)
        monsters_at = self.find_monsters(wrld)
        #look for empty cells around monster to generate possible monster moves


        max_path_length = 0
        for move in moves:
            (dx, dy, path_length) = self.a_star(wrld, move[0], move[1], exit_x, exit_y)
            if max_path_length < path_length: max_path_length = path_length
            dists_from_goal[moves.index(move)] = path_length

            min_monster_dist = 10

            for monster in monsters_at: # TODO : change to using all possible monster positions
                (dx, dy, dist_from_monster) = self.a_star(wrld, move[0], move[1], monster[0], monster[1])
                if min_monster_dist > dist_from_monster: min_monster_dist = dist_from_monster

                # TODO : Now guess what monster will do and weight the function with that, or create 2d array?

            # Might need a dist from bomb, but not yet
            dists_from_monsters[moves.index(move)] = min_monster_dist


        # TODO : Using stuff from above, use equation below to weigh the viable neighbors
        # (max_distance - distance + 1) * (distance_from_monster - 1)
        move_utilities = []
        for i in range(len(moves)):
            # TODO : Add a weight to each move, probably with a new array
            utility = (max_path_length - dists_from_goal[i] + 1) * (dists_from_monsters[i] -1)
            move_utilities.append(moves[i].x, moves[i].y, utility)

        return move_utilities

    def generate_chance_node(self):

    def find_exit(self,wrld):
        for x in range(wrld.height):
            for y in range(wrld.width):
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
