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
        dx = 0
        dy = 0
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
