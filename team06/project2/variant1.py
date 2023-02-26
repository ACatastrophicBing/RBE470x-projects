# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game

# TODO This is your code!
sys.path.insert(1, '../team06')
from testcharacter import TestCharacter


# Create the game
g = Game.fromfile('map.txt')

lil_Johnny = TestCharacter("me", "C", 0, 0)
# TODO Add your character
g.add_character(lil_Johnny)

# Run!
iterations = 1000

for i in range(iterations):
    g.go()

# Save this characters info :
lil_Johnny.save_to_CSV()