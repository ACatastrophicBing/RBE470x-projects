# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster
from monsters.stupid_monster import StupidMonster
import csv

# TODO This is your code!
sys.path.insert(1, '../teamNN')
from testcharacter import TestCharacter

def read_from_csv():  # returns a list of numbers saved in weights.csv
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

def save_to_csv(weights): #saves a list of numbers to weights.csv
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

# Create the game
random.seed(123) # TODO Change this if you want different random choices

# TODO Add your character
weights = read_from_csv()
# Run!
iterations = 1000
for i in range(iterations):
    weights = read_from_csv()
    lil_Johnny = TestCharacter("me", "C", 0, 0, weights)
    Chadler = SelfPreservingMonster("death", "S", 3, 2, 2)
    hWang = StupidMonster("stupid", "S", 3, 9)
    g = Game.fromfile('map.txt')
    g.add_character(lil_Johnny)
    g.add_monster(Chadler)
    g.add_monster(hWang)
    g.go(1)
    print(lil_Johnny.weights)
    weights = lil_Johnny.weights
    save_to_csv(weights)