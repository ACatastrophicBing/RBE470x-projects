# This is necessary to find the main code
import sys
import csv
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game

# TODO This is your code!
sys.path.insert(1, '../team06')
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

# TODO Add your character
weights = read_from_csv()
# Run!
iterations = 1000
for i in range(iterations):
    lil_Johnny = TestCharacter("me", "C", 0, 0, weights)
    g = Game.fromfile('map.txt')
    g.add_character(lil_Johnny)
    g.go(1)
    print(lil_Johnny.weights)
    weights = lil_Johnny.weights

# Save this characters info :
save_to_csv(weights)
