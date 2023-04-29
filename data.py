import pickle

# read the data from the file

with open('data.pkl', 'rb') as f:
    arr = pickle.load(f)

with open('data1.pkl', 'rb') as q:
    arr2 = pickle.load(q)

