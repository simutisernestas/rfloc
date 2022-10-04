import pickle

with (open("data/123.0.pkl", "rb")) as openfile:
    measurements = pickle.load(openfile)
    print(measurements)