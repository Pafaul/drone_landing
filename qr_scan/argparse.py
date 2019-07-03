import argparse

 
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", required=True)
args = vars(ap.parse_args())

for arg in args["input"]:
	print(arg)

