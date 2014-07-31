from __future__ import division
from turtle import *

def main():
	f = open('MetaData.txt')
	avgwt = []
	pos = []
	for line in f:
		line = line.split()
		avgwt += [float(line[1])]
		pos += [[float(line[2]), float(line[2])]]

	[minx, maxx] = AnalyzeListOfList(pos, 0)
	[miny, maxy] = AnalyzeListOfList(pos, 1)

	wn = Screen()
	wn.title("Trace")

	t = Turtle()
	t.speed(0)
	for p in pos:
		t.goto(transform(p[0], minx, maxx, 500), 500 - transform(p[1], miny, maxy, 500))

	wn.exitonclick()

def AnalyzeListOfList(L, i):
	minimum = 10000
	maximum = -10000

	for l in L:
		if l[i] < minimum:
			minimum = l[i]
		elif l[i] > maximum:
			maximum = l[i]
	return [minimum, maximum]

def transform(x, minx, maxx, newmax):
	return (x - minx) * newmax / (maxx - minx)

if __name__ == "__main__": main()