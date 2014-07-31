# Viewer.py
from __future__ import division

from graphics import *

def main():
    f = open('MetaData.txt')
    avgwt = []
    for line in f:
        line = line.split()
        avgwt += [float(line[1])]

    f.close()

    win = GraphWin('Points', 600, 600)

    l = Line(Point(20, 20), Point(20,650))
    l.setArrow("first")
    l.draw(win)

    l = Line(Point(0, 580), Point(580,580))
    l.setArrow("last")
    l.draw(win)

    l = Line(Point(15, 300), Point(25, 300))
    l.draw(win)

    t = Text(Point(40, 300), str(round(sum(avgwt)/len(avgwt), 2)))
    t.draw(win)

    mulx = 560 / len(avgwt)

    for i in range(len(avgwt) - 1):
        pt1 = Point(20+i * mulx, 580-transform(avgwt[i], min(avgwt), max(avgwt), 560))
        pt2 = Point(20+(i+1) * mulx, 580-transform(avgwt[i + 1], min(avgwt), max(avgwt), 560))
        l = Line(pt1, pt2)
        l.draw(win)

    win.getMouse()

def transform(x, minx, maxx, newmax):
    return (x - minx) * newmax / (maxx - minx)

def round(num, p):
    num = num * 10 ** p
    return int(num) / 10 ** p

if __name__ == "__main__": main()