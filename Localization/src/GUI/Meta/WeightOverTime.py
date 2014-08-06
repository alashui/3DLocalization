from __future__ import division
from graphics import *

def main():
    f = open('MetaData.txt')
    avgwt = []
    for line in f:
        line = line.split()
        avgwt += [float(line[1])]

    f.close()

    sl = 40 # shift left
    sd = 20  # shift down

    win = GraphWin('Weight Over time', 600+sl, 600 + sd)
    win.setBackground("white")

    t = Text(Point((600+sl)/2, 10), "Average Weight Over Time")
    t.draw(win)

    t = Text(Point(sl/2 + 10, 300 + sd), "Avg\nWeight")
    t.draw(win)

    l = Line(Point(20+sl, 20+sd), Point(20+sl,650+sd))
    l.setArrow("first")
    l.draw(win)

    l = Line(Point(0+sl, 580+sd), Point(580+sl,580+sd))
    l.setArrow("last")
    l.draw(win)

    l = Line(Point(15+sl, 300+sd), Point(25+sl, 300+sd))
    l.draw(win)

    t = Text(Point(40+sl, 300+sd), str(round((max(avgwt) + min(avgwt))/2, 1)))
    t.setTextColor("blue")
    t.draw(win)

    t = Text(Point(35+sl, 570+sd), str(round(min(avgwt), 1)))
    t.setTextColor("blue")
    t.draw(win)

    t = Text(Point(40+sl, 20+sd), str(round(max(avgwt), 1)))
    t.setTextColor("blue")
    t.draw(win)

    mulx = 560 / len(avgwt)

    for i in range(len(avgwt) - 1):
        pt1 = Point(20+sl+i * mulx, sd+580-transform(avgwt[i], min(avgwt), max(avgwt), 560))
        pt2 = Point(20+sl+(i+1) * mulx, sd+580-transform(avgwt[i + 1], min(avgwt), max(avgwt), 560))
        l = Line(pt1, pt2)
        l.draw(win)

    win.getMouse()

def transform(x, minx, maxx, newmax):
    return (x - minx) * newmax / (maxx - minx)

def round(num, p):
    num = num * 10 ** p
    return int(num + 0.5) / 10 ** p

if __name__ == "__main__": main()