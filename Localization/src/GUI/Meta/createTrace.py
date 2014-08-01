from __future__ import division
from graphics import *
from time import sleep

def main():
    f = open('MetaData.txt')
    avgwt = []
    pos = []
    for line in f:
        line = line.split()
        pos += [[float(line[2]), float(line[3])]]
    f.close()
    
    [minx, maxx] = AnalyzeListOfList(pos, 0)
    [miny, maxy] = AnalyzeListOfList(pos, 1)

    win = GraphWin('Points', 600, 600)
    win.setBackground("white")

    l = Line(Point(20, 20), Point(20,650))
    l.setArrow("first")
    l.draw(win)

    l = Line(Point(0, 580), Point(580,580))
    l.setArrow("last")
    l.draw(win)

    l = Line(Point(15, 300), Point(25, 300))
    l.draw(win)

    l = Line(Point(300, 575), Point(300, 585))
    l.draw(win)

    t = Text(Point(40, 300), str(round((maxy + miny)/2, 1)))
    t.setTextColor("blue")
    t.draw(win)

    t = Text(Point(35, 570), str(round(miny, 1)))
    t.setTextColor("blue")
    t.draw(win)

    t = Text(Point(40, 20), str(round(maxy, 1)))
    t.setTextColor("blue")
    t.draw(win)

    t = Text(Point(300, 590), str(round((maxx + minx)/2, 1)))
    t.setTextColor("orange")
    t.draw(win)

    t = Text(Point(35, 590), str(round(minx, 1)))
    t.setTextColor("orange")
    t.draw(win)

    t = Text(Point(580, 590), str(round(maxx, 1)))
    t.setTextColor("orange")
    t.draw(win)

    circ = Circle(Point(20 + transform(pos[0][0], minx, maxx, 560), \
            580-transform(pos[0][1], miny, maxy, 560)), 10)
    circ.setFill('red')
    circ.draw(win)

    NUMTIMES = 10

    for i in range(len(pos) - 1):
        win.delItem(l)
        pt1 = Point(20 + transform(pos[i][0], minx, maxx, 560), \
            580-transform(pos[i][1], miny, maxy, 560))
        pt2 = Point(20 + transform(pos[i+1][0], minx, maxx, 560), \
            580-transform(pos[i+1][1], miny, maxy, 560))
        l = Line(pt1, pt2)
        # l.setArrow("last")
        l.draw(win)

        for i in range(NUMTIMES):
            circ.move((pt2.x - pt1.x)/NUMTIMES, (pt2.y - pt1.y)/NUMTIMES)
            sleep(0.05/NUMTIMES)

    win.getMouse()

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

def round(num, p):
    num = num * 10 ** p
    return int(num + 0.5) / 10 ** p

if __name__ == "__main__": main()