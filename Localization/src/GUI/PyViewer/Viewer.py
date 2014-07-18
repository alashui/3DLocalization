# Viewer.py

from graphics import *

from time import sleep

def main():
	f = open('ParticleLists.txt')

	particles = []

	minx = 200
	maxx = 0
	miny = 200
	maxy = 0

	maxw = 0

	for line in f:
		oldp = line.split()
		p = []
		for i in oldp:
			p += [float(i) * 50]
		p.pop(2)
		p.pop(4)
		particles += [p]

		if p[0] < minx:
			minx = p[0]
		elif p[0] > maxx:
			maxx = p[0]
		if p[1] < miny:
			miny = p[1]
		elif p[1] > maxy: 
			maxy = p[1]
		if p[4] > maxw:
			maxw = p[4]

	f.close()

	w = maxx - minx
	h = maxy - miny

	mulw = 255 / maxw;

	win = GraphWin('Points', 570, 570)

	x = 10+(particles[0][0] - minx) * 550/w 
	y = 10+(particles[0][1] - miny) * 550/h
	c = Circle(Point(x, y), 6)
	c.setFill("green")
	c.draw(win)
	dx = 10 + ( particles[0][0] + particles[0][2] * 0.2 - minx) * 550 / w
	dy = 10 + ( particles[0][1] + particles[0][3] * 0.2 - miny) * 550 / h
	l = Line(Point(x, y), Point(dx, dy))
	l.setArrow("last")
	l.draw(win)
	
	particles.pop(0)

	for point in particles:
		x = 10+(point[0] - minx) * 550/w 
		y = 10+(point[1] - miny) * 550/h
		dx = 10 + ( point[0] + point[2] * 0.2 - minx) * 550 / w
		dy = 10 + ( point[1] + point[3] * 0.2 - miny) * 550 / h

		wt = point[4]

		c = Circle(Point(x, y), 4)
		c.setFill(color_rgb(wt*mulw, 0, 255 - wt*mulw))
		c.draw(win)
		l = Line(Point(x, y), Point(dx, dy))
		l.setArrow("last")
		l.draw(win)
		# print "Point", x - minx, y - miny

	win.getMouse()
	# sleep(5)
	win.close()

	return main()

if __name__ == "__main__": main()