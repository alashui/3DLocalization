# Viewer.py

from graphics import *

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

	if w == 0:
		w = 550
	if h == 0:
		h = 550

	mulx = 500/w 
	muly = 500/h
	mulw = 255 / maxw

	win = GraphWin('Points', 570, 570)

	# colors = {1:"blue", 2:"light blue"}

	for point in particles:
		x = point[0] * mulx
		y = point[1] * muly

		wt = point[4]

		c = Circle(Point(x - minx, y - miny), 4)
		c.setFill(color_rgb(wt*mulw, 0, 255 - wt*mulw))
		c.draw(win)
		print "Point", x - minx, y - miny

	win.getMouse()
	win.close()

	return main()

if __name__ == "__main__": main()