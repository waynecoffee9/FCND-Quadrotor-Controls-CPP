import math;

def fmt(value):
	return "%.3f" % value

period = 4
radius = 1.5
timestep = 0.02
maxtime = period*1
brake_timestep = 20
counter = brake_timestep
with open('CircleFF.txt', 'w') as the_file:
	t=0;
	px = 0;
	py = 0;
	pz = 0;
	while t <= maxtime:
		x = math.sin(t * 2 * math.pi / period) * radius;
		y = math.cos(t * 2 * math.pi / period) * radius;
		the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + "-1");
		vx = 0;
		vy = 0;
		if t + timestep * brake_timestep >= maxtime:
			counter -= 1
		vx = (x - px)/timestep*counter/brake_timestep;
		vy = (y - py)/timestep*counter/brake_timestep;
		px = x;
		py = y;
		the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + "0");
		heading = math.atan2(vy, vx);
		the_file.write("," + fmt(heading));
		the_file.write("\n");
		t += timestep;
			
