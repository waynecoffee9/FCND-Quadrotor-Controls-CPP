import math;

def fmt(value):
	return "%.3f" % value

period = 5
radius = 1.5
timestep = 0.02
maxtime = period*1
brake_timestep = 10
counter = brake_timestep
z = -1

with open('HelixFF.txt', 'w') as the_file:
	t=0;
	px = 0;
	py = 0;
	pz = 0;
	while t <= maxtime:
		x = math.sin(t * 2 * math.pi / period) * radius;
		y = math.cos(t * 2 * math.pi / period) * radius; 
		the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z));
		if t + timestep * brake_timestep >= maxtime:
			counter -= 1
		vx = (x - px)/timestep*counter/brake_timestep;
		vy = (y - py)/timestep*counter/brake_timestep;
		vz = (z - pz)/timestep*counter/brake_timestep;
		px = x;
		py = y;
		pz = z;
		the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + fmt(vz));
		heading = math.atan2(vy, vx);
		the_file.write("," + fmt(heading));
		the_file.write("\n");
		t += timestep;
		z -= 0.01 
