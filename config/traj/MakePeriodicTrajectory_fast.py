import math;

def fmt(value):
	return "%.3f" % value

period = [4, 2, 4]
radius = 1.5
timestep = 0.02
maxtime = max(period)*3
timemult = [1, 1, 1]
phase=[0,0,0]
amp = [1,0.4,.5]
center = [0, 0, -2]
phase2=[0,0,0]
with open('FigureEightFF.txt', 'w') as the_file:
	t=0;
	t2=0;
	px = 0;
	py = 0;
	pz = 0;
	speed = 1.13;
	while t2 <= maxtime:
		x = math.sin(t * 2 * math.pi / period[0]*speed + phase[0]) * radius * amp[0] + center[0];
		y = math.sin(t * 2 * math.pi / period[1]*speed + phase[1]) * radius * amp[1] + center[1];
		z = math.sin(t * 2 * math.pi / period[2]*speed + phase[2]) * radius * amp[2] + center[2];
		
		if (y < 0):
			if (abs(x) < 1.1 and abs(x) > 0.0):
				t += timestep*0.08
		if (y >=0):
			abs_x = min(0.5, abs(x))
			delta = (0.5-abs_x)*0.16
			t += timestep*delta
		

		the_file.write(fmt(t2) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z));
		vx = 0;
		vy = 0;
		vz = 0;
		######## BEGIN STUDENT CODE
		vx = (x - px)/timestep;
		vy = (y - py)/timestep;
		vz = (z - pz)/timestep;
		px = x;
		py = y;
		pz = z;
		if t2 == 0:
			vz = 0

		######## END STUDENT CODE
		the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + fmt(vz));
		heading = math.atan2(vy, vx);
		the_file.write("," + fmt(heading));
		######## EXAMPLE SOLUTION
		#the_file.write("," + fmt((x-px)/timestep) + "," + fmt((y-py)/timestep) + "," + fmt((z-pz)/timestep));
		#px = x;
		#py = y;
		#pz = z;
		######## END EXAMPLE SOLUTION
		
		the_file.write("\n");
		
		t += timestep;
		t2 += timestep;
			
