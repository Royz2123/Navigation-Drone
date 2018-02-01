#include "pid.hpp"

float Pid::calc(){
	return p*kp/500.0+i*ki/10000.0+d*kd/10000.0;
}

void Pid::update(float deltaTime,float delta){
	//std::cout << "Delta =" << delta<< " |  DeltaTime =" << deltaTime << std::endl;
	p = delta;
	i += deltaTime * (last+delta)/2;
	d = (p-last)/deltaTime;
	int maxI = 1000;
	if (i > maxI) i = maxI;
	else if (i < -maxI) i = -maxI;

	last = p;
}
