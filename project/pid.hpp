#pragma once
#include <string>
#include <stdio.h>
#include <iostream>

using namespace std;

class Pid
{
public:
	string name;
	float p,i,d;
	int kp, ki, kd;
	float last = 0;

	Pid() {};

	Pid(int kp, int ki, int kd, string name = ""): name{name}, kp{kp}, ki{ki}, kd{kd}
			{p=0;i=0;d=0;}

	void update(float deltaTine, float delta);
	float calc();
};
