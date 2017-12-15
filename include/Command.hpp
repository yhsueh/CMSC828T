#pragma once
#include "ardrone_autonomy/Navdata.h"
class Command {
public:
	Command(): rotz(0), state(0){}
	void stateCallback(const ardrone_autonomy::Navdata &msg);
	bool rotReset() {lrotz = rotz;}
	int diffRot() {return rotz-lrotz;}
	int getState() {return state;}
	float getRotz() {return rotz;}
private:
	float rotz, lrotz;
	int state;
};
