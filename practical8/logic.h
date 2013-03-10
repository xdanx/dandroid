#ifndef LOGIC_H
#define LOGIC_H

#include "space.h"

int get_cubicle_index(int left, int right) {
	bool left_big = left > LEFT_RIGHT_THRESH;
	bool right_big = right > LEFT_RIGHT_THRESH;
	if (left_big && right_big) return 2;
	if (!left_big && right_big) return 3;
	if (left_big && !right_big) return 1;
	writeDebugStream("ERROR: should not reach this code!");
	return 1337;
}

#endif
