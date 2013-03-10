#ifndef _SPACE_H
#define _SPACE_H
// stuff related to space and localization

typedef struct {int x; int y;} Point;

/* 360 version - adding positibe values means rotating left, adding negative ones means rotating right */
float add_to_angle(float original, float modifier) {
	int newAngle = original + modifier;
	if (newAngle < 0) return 360 + newAngle;
	if (newAngle > 360) return newAngle - 360;
	return newAngle;
}

// write down the map and shit here and all distance constants

#endif
