#ifndef COORD_H
#define COORD_H

struct coord {
	double x;
	double y;
	double z;

	coord operator-(const coord& sub)
	{
		return {x-sub.x, y-sub.y, z-sub.z};
	}
};

#endif
