#include "nv/nv_math.h"
#include "nv/nv_algebra.h"
#include<math.h>
#include<stdlib.h>

class CameraRay
{
private:
	vec3 origin;
	vec3 direction;
public:
	CameraRay()
	{
		origin = vec3(0,0,0);
		direction = vec3(0,0,0);
	}
	CameraRay(vec3& _origin,vec3& _direction)
	{
		origin = vec3(_origin);
		direction = vec3(_direction);
	}
};


class Plane
{
private:
	vec3 normal;
	float d;
public:
	Plane()
	{
		normal = vec3(0,0,0);
		d = 0.0;
	}
	Plane(vec3& _normal,float dist)
	{
		normal = vec3(_normal);
		d = dist;
	}
	vec3 getNormal()
	{
		return normal;
	}
	float getd()
	{
		return d;
	}
};