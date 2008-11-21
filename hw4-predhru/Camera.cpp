#include "Scene.h"
#define pi 3.1428571

vec3 CameraRay::calcRays(int i, int j)
{
	vec3 ray;
	float A;
	float B;

	float fovy=fov;
	float fovx=fovy*float(float(WIDTH)/float(HEIGHT));
	
	A = tanf((pi / 180) * (fovx/2.0)) * ((j - (WIDTH/2.0))/(WIDTH/2.0));
	B = tanf((pi / 180) * (fovy/2.0)) * ((HEIGHT/2.0) - i)/ (HEIGHT/2.0);

	ray = (A * u) + (B * v) - w;
	ray.normalize();

	return ray;
}
