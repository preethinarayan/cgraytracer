#include "StdAfx.h"
#include "Camera.h"

bool Camera::setArgs(char **args)
{
	if( !args )
		return false;

	for(int i=0; i<10; i++)
	{
		if(!args[i])
			return false;
	}

	eye = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
	center = vec3((float)atof(args[3]),(float)atof(args[4]),(float)atof(args[5]));
	up = vec3((float)atof(args[6]),(float)atof(args[7]),(float)atof(args[8]));

	fov = atof(args[9]);

	initCoordinates();

	return true;
}

/* calc 'u', 'v', 'w' given eye, center and up */
void Camera::initCoordinates()
{
	w = eye - center;
	w.normalize();
	u = cross(u,up,w);
	u.normalize();
	v = cross(v,w,u);
}

vec3 Camera::calcRays(int i, int j)
{
	vec3 delta,ray;
	float alpha;
	float beta;

	float fovy=fov;
	float fovx=fovy*float(float(width)/float(height));

	//float fovx=fov;
	//float fovy=fovx*width/height;
	
	alpha = tanf(degToRad * (fovx/2.0)) * ((j - (width/2.0))/(width/2.0));
	beta = tanf(degToRad * (fovy/2.0)) * ((height/2.0) - i)/ (height/2.0);

	delta = (alpha * u) + (beta * v) - w;
	delta.normalize();
	ray = delta;

	return ray;

}

void Camera::setImageParameters(int width, int height)
{
	this->width = width;
	this->height = height;
}

Camera::Camera(void)
{
}

Camera::~Camera(void)
{
}
