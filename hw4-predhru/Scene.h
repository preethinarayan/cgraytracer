#include "nv/nv_math.h"
#include "nv/nv_algebra.h"
#include<math.h>
#include<stdlib.h>

#define WIDTH 600
#define HEIGHT 400

//void traceScene();

class CameraRay
{
private:
	vec3 u,v,w;
	vec3 eye;
	vec3 center;
	vec3 up;
	float fov;

public:
	CameraRay();
	CameraRay(vec3& eye, vec3& center, vec3& up, float _fov);
	vec3 calcRays(int i,int j);
	vec3 getEye(){return eye;}
};
//CameraRay *camera;

class Ray
{
public:
	vec3 begin; 
	vec3 end;
	Ray();
};

class Scene
{
private:

public:
Ray ray;
void generateRays(CameraRay *camera);


};
//
//class Plane
//{
//private:
//	vec3 normal;
//	float d;
//public:
//	Plane()
//	{
//		normal = vec3(0,0,0);
//		d = 0.0;
//	}
//	Plane(vec3& _normal,float dist)
//	{
//		normal = vec3(_normal);
//		d = dist;
//	}
//	vec3 getNormal()
//	{
//		return normal;
//	}
//	float getd()
//	{
//		return d;
//	}
//};