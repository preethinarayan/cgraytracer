#include "nv/nv_math.h"
#include "nv/nv_algebra.h"
#include<math.h>
#include<stdlib.h>

#define WIDTH 640
#define HEIGHT 480

//void traceScene();

class CameraRay
{
private:
	vec3 u,v,w;
	
	float fov;

public:
	vec3 eye;
	vec3 center;
	vec3 up;
	CameraRay();
	CameraRay(vec3 eye, vec3 center, vec3 up, float _fov);
	vec3 calcRays(int i,int j);
	vec3 getEye(){return eye;};
};
//CameraRay *camera;

class Ray
{
public:
	vec3 begin; 
	vec3 end;
	Ray();
};

class Quadrilateral
{
	private:
		vec3 vertices[4];
	public:
	Quadrilateral();
	Quadrilateral(vec3& v_00,vec3& v_10,vec3& v_11, vec3& v_01);	
	 vec3& v_00()  { return vertices[0]; }
	 vec3& v_10()  { return vertices[1]; }
	 vec3& v_11()  { return vertices[2]; }
	 vec3& v_01()  { return vertices[3]; }
	 bool intersect_ray(Quadrilateral& q,Ray& r, float& u, float& v, float& t);
};

class Triangle
{
private:
	vec3 vertices[3];
public:
	Triangle();
	Triangle(vec3 v1, vec3 v2, vec3 v3);
	vec3 getVertex1(){return vertices[0];};
	vec3 getVertex2(){return vertices[1];};
	vec3 getVertex3(){return vertices[2];};
	bool intersect_ray(Triangle& tr, Ray& ray, float& u, float& v, float& t);
};

class Sphere
{
private:
	float rad;
	vec3 center;

public:
	Sphere();
	Sphere(vec3 _center,float _rad);
	vec3 getCenter(){return center;};
	float getRadius(){return rad;};
	bool intersect_ray(Sphere& sphere, Ray& r, float& t);
};


class Scene
{
private:

public:
Ray ray;
//vec3 subt(vec3 v1,vec3 v2);
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