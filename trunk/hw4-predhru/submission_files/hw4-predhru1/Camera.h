#include "nv/nv_math.h"
#include "nv/nv_algebra.h"
#include <math.h>
#include "SceneObjs.h"
#include <stdlib.h>

#define WIDTH 640
#define HEIGHT 480

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
	vec3 getEye(){return eye;}
	Ray ray;
	void generateRays(Scene *scene);
};

