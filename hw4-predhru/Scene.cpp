#include "Scene.h"

CameraRay::CameraRay()
	{
		u = vec3(0,0,0);
		w = vec3(0,0,0);
		v = vec3(0,0,0);
		fov = 0.0;
	}
CameraRay::CameraRay(vec3& eye, vec3& center, vec3& up, float _fov)
	{
		w = eye - center;
		w.normalize();
		u = cross(u,up,w);
		u.normalize();
		v = cross(v,w,u);
		fov = _fov;
	}
Ray::Ray()
{
	begin = vec3(0,0,0);
	end = vec3(0,0,0);
}

void Scene::generateRays()
{
	vec3 pt;
	.P0 = Cam->getEye();
	printf("eye: %f, %f, %f\n", myR.P0.x, myR.P0.y, myR.P0.z); 

	float remainingtime = 0;
	float totaltime = I->getHeight() + I->getWidth();
	float percentremaining;
	for(int i=0; i<I->getHeight(); i++)
	{
		//printf("\nTracing Row %d", i);
		for(int j=0; j<I->getWidth(); j++)
		{
			vec3 color(0.0, 0.0, 0.0);
			myR.P1 = Cam->calcRays(i,j);
			color = getRayIntersection(&myR, RECURSION_DEPTH, false);
			/*if(Intersection(&myR, &pt))
			{ 
				color = FindColor(pt, RECURSION_DEPTH);
			}*/
			I->setPixel(i, j, color.x, color.y, color.z);
			//remainingtime += 1;
			//percentremaining = (remainingtime / totaltime) * 100;
			//system("cls");
			//printf("Progress: %0.3f", percentremaining); 
		}
	}
}