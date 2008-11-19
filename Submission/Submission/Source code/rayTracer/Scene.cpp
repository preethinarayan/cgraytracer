#include "StdAfx.h"
#include "Scene.h"

bool Scene::addObject(SceneObjects *obj)
{
	if(!obj)
		return false;
	objs.push_back(obj);
	nObjs++;
	return true;
}

bool Scene::addLight(Light *light)
{
	if(!light)
		return false;
	lightObjs.push_back(light);
	nLightObjs++;
	return true;
}

bool Scene::Intersection(Ray_t *R, vec3 *pt, bool secLargest)
{
	float t;	/* parameter of intersection */
	int min_primitive = -1;
	float min_t = INF;
	float sec_min_t = INF;
	int sec_min_primitive = -1;
	mat4 M;

	for(int k=0; k<objs.size(); k++)
	{
		t = objs[k]->Intersect(R);
		if(t>0 && t<min_t)
		{
			sec_min_t=min_t;
			sec_min_primitive = min_primitive;

			min_t=t;
			min_primitive = k;
		}
	}
	currIntersectionObj = min_primitive;
	if(min_t!=INF)
	{
		objs[currIntersectionObj]->transformNormal();
		*pt = R->P0 + min_t*R->P1;
		*pt = objs[currIntersectionObj]->transformPoint(*pt);
		return true;		
	}
	if(secLargest && sec_min_t !=INF)
	{
		currIntersectionObj = sec_min_primitive;
		objs[currIntersectionObj]->transformNormal();
		*pt = R->P0 + sec_min_t*R->P1;
		//*pt = objs[currIntersectionObj]->transformPoint(*pt);
		return true;		
	}	
	return false;
}

bool Scene::Visible(Ray_t *RayToLight,vec3 currentlightposition)
{
	float t;
	float min_t = INF;
	vec3 intersectionpoint;
	vec3 lightvector;
	vec3 check;
	nv_scalar dotproduct;
	for(int i = 0 ; i < objs.size(); i++)
	{
			t = objs[i]->Intersect(RayToLight);
			if(t > 0 && t < min_t)
			{
				min_t=t;

			}
	}
	if(min_t!=INF)
	{
		intersectionpoint = RayToLight->P0 + (min_t * RayToLight->P1);
		lightvector = currentlightposition - intersectionpoint;
		lightvector.normalize();
		dotproduct = dot(lightvector,RayToLight->P1);
//		check = cross(check,lightvector,RayToLight->P1);
//		check.normalize();
		//if(check.x == 0.0 && check.y == 0.0 && check.z == 0.0)
		//{
		//	return true;
		//}
		// false = not visible
		if(dotproduct <= 0)
		{
			// visible
			return true;
		}
		else
		{
			// not visible
			return false;
		}
	}
	// visible
	return true;
}

vec3 Scene::FindColor(vec3 pt, int depth)
{return vec3(0, 0, 0);
}

vec3 Scene::getRayIntersection(Ray_t *R, int depth, bool secLargest)
{
	vec3 color(0.0, 0.0, 0.0);
	vec3 pt;
	if(Intersection(R, &pt, secLargest))
	{	
		color = objs[currIntersectionObj]->getColor(R, pt, depth);
	}
	return color;
}

void Scene::traceScene()
{
	vec3 pt;
	myR.P0 = Cam->getEye();
	printf("eye: %f, %f, %f\n", myR.P0.x, myR.P0.y, myR.P0.z); 
//	system("cls");
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