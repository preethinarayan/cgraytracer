#include "Camera.h"
#include "stdio.h"
#include "Image.h"
#define INF 10000
#define pi 3.1428571
#define RECURSION_DEPTH 5

CameraRay::CameraRay()
	{
		u = vec3(0,0,0);
		w = vec3(0,0,0);
		v = vec3(0,0,0);
		fov = 0.0;
	}

bool CameraRay::Intersection(Ray *ray, vec3 *pt, bool secLargest,Scene *scene)
{
	float u,v,t;	/* parameter of inte

//CameraRayrsection */
	int min_primitive = -1;
	float min_t = INF;
	float sec_min_t = INF;
	int sec_min_primitive = -1;
	mat4 M;
	bool intersect = false;
	int  i=0;
	//printf("Number of objects : %d\n",scene->sobjects.size());
	for(int k=0; k<scene->sobjects.size(); k++)
	{
		intersect = scene->sobjects[k]->intersect_ray(*(ray),u,v,t);
		if(intersect)
			//printf("INTERSECTION!!!!\n");

		if(t>0 && t<min_t)
		{
			sec_min_t=min_t;
			sec_min_primitive = min_primitive;

			min_t=t;
			min_primitive = k;
		}
	}
	scene->currIntersectionObj = min_primitive;
	if(min_t!=INF)
	{
		//objs[scene->currIntersectionObj]->transformNormal();
		
		*pt = ray->begin + min_t*ray->end;
		//printf("Point before transform : %f %f %f\n",(*pt).x,(*pt).y,(*pt).z);
		*pt = scene->sobjects[scene->currIntersectionObj]->transformPoint(*pt);
		//printf("Point after transform : %f %f %f\n",(*pt).x,(*pt).y,(*pt).z);
		return true;		
	}
	if(secLargest && sec_min_t !=INF)
	{
		scene->currIntersectionObj = sec_min_primitive;
		//scene->sobjects[scene->currIntersectionObj]->transformNormal();
		*pt = ray->begin + sec_min_t*ray->end;
		//*pt = objs[currIntersectionObj]->transformPoint(*pt);
		return true;		
	}	
	return false;
}

vec3 CameraRay::getRayIntersection(Ray *ray,int depth, bool secLargest,Scene *scene)
{
	vec3 color(0.0, 0.0, 0.0);
	vec3 point;
	if(Intersection(ray, &point, secLargest,scene))
	{	
		color = scene->sobjects[scene->currIntersectionObj]->getColor(scene,ray, point, depth);
	}
	return color;
	
}

CameraRay::CameraRay(vec3 _eye, vec3 _center, vec3 _up, float _fov)
	{
		eye = _eye;
		up = _up;
		center = _center;
		w = eye - center;
		printf("w is : %f %f %f\n",w.x,w.y,w.z);
		w.normalize();
		u = cross(u,up,w);
		u.normalize();
		printf("u is : %f %f %f\n",u.x,u.y,u.z);
		v = cross(v,w,u);
		printf("v is : %f %f %f\n",v.x,v.y,v.z);
		fov = _fov;
		printf("eye is : %f %f %f\n",eye.x,eye.y,eye.z);
	}

vec3 CameraRay::calcRays(Scene *scene,int i, int j)
{
	vec3 ray;
	float A;
	float B;

	float fovy=fov;
	float fovx=fovy*float(float(scene->width)/float(scene->height));
	
	A = tanf((pi / 180) * (fovx/2.0)) * ((j - (scene->width/2.0))/(scene->width/2.0));
	B = tanf((pi / 180) * (fovy/2.0)) * ((scene->height/2.0) - i)/ (scene->height/2.0);

	ray = (A * u) + (B * v) - w;
	ray.normalize();

	return ray;
}

void CameraRay::generateRays(Scene *scene)
{
	vec3 pt;
	ray.begin = eye;
	printf("eye is %f, %f, %f\n", eye.x, eye.y, eye.z); 
	int flag=0;
	float u,v,t;
	Image img;
	img.setSize(scene->width,scene->height);
	//float remainingtime = 0;
	//float totaltime = I->getHeight() + I->getWidth();
	//float percentremaining;
	
	for(int i=0; i<scene->height; i++)
	{
		//printf("\nRow %d\n", i);
		for(int j=0; j<scene->width; j++)
		{
			flag=0;
			vec3 color(0.0, 0.0, 0.0);
			ray.end = calcRays(scene,i,j);
			
			
			color = getRayIntersection(&ray, RECURSION_DEPTH, false,scene);
			img.setPixel(i,j,color.x,color.y,color.z);
			/*if(Intersection(&myR, &pt))
			{ 
				color = FindColor(pt, RECURSION_DEPTH);
			}*/
			//I->setPixel(i, j, color.x, color.y, color.z);
			//remainingtime += 1;
			//percentremaining = (remainingtime / totaltime) * 100;
			//system("cls");
			//printf("Progress: %0.3f", percentremaining); 
		}
	}

	img.writeImage(scene->outputfile);
}
