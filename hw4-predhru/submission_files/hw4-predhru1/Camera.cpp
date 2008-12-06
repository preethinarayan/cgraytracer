#include "Camera.h"
#include "stdio.h"
#include "Image.h"
#define pi 3.1428571


//CameraRay
CameraRay::CameraRay()
	{
		u = vec3(0,0,0);
		w = vec3(0,0,0);
		v = vec3(0,0,0);
		fov = 0.0;
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

void CameraRay::generateRays(Scene *scene)
{
	vec3 pt;
	ray.begin = eye;
	printf("eye is %f, %f, %f\n", eye.x, eye.y, eye.z); 
	int flag=0;
	float u,v,t;
	Image img;
	img.setSize();
	//float remainingtime = 0;
	//float totaltime = I->getHeight() + I->getWidth();
	//float percentremaining;
	Triangle tr;
	Sphere s;
	Quadrilateral q;
	for(int i=0; i<HEIGHT; i++)
	{
		//printf("\nRow %d\n", i);
		for(int j=0; j<WIDTH; j++)
		{
			flag=0;
			vec3 color(0.0, 0.0, 0.0);
			ray.end = calcRays(i,j);
			//color = getRayIntersection(&myR, RECURSION_DEPTH, false);
			//img.setPixel(i,j,5,5,5);
			for(int m=0;m<scene->spherecount;m++)
			{
				s = scene->slist[m];
				if(s.intersect_ray(s,ray,t))
				{
				img.setPixel(i,j,50,100,100);	
				flag=1;
				//printf("intersection!\n");
				}
			}

			for(int m=0;m<scene->quadcount;m++)
			{
				q = scene->qlist[m];
				if(q.intersect_ray(q,ray,u,v,t))
				{
				img.setPixel(i,j,100,50,200);
				flag=1;
				printf("intersection!\n");
				}
			}

			for(int m=0;m<scene->trianglecount;m++)
			{
				tr = scene->tlist[m]; 
				if(tr.intersect_ray(tr,ray,u,v,t))
				{
				img.setPixel(i,j,200,50,50);
				flag=1;
				//printf("intersection!\n");
				}
			}
				
			if(flag==0)
				img.setPixel(i,j,5,5,5);
			
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

	img.writeImage();
}
