
#include "SceneObjs.h"
#include "Image.h"
#include "string.h"
#include "stdio.h"
#include <stdio.h>
#include <math.h>
#include "Transformation.h"
#define EP 0.000001
#define TEST_CULL 1
#define INF 1000000
#define pi 3.1428571
#define RECURSION_DEPTH 0
#define max(a,b) (((a) > (b)) ? (a) : (b))

Transformation *transform=new Transformation();
Shade *shading = new Shade();
Light *light;
CameraRay *camera;

void SceneObjs::convertRayToHomogenous(Ray &ray, RayHomo &rhomo)
{
	rhomo.begin.x=ray.begin.x;
	rhomo.begin.y=ray.begin.y;
	rhomo.begin.z=ray.begin.z;
	rhomo.begin.w=1.0;

	rhomo.end.x=ray.end.x;
	rhomo.end.y=ray.end.y;
	rhomo.end.z=ray.end.z;
	rhomo.end.w=0.0;

	return;
}

Ray SceneObjs::inverseTransformRay(Ray &ray, Ray &raytrans, mat4 M)
{
	mat4 invM;
	invM=invert(invM, M);

	RayHomo rayh;
	convertRayToHomogenous(ray, rayh);

	/* apply transformations */
	rayh.begin = invM*rayh.begin;
	rayh.end = invM*rayh.end;

	/* set individual coordinates, as 'w' is 0 for a ray */
	raytrans.begin = vec3(rayh.begin);
	raytrans.end = vec3(rayh.end);

	return raytrans;
}

//Ray
Ray::Ray()
{
	begin = vec3(0,0,0);
	end = vec3(0,0,0);
}


RayHomo::RayHomo()
{
	begin = vec4(0,0,0,0);
	end = vec4(0,0,0,0);
}

//Quadrilateral
Quadrilateral::Quadrilateral()
{
		vertices[0] = vec3(0,0,0);
		vertices[1] = vec3(0,0,0);
		vertices[2] = vec3(0,0,0);
		vertices[3] = vec3(0,0,0);
		transformation=transform->getTransform(); 
}

Quadrilateral::Quadrilateral(vec3& v_00,vec3& v_10,vec3& v_11, vec3& v_01)
	{
		vertices[0] = v_00;
		vertices[1] = v_10;
		vertices[2] = v_11;
		vertices[3] = v_01;
		transformation=transform->getTransform(); 
	}

vec3 Quadrilateral::transformPoint(vec3 pt)
{
	vec4 v=getHomogenous(pt);
	v=transformation*v;
	return getDehomogenous(v);
}
/*
vec3 Quadrilateral::getColor(Scene *scene,Ray *ray, vec3 pt, int depth)
{
	vec3 color(0.0,0.0,0.0);
	return color;
}*/

//Sphere
Sphere::Sphere()
{
	rad = 0.0;
	center = vec3(0,0,0);
}

Sphere::Sphere(vec3 _center, float _rad)
{
	rad = _rad;
	center = _center;
	transformation=transform->getTransform(); 
}

vec3 Sphere::transformPoint(vec3 pt)
{
	vec4 v=getHomogenous(pt);
	v=transformation*v;
	return getDehomogenous(v);
}

vec3 Sphere::getNormal(vec3 point)
{
	vec3 normal = point - getCenter();
	normal.normalize();
	normal = TransformNormal(transformation, normal);
	return normal;
}

vec3 Sphere::getReflection(Ray *ray,int depth,bool secLargest,Scene *scene,vec3 pt)
{
	if(depth == scene->maxdepth)
		return vec3(0.0,0.0,0.0);

	vec3 color = vec3(0.0,0.0,0.0);
	if((this->specular.x != 0.0 || this->specular.y != 0.0 || this->specular.z != 0.0)&&shininess==128)
	{
		vec3 normal=getNormal(pt);
		vec3 I = pt - ray->begin;
		I.normalize();
		float dotproduct = dot(dotproduct,I,normal);
		//vec3 R = ray->end-2.0f*dotproduct*normal;
		vec3 R = -2*dotproduct*normal + I;
		R.normalize();

		Ray reflected;
		reflected.begin = pt;
		//reflected.end = R;
		reflected.begin += 0.1*reflected.end;
		reflected.end = R;
		//reflected.begin = pt + 0.1*reflected.end;

		if(depth<scene->maxdepth)
		{
			vec3 tempcolor = camera->getRayIntersection(&reflected,depth++,false,scene);
			tempcolor = tempcolor / 255.0;
			color += tempcolor * this->specular;
			//printf("The color in reflection is %f %f %f\n",color.x,color.y,color.z);
		}
	}
	return color;
}

//void Triangle::transNorm()
//{
//	if(hasNormals)
//	{
//		mat4 M, transpM;
//		M=invert(M, transform);
//		transpM=transpose(transpM, M);
//		vec4 n;
//		/* normal transformations : n_trans = (inv(M))'.n */
//		for(int i=0; i<3; i++)
//		{
//			n=getHomogenous(norms[i]);
//			n=transpM*n;
//			norms[i]=getDehomogenous(n);
//		}
//	}
//}

vec3 Triangle::getReflection(Ray *ray,int depth,bool secLargest,Scene *scene,vec3 pt)
{
	
	if(depth == scene->maxdepth)
		return vec3(0.0,0.0,0.0);

	vec3 color = vec3(0.0,0.0,0.0);
	if(this->specular.x > 0.0 || this->specular.y >0.0 || this->specular.z >0.0)
	{
		vec3 normal=getNormal(pt);
		vec3 I = pt - ray->begin;
		I.normalize();
		//float dotproduct = dot(dotproduct,I,normal);
		//vec3 R = ray->end-2.0f*dotproduct*normal;
		vec3 R = -2*(I*normal)*normal + I;
		R.normalize();

		Ray reflected;
		reflected.begin = pt;
		reflected.end = R;
		reflected.begin += 0.1*reflected.end;
		//reflected.end = R;
		//reflected.begin = pt + 0.1*reflected.end;

		if(depth<scene->maxdepth)
		{
			vec3 tempcolor = camera->getRayIntersection(&reflected, depth++ ,false,scene);
			//tempcolor = tempcolor / 255.0;
			color += tempcolor * this->specular;
			//printf("The color in reflection is %f %f %f\n",color.x,color.y,color.z);
		}
	}
	return color;
}

vec3 SceneObjs::getColor(Scene *scene,Ray *ray, vec3 pt, int depth,bool secLargest)
{

	int visibility=0;
	vec3 viewer_direction = ray->begin - pt;
	vec3 color(0.0,0.0,0.0);
	Ray pointToLight;
	pointToLight.begin = pt;
	for(int k = 0 ; k < scene->lights.size() ; k++)
	{
		pointToLight.end = scene->lights[k]->position - pt;
		pointToLight.end.normalize();
		pointToLight.begin += 0.1 * pointToLight.end;
		
		//check visibility
		if(scene->isVisible(&pointToLight,scene->lights[k]->position))
		{
			visibility = 1;
		}
		
		//applying the equation
		vec3 N = getNormal(pt);
		vec3 L = pointToLight.end;
		vec3 light_specular = scene->lights[k]->specular;
		vec3 light_diffuse = scene->lights[k]->diffuse;
		vec3 light_ambient = scene->lights[k]->ambient;

		vec3 dir = pt - scene->lights[k]->position;
		
		// distance between light source and vertex
		float distance = dir.norm();
		vec3 attenuation = scene->lights[k]->attenuation;

		float specularfactor;
		float diffusefactor = dot(diffusefactor,L,N);
		vec3 R = -L + (2 * diffusefactor * N);
		float tempdiff = dot(tempdiff,R,viewer_direction);
		specularfactor = max(tempdiff,0.0);
		specularfactor = pow(specularfactor,shininess);
		diffusefactor = max(diffusefactor,0.0);

		//applying the model equation
		color = (calculateAttenuation(attenuation,distance) * spotlight_effect(scene->lights[k],dir)) * ((light_ambient * this->ambient) + (specularfactor * light_specular * this->specular) + (diffusefactor * light_diffuse * this->diffuse));
//		printf("Color is %f %f %f\n",color.x,color.y,color.z);
	}
	
	//adding visibility
	color += visibility*color;

	if(scene->lights.size() > 0)
	{
		color += emission + (scene->lights[0]->getGlobalAmbient() * this->ambient);
	}
	else
	{
		color += emission + this->ambient;
	}

	/*  reflection       */

	color += getReflection(ray,depth,secLargest,scene,pt);

	//printf("The color value is : %f %f %f \n",color.x,color.y,color.z);
	color.x *= 255.0;
	color.y *= 255.0;
	color.z *= 255.0;
	if(color.x>255.0)
		color.x=255.0;
	if(color.y>255.0)
		color.y=255.0;
	if(color.z>255.0)
		color.z=255.0;

	return color;
}

void Sphere::setparams(Shade *shade)
{
	if(shade)
	{
	if(shade->isAmbientSet)
		ambient = shade->ambient;
	if(shade->isDiffuseSet)
		diffuse = shade->diffuse;
	if(shade->isEmissionSet)
		emission = shade->emission;
	if(shade->isSpecularSet)
		specular = shade->specular;
	if(shade->isShininessSet)
		shininess = shade->shininess;
	}
}

//Triangle
Triangle::Triangle()
{
	vertices[0] = vec3(0.0,0.0,0.0);
	vertices[1] = vec3(0.0,0.0,0.0);
	vertices[2] = vec3(0.0,0.0,0.0);
	norms[0] = vec3(0.0,0.0,0.0);
	norms[1] = vec3(0.0,0.0,0.0);
	norms[2] = vec3(0.0,0.0,0.0);
	transformation=transform->getTransform(); 
}

Triangle::Triangle(vec3 v1, vec3 v2, vec3 v3)
{
	vertices[0] = v1;
	vertices[1] = v2;
	vertices[2] = v3;
	transformation=transform->getTransform(); 
}

vec3 Triangle::transformPoint(vec3 pt)
{
	vec4 v=getHomogenous(pt);
	v=transformation*v;
	return getDehomogenous(v);
}

vec3 Triangle::getNormal(vec3 point)
{
	vec3 A = vertices[0];
	vec3 B = vertices[1];
	vec3 C = vertices[2];

	vec3 e1 = A - B;
	vec3 e2 = C - B;
	vec3 normal = cross(normal,e1,e2);
	normal.normalize();

	normal=TransformNormal(transformation, normal);

	return normal;
}

vec3 SceneObjs::TransformNormal(mat4 trans, vec3 normal)
{
	mat4 M, transM;
	M=invert(M, trans);
	transM=transpose(transM, M);
	vec4 ntrans = getHomogenous(normal);
	ntrans = transM*ntrans;
	return(getDehomogenous(ntrans));
}

/*vec3 Triangle::getColor(Scene* scene,Ray *ray, vec3 pt, int depth)
{
	vec3 color(255.0,0.0,0.0);
	
	return color;
}*/

vec3 subt(vec3 v1, vec3 v2)
{
	return (v1 - v2);
}

void Triangle::setparams(Shade *shade)
{
	if(shade)
	{
	if(shade->isAmbientSet)
		ambient = shade->ambient;
	if(shade->isDiffuseSet)
		diffuse = shade->diffuse;
	if(shade->isEmissionSet)
		emission = shade->emission;
	if(shade->isSpecularSet)
		specular = shade->specular;
	if(shade->isShininessSet)
		shininess = shade->shininess;
	}	

}

vec4 SceneObjs::getHomogenous(vec3 v)
{ return vec4(v.x, v.y, v.z, 1.0); }

vec3 SceneObjs::getDehomogenous(vec4 v)
{ return vec3(v.x/v.w, v.y/v.w, v.z/v.w); }

float SceneObjs::calculateAttenuation(vec3 attenuation, float distance)
{
	float value;
	value = 1.0/(attenuation.x + (attenuation.y * distance) + (attenuation.z * distance * distance));
	return value;
}

float SceneObjs::spotlight_effect(Light *light,vec3 dir)
{
	// If the spot light is actually not a spot light ;)
	if(light->spot_cutoff >= 180.0)
	{
		return 1.0;
	}
	float c;
	dir.normalize();
	light->spot_direction.normalize();
	
	c = dot(dir,light->spot_direction);
	// If the vertex is outside the spotlight
	if(c < cosf(((pi/180.0) * light->spot_cutoff)))
	{
		return 0.0;
	}
	c = max(c,0.0);
	c = pow(c, light->spot_exponent);
	return c;
}

bool Scene::isVisible(Ray *pointToLight,vec3 lightposition)
{
	float t,u,v;
	float mint = INF;
	vec3 pt;
	vec3 lightdirection;
	vec3 check;
	float visible;
	for(int i = 0 ; i < sobjects.size(); i++)
	{
			sobjects[i]->intersect_ray((*pointToLight),u,v,t);
			if(t > 0 && t < mint)
			{
				mint=t;

			}
	}
	if(mint!=INF)
	{
		pt = pointToLight->begin + (mint * pointToLight->end);
		lightdirection = lightposition - pt;
		lightdirection.normalize();
		visible = dot(visible,lightdirection,pointToLight->end);

		//check visibility
		if(visible <= 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	// visible
	return true;
}



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
		color = scene->sobjects[scene->currIntersectionObj]->getColor(scene,ray, point, depth,secLargest);
		//printf("Color in intersection is: %f %f %f\n",color.x,color.y,color.z);
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



/********************Intersections**************************/

bool Quadrilateral::intersect_ray(Ray& r, float& u, float& v, float& t)
{
  static float eps = float(10e-6);

  // Rejects rays that are parallel to Q, and rays that intersect the plane of
  // Q either on the left of the line V00V01 or on the right of the line V00V10.
	
  vec3 E_01 = v_10() - v_00();
  vec3 E_03 = v_01() - v_00();
  vec3 P = cross(P,r.end, E_03);
  float det = dot(det,E_01, P);
  if (std::abs(det) < eps) return false;
  float inv_det = float(1.0) / det;
  vec3 T = r.begin - v_00();
  float alpha = dot(alpha,T, P) * inv_det;
  if (alpha < float(0.0)) return false;
  // if (alpha > float(1.0)) return false; // Uncomment if VR is used.
  vec3 Q = cross(Q,T, E_01);
  float beta = dot(beta,r.end, Q) * inv_det;
  if (beta < float(0.0)) return false; 
  // if (beta > float(1.0)) return false; // Uncomment if VR is used.

  if ((alpha + beta) > float(1.0)) {

    // Rejects rays that intersect the plane of Q either on the
    // left of the line V11V10 or on the right of the line V11V01.

    vec3 E_23 = v_01() - v_11();
    vec3 E_21 = v_10() - v_11();
    vec3 P_prime = cross(P_prime,r.end, E_21);
    float det_prime = dot(det_prime,E_23, P_prime);
    if (std::abs(det_prime) < eps) return false;
    float inv_det_prime = float(1.0) / det_prime;
	vec3 T_prime = r.begin - v_11();
    float alpha_prime = dot(alpha_prime,T_prime, P_prime) * inv_det_prime;
    if (alpha_prime < float(0.0)) return false;
    vec3 Q_prime = cross(Q_prime,T_prime, E_23);
	float beta_prime = dot(beta_prime,r.end, Q_prime) * inv_det_prime;
    if (beta_prime < float(0.0)) return false;
  }

  // Compute the ray parameter of the intersection point, and
  // reject the ray if it does not hit 

  t = dot(t,E_03, Q) * inv_det;
  if (t < float(0.0)) return false; 

  // Compute the barycentric coordinates of the fourth vertex.
  // These do not depend on the ray, and can be precomputed
  // and stored with the quadrilateral.  

  float alpha_11, beta_11;
  vec3 E_02 = v_11() - v_00();
  vec3 n = cross(n,E_01, E_03);

  if ((std::abs(n.x) >= std::abs(n.y))
    && (std::abs(n.x) >= std::abs(n.z))) {

    alpha_11 = ((E_02.y * E_03.z) - (E_02.z * E_03.y)) / n.x;
    beta_11  = ((E_01.y * E_02.z) - (E_01.z * E_02.y)) / n.x;
  }
  else if ((std::abs(n.y) >= std::abs(n.x))
    && (std::abs(n.y) >= std::abs(n.z))) {  

    alpha_11 = ((E_02.z * E_03.x) - (E_02.x * E_03.z)) / n.y;
    beta_11  = ((E_01.z * E_02.x) - (E_01.x * E_02.z)) / n.y;
  }
  else {

    alpha_11 = ((E_02.x * E_03.y) - (E_02.y * E_03.x)) / n.z;
    beta_11  = ((E_01.x * E_02.y) - (E_01.y * E_02.x)) / n.z;
  }

  // Compute the bilinear coordinates of the intersection point.

  if (std::abs(alpha_11 - float(1.0)) < eps) {    

    // Q is a trapezium.
    u = alpha;
    if (std::abs(beta_11 - float(1.0)) < eps) v = beta; // Q is a parallelogram.
    else v = beta / ((u * (beta_11 - float(1.0))) + float(1.0)); // Q is a trapezium.
  }
  else if (std::abs(beta_11 - float(1.0)) < eps) {

    // Q is a trapezium.
    v = beta;
    u = alpha / ((v * (alpha_11 - float(1.0))) + float(1.0));
  }
  else {

    float A = float(1.0) - beta_11;
    float B = (alpha * (beta_11 - float(1.0)))
      - (beta * (alpha_11 - float(1.0))) - float(1.0);
    float C = alpha;
    float D = (B * B) - (float(4.0) * A * C);
    float Q = float(-0.5) * (B + ((B < float(0.0) ? float(-1.0) : float(1.0))
      * std::sqrt(D)));
    u = Q / A;
    if ((u < float(0.0)) || (u > float(1.0))) u = C / Q;
    v = beta / ((u * (beta_11 - float(1.0))) + float(1.0)); 
  }

  return true;
}


bool Sphere::intersect_ray(Ray &ray,float &u,float &v, float &t)
{
	float  root,root1,root2;
	float r = getRadius();
	
	Ray raytrans, raytrans2;
	inverseTransformRay(ray, raytrans, transformation);
	
	raytrans2.begin=raytrans.begin;
	raytrans2.end=raytrans.end;
	vec3 dis = raytrans.begin-getCenter();

	raytrans.end.normalize();
	float temp = dot(temp, dis, dis);

	float a = dot(a,raytrans.end, raytrans.end);
	float b = 2 * dot(b,raytrans.end, dis);
	float c = temp - (r * r);

	//Discriminant
	float D = b * b - 4 * a * c;
	
	//no intersection
	if(D < 0)
		return false;
	
	if(b < 0)
		root = (-b - sqrtf(D))/2.0;
	else
		root = (-b + sqrtf(D))/2.0;

	//computing root1 and root2
	root1 = root/a;
	root2 = c/root;

	if (root1 > root2)
    {
        float swap = root1;
        root1 = root2;
        root2 = swap;
    }

	//no intersection since sphere in ray's -ve direction
	if (root2 < 0)
        return false;

    // if root1 is less than zero, the intersection point is at root2
    if (root1 < 0)
    {
		t = root2*1/nv_norm(raytrans2.end);
		return true;
    }
    // else the intersection point is at root1
    else
    {
		t = root1*1/nv_norm(raytrans2.end);
        return true;
    }
}

float Triangle::area( vec3 v1, vec3 v2, vec3 v3)
{
	vec3 v;
	v = cross(v, (v2-v1), (v3-v1));

	return(v.norm()/2);
}

bool Triangle::intersect_ray(Ray& ray, float& u, float& v, float& t)
{
//	vec3 pvec,tvec,qvec;
	
	//get vertices of triangle
	vec3 v1 = getVertex1();
	vec3 v2 = getVertex2();
	vec3 v3 = getVertex3();

	//finding the edges
	vec3 edge1, edge2;
	edge1 = subt(v2,v1);
	edge2 = subt(v3,v1);

	Ray raytrans;
	inverseTransformRay(ray, raytrans, transformation);
	vec3 normal;
	normal = cross(normal,edge2,edge1);
	normal.normalize();

	float v1normal;
	float begnormal;
	float endnormal;

	v1normal = dot(v1normal,v1,normal);
	begnormal = dot(begnormal,ray.begin,normal);
	endnormal = dot(endnormal,ray.end,normal);

	t = (v1normal - begnormal)/endnormal;
	vec3 pt = raytrans.begin + t*raytrans.end;
	float area1 = area(v1, v2, v3);
	float area2 = area(v2, v3, pt);
	float area3 = area(v3, v1, pt);
	float area4 = area(v1,v2,pt);

	float al = area2/area1;
	float be = area3/area1;
	float ga = area4/area1;
	float sum = al+be+ga;

	if(sum <= 1.0001 && sum >= 0.9999)
		return true;
	else 
	{
		t=-1;
		return false;
	}

//	pvec = cross(pvec,raytrans.end,edge2);
//	float det = dot(det,edge1,pvec);
//
//#ifdef TEST_CULL
//	if(det < EP)
//		return false;
//	
//	//calculate distance from vertex1 to ray
//	tvec = subt(raytrans.begin,v1);
//	u = dot(u,tvec,pvec);
//
//	if(u > det)
//		return false;
//
//	qvec = cross(qvec,tvec,edge1);
//	v = dot(v,raytrans.end,qvec);
//
//	if(v <0 || (u+v) > det)
//		return false;
//
//	//calculate t
//	t = dot(t,edge2,qvec);
//	t = t * (1.0/det);
//	u = u * (1.0/det);
//	v = v * (1.0/det);
//
//#else 
//	if(det > -EP && det < EP)
//		return false;
//
//	tvec = subt(ray.begin,v1);
//	u = dot(u,tvec,pvec) * (1.0/det);
//
//	if(u < 0 || u > 1.0);
//		return false;
//
//	qvec = cross(qvec,tvec,edge1);
//	v = dot(v,ray.end,qvec) * (1.0/det);
//	
//	if(v < 0 || (u+v) > 1.0)
//		return false;
//
//	t = dot(t,edge2,qvec) * (1.0/det);
//#endif

	//return true;
	
}

void Scene::parsefile (FILE *fp) {
  char line[1000], command[1000] ; // Very bad to prefix array size :-)
  int sizeset = 0 ;	
  int count;
  int qflag=0,sflag=0,tflag=0;
  
  while (!feof(fp)) {
	count =0;
    fgets(line,sizeof(line),fp) ;
    if (feof(fp)) break ;
    if (line[0] == '#') continue ; // Comment lines
    
    int num = sscanf(line, "%s", command) ;
    if (num != 1) continue ; // Blank line etc.

	  // The first line should be the size command setting the image size
    if (!strcmp(command, "size")) {
		int sizex,sizey;
		assert(!strcmp(command, "size")) ;
    int num = sscanf(line, "%s %d %d", command, &sizex, &sizey) ;
    assert(num == 3) ;
    assert(!strcmp(command, "size")) ;
	width = sizex;
	height = sizey;
	}
    
	/* Other properties  */

	 else if (!strcmp(command, "maxdepth")) {
	  int num = sscanf(line, "%s %d", command, &maxdepth) ;
	  assert(num == 2) ;
	  assert(!strcmp(command, "maxdepth")) ;
	  fprintf(stderr, "Maxdepth set to %d but irrelevant for OpenGL\n", 
		  maxdepth) ;
	  printf("maxdepth is %d\n",maxdepth);
	}

       else if (!strcmp(command, "output")) {
	   char out[300] ;
	   int num = sscanf(line, "%s %s", command, out) ;
	   assert(num == 2) ;
	   assert(!strcmp(command, "output")) ;
	   //fprintf(stderr, "Output image file set to: %s but ignored for OpenGL\n",out) ;
	   strcpy(outputfile,out);
	   printf("The output file is : %s\n",out);
       }


    // Now, we simply parse the file by looking at the first line for the 
    // various commands
    
    /**************        CAMERA LOCATION **********/
    if (!strcmp(command, "camera")) {
		printf("In camera\n");
      double lookfrom[3], lookat[3], up[3], fovxy ;
      int num = sscanf(line, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
		       command, lookfrom, lookfrom+1, lookfrom+2, 
		       lookat, lookat+1, lookat+2, up, up+1, up+2, &fovxy) ;
	  printf("Number of arguments on camera is %d\n",num);
      if (num != 11) {
	fprintf(stderr, "camera from[3] at[3] up[3] fov\n") ;
	exit(1) ;
	  }
      assert(!strcmp(command,"camera")) ;
	printf("camera vals read are are %f %f %f , %f %f %f, %f %f %f and fov %f\n",lookfrom[0],lookfrom[1],lookfrom[2],lookat[0],lookat[1],lookat[2],up[0],up[1],up[2],fovxy);
	cameraval[0] = vec3(lookfrom[0],lookfrom[1],lookfrom[2]);
	cameraval[1] = vec3(lookat[0],lookat[1],lookat[2]);
	cameraval[2] = vec3(up[0],up[1],up[2]);
	fov = fovxy;
	camera = new CameraRay(cameraval[0],cameraval[1],cameraval[2],fov);
	printf("camera vals are %f %f %f , %f %f %f, %f %f %f and fov %f\n",cameraval[0].x,cameraval[0].y,cameraval[0].z,cameraval[1].x,cameraval[1].y,cameraval[1].z,cameraval[2].x,cameraval[2].y,cameraval[2].z,fov);
	}
	
    /****************************************/

    /***********  GEOMETRY *******************/
	else if (!strcmp(command, "sphere")) {
		printf("In sphere\n");
	  double radius ; // Syntax is sphere x y z radius 
	  double pos[3] ;
	  int num = sscanf(line, "%s %lf %lf %lf %lf", command, pos, pos+1, pos+2, &radius) ;
	  if (num != 5) {
		fprintf(stderr, "sphere x y z radius\n") ;
		exit(1) ;
	  }
	  vec3 center = vec3(pos[0],pos[1],pos[2]);
	  Sphere *s =new Sphere(center,radius);
	  s->setparams(shading);
	  sobjects.push_back(s);
	  spherecount++;
	  sflag=1;
	}

	else if (!strcmp(command, "maxverts")) {
	  int num = sscanf(line, "%s %d", command, &maxverts) ;
	  assert(num == 2) ; assert(maxverts > 0) ;
	  assert(!strcmp(command,"maxverts")) ;
	  assert(vert = new Vertex[maxverts]) ;
	}

	else if (!strcmp(command, "vertex")) {  // Add a vertex to the stack
	  assert(maxverts) ; assert(curvert < maxverts) ;
	  Vertex v ;
	  int num = sscanf(line, "%s %lf %lf %lf", command, v.pos, v.pos+1, v.pos+2) ;
	  assert(num == 4) ; assert(!strcmp(command,"vertex")) ;
	  vert[curvert] = v ;
	  ++curvert ;
	}
	  
	else if (!strcmp(command, "vertexnormal")) {  
	  // Add a vertex to the stack with a normal
	  assert(maxvertnorms) ; assert(curvertnorm < maxvertnorms) ;
	  VertexNormal vn ;

	  int num = sscanf(line, "%s %lf %lf %lf %lf %lf %lf", 
		  command, vn.pos, vn.pos+1, vn.pos+2, 
		  vn.normal, vn.normal+1, vn.normal+2) ;

	  assert(num == 7) ; assert(!strcmp(command,"vertexnormal")) ;
	  vertnorm[curvertnorm] = vn ;
	  ++curvertnorm ;
	}		

	else if (!strcmp(command, "tri")) { // Triangle from 3 vertices
	 int pts[3] ; 
	 int num = sscanf(line, "%s %d %d %d", command, pts, pts+1, pts+2) ;
	 assert(num == 4) ; assert(!strcmp(command,"tri")) ;
	 int i,j ;
	 for (i = 0 ; i < 3 ; i++) {
	   assert(pts[i] >= 0 && pts[i] < maxverts) ;
	 }
	  double vertex[3][3] ;
	  double normal[3] ;
	  for (i = 0 ; i < 3 ; i++) 
	    for (j = 0 ; j < 3 ; j++)
	      vertex[i][j] = vert[pts[i]].pos[j] ;

	vec3 vertex1 = vec3(vertex[0][0],vertex[0][1],vertex[0][2]);
	vec3 vertex2 = vec3(vertex[1][0],vertex[1][1],vertex[1][2]);
	vec3 vertex3 = vec3(vertex[2][0],vertex[2][1],vertex[2][2]);
	Triangle *t = new Triangle(vertex1,vertex2,vertex3);
	t->setparams(shading);
	sobjects.push_back(t);
	trianglecount++;
	tflag=1;
	}

	/* Transformations  */
	
	else if (!strcmp(command, "translate")) {
	  double x,y,z ; // Translate by x y z as in standard OpenGL

	  int num = sscanf(line, "%s %lf %lf %lf",command, &x, &y, &z) ;
	  if (num != 4) {
		fprintf(stderr, "translate x y z\n") ;
		exit(1) ;
	  }
	  vec3 args = vec3(x,y,z);
	  printf("the args are : %f %f %f\n",x,y,z);
	  transform->translate(args);
	}

	else if (!strcmp(command, "rotate")) {
	  double ang, x,y,z ; // Rotate by an angle about axis x y z as in standard OpenGL

	  int num = sscanf(line, "%s %lf %lf %lf %lf",command, &x, &y, &z, &ang) ;
	  if (num != 5) {
		fprintf(stderr, "rotate angle x y z\n") ;
		exit(1) ;
	  }
	  vec3 args = vec3(x,y,z);
	  transform->rotate(args,ang);
	}

	else if (!strcmp(command, "scale")) {
	  double x,y,z ; // Scale by x y z as in standard OpenGL

	  int num = sscanf(line, "%s %lf %lf %lf",command, &x, &y, &z) ;
	  if (num != 4) {
		fprintf(stderr, "scale x y z\n") ;
		exit(1) ;
	  }
	
	  vec3 args = vec3(x,y,z);
	  transform->scale(args);
	
	}

	else if (!strcmp(command, "pushTransform")) {
	  // Push the current matrix on the stack as in OpenGL
		transform->pushMat();
	}

	else if (!strcmp(command, "popTransform")) {
	  // Pop the current matrix as in OpenGL
		transform->popMat();
	}

	
	   /**************** Lights******************/

	   else if (!strcmp(command, "directional")) {
	 float direction[4], color[4] ; color[3] = 1.0 ; direction[3] = 0.0 ;
	 int num = sscanf(line, "%s %f %f %f %f %f %f", command, direction, direction+1, direction+2, color, color+1, color+2) ;
	 assert(num == 7) ;
	 assert(lightnum >= 0 && lightnum < 7) ;

	 //int mylight = GL_LIGHT0 + lightnum ;
	 light = new Light();
	 light->setvalues(direction,color,shading,0);
	 lights.push_back(light);
	 ++lightnum ;
       }

      else if (!strcmp(command, "point")) {
	 float direction[3], color[3];
	 int num = sscanf(line, "%s %f %f %f %f %f %f", command, direction, direction+1, direction+2, color, color+1, color+2) ;
	 assert(num == 7) ;
	 assert(lightnum >= 0 && lightnum < 7) ;
	 //int mylight = GL_LIGHT0 + lightnum ;
	 Light *light = new Light();
	 light->setvalues(direction,color,shading,1);
	 lights.push_back(light);
	 ++lightnum ;	 
       }

     else if (!strcmp(command, "attenuation")) {
		 float attval[6]={0.0,0.0,0.0,0.0,0.0,0.0};			
       int num = sscanf(line, "%s %f %f %f", command, &attval[0],&attval[1],&attval[2]) ;
       assert(num == 4) ;
       assert(!strcmp(command, "attenuation")) ;
		if(!shading)
		   shading = new Shade();
		shading->setvalues(attval,5);
     }

     else if (!strcmp(command, "ambient")) {
       float args[6];
	   int num = sscanf(line, "%s %f %f %f", command, args, args + 1, args +2) ;
       assert(num == 4) ;
       assert(!strcmp(command, "ambient")) ;
       if(!shading)
		   shading = new Shade();
	   else
		   if(shading->isAmbientSet)
		   {
			   delete shading;
			   shading = new Shade();
		   }
	   shading->setvalues(args,0);
     }

    /*******************************************************/

    /****************** MATERIALS ************************/

     else if (!strcmp(command, "diffuse")) {
       float args[6];
       int num = sscanf(line, "%s %f %f %f", command, args, args + 1, args +2) ;
       assert(num == 4) ; assert (!strcmp(command, "diffuse")) ;
       if(!shading)
		   shading = new Shade();
	   else
		   if(shading->isDiffuseSet)
		   {
			   delete shading;
			   shading = new Shade();
		   }
	   shading->setvalues(args,2);
     }

     else if (!strcmp(command, "specular")) {
       float args[6];
       int num = sscanf(line, "%s %f %f %f", command,args, args + 1, args +2) ;
       assert(num == 4) ; assert (!strcmp(command, "specular")) ;
       if(!shading)
		   shading = new Shade();
	   else
		   if(shading->isSpecularSet)
		   {
			   delete shading;
			   shading = new Shade();
		   }
	   shading->setvalues(args,1);
     }

     else if (!strcmp(command, "shininess")) {
       float args[6];
       int num = sscanf(line, "%s %f", command, args) ;
       assert(num == 2) ; assert (!strcmp(command, "shininess")) ;
       if(!shading)
		   shading = new Shade();
	   else
		   if(shading->isShininessSet)
		   {
			   delete shading;
			   shading = new Shade();
		   }
		   shading->setvalues(args,4);
     }

     else if (!strcmp(command, "emission")) {
	  float args[6];
       int num = sscanf(line, "%s %f %f %f", command, args, args + 1, args +2) ;
       assert(num == 4) ; assert (!strcmp(command, "emission")) ;
       if(!shading)
		   shading = new Shade();
	   else
		   if(shading->isEmissionSet)
		   {
			   delete shading;
			   shading = new Shade();
		   }
		   shading->setvalues(args,3);
     }

	 else if (!strcmp(command, "spot")) {
       float args[6];
       int num = sscanf(line, "%s %f %f %f %f %f", command, args, args + 1, args +2,args +3,args +4) ;
       assert(num == 6) ; assert (!strcmp(command, "spot")) ;
       if(!shading)
		   shading = new Shade();
	   shading->setvalues(args,6);
     }

	count++;
	}
	totalobjcount = spherecount+trianglecount+quadcount;

	camera->generateRays(this);
}

