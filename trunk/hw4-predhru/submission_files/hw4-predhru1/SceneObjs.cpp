#include "SceneObjs.h"
#include "Image.h"
#include "string.h"
#include "stdio.h"
#include <stdio.h>
#include <math.h>
#include "Transformation.h"
#define EP 0.000001
#define TEST_CULL 1

Transformation *transform=new Transformation();

//vec3 SceneObjs::calcIllumination(Light *light,vec3 normal, vec3 light_direction, vec3 viewer_direction,vec3 pt)
//{
//	vec3 color;
//
//	vec3 light_specular = light->specular;
//	vec3 light_diffuse = light->diffuse;
//	vec3 light_ambient = light->ambient;
//
//	vec3 direction_vector = pt - light->position;
//	// distance between light source and vertex
//	float d = direction_vector.norm();
//	vec3 light_attenuation = light->attenuation;
//
//	vec3 reflection_direction;
//	nv_scalar specularfactor;
//	nv_scalar diffusefactor;
//
//	reflection_direction = -light_direction + (2 * dot(light_direction,normal) * normal);
//	
//	specularfactor = max(dot(reflection_direction,viewer_direction),0.0);
//	specularfactor = pow(specularfactor,shininess);
//	diffusefactor = max(dot(light_direction,normal),0.0);
//	
//	color = (calculateAttenuation(light_attenuation,d) * spotlight_effect(light,direction_vector)) * ((light_ambient * this->ambient) + (specularfactor * light_specular * this->specular) + (diffusefactor * light_diffuse * this->diffuse));
//	return color;
//}
//
//float SceneObjs::calcAttenuation(vec3 attenuation, float d)
//{
//	float value;
//	value = attenuation.x + (attenuation.y * d) + (attenuation.z * d * d);
//	value = 1/value;
//	return value;
//}
//
//float SceneObjs::spotlight_effect(Light *light,vec3 direction_vector)
//{
//	// If the spot light is actually not a spot light ;)
//	if(light->spot_cutoff >= 180.0)
//	{
//		return 1.0;
//	}
//	float cone;
//	direction_vector.normalize();
//	light->spot_direction.normalize();
//	
//	cone = dot(direction_vector,light->spot_direction);
//	// If the vertex is outside the spotlight
//	if(cone < cosf(degToRad * light->spot_cutoff))
//	{
//		return 0.0;
//	}
//	cone = max(cone,0.0);
//	cone = pow(cone, light->spot_exponent);
//	return cone;
//}

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

vec3 Quadrilateral::getColor(Ray *ray, vec3 pt, int depth)
{
	vec3 color(0.0,0.0,0.0);
	return color;
}

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

vec3 Sphere::getColor(Ray *ray, vec3 pt, int depth)
{
	vec3 color(0.0,0.0,255.0);
	return color;
}

//Triangle
Triangle::Triangle()
{
	vertices[0] = vec3(0.0,0.0,0.0);
	vertices[1] = vec3(0.0,0.0,0.0);
	vertices[2] = vec3(0.0,0.0,0.0);
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

vec3 Triangle::getColor(Ray *ray, vec3 pt, int depth)
{
	vec3 color(255.0,0.0,0.0);
	return color;
}

vec3 subt(vec3 v1, vec3 v2)
{
	return (v1 - v2);
}

vec4 SceneObjs::getHomogenous(vec3 v)
{ return vec4(v.x, v.y, v.z, 1.0); }

vec3 SceneObjs::getDehomogenous(vec4 v)
{ return vec3(v.x/v.w, v.y/v.w, v.z/v.w); }

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


bool Triangle::intersect_ray(Ray& ray, float& u, float& v, float& t)
{
	vec3 pvec,tvec,qvec;

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

	pvec = cross(pvec,raytrans.end,edge2);
	float det = dot(det,edge1,pvec);

#ifdef TEST_CULL
	if(det < EP)
		return false;
	
	//calculate distance from vertex1 to ray
	tvec = subt(raytrans.begin,v1);
	u = dot(u,tvec,pvec);

	if(u < 0 || u > det)
		return false;

	qvec = cross(qvec,tvec,edge1);
	v = dot(v,raytrans.end,qvec);

	if(v <0 || (u+v) > det)
		return false;

	//calculate t
	t = dot(t,edge2,qvec);
	t = t * (1.0/det);
	u = u * (1.0/det);
	v = v * (1.0/det);

#else 
	if(det > -EP && det < EP)
		return false;

	tvec = subt(ray.begin,v1);
	u = dot(u,tvec,pvec) * (1.0/det);

	if(u < 0 || u > 1.0);
		return false;

	qvec = cross(qvec,tvec,edge1);
	v = dot(v,ray.end,qvec) * (1.0/det);
	
	if(v < 0 || (u+v) > 1.0)
		return false;

	t = dot(t,edge2,qvec) * (1.0/det);
#endif

	return true;
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
	  ++curvert;
	  //Quadrilateral q = Quadrilateral(vert[0],vert[1],vert[2],vert[3]);	
	  //qlist[quadcount]=q;
	  //quadcount++;
	  //qflag=1;
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

	/* Other properties  */

	 else if (!strcmp(command, "maxdepth")) {
	  int num = sscanf(line, "%s %d", command, &maxdepth) ;
	  assert(num == 2) ;
	  assert(!strcmp(command, "maxdepth")) ;
	  fprintf(stderr, "Maxdepth set to %d but irrelevant for OpenGL\n", 
		  maxdepth) ;
	}

       else if (!strcmp(command, "output")) {
	   char out[300] ;
	   int num = sscanf(line, "%s %s", command, out) ;
	   assert(num == 2) ;
	   assert(!strcmp(command, "output")) ;
	   fprintf(stderr, "Output image file set to: %s but ignored for OpenGL\n",out) ;
	   strcpy(outputfile,out);
       }

	   /**************** Lights******************/

	   else if (!strcmp(command, "directional")) {
	 float direction[4], color[4] ; color[3] = 1.0 ; direction[3] = 0.0 ;
	 int num = sscanf(line, "%s %f %f %f %f %f %f", command, direction, direction+1, direction+2, color, color+1, color+2) ;
	 assert(num == 7) ;
	 assert(lightnum >= 0 && lightnum < 7) ;

	 //int mylight = GL_LIGHT0 + lightnum ;
	 Light *light = new Light();
	 lights.push_back(light);
	 ++lightnum ;
       }

      else if (!strcmp(command, "point")) {
	 float direction[4], color[4] ; color[3] = 1.0 ; direction[3] = 1.0 ;
	 int num = sscanf(line, "%s %f %f %f %f %f %f", command, direction, direction+1, direction+2, color, color+1, color+2) ;
	 assert(num == 7) ;
	 assert(lightnum >= 0 && lightnum < 7) ;

	 //int mylight = GL_LIGHT0 + lightnum ;
	 Light *light = new Light();
	 lights.push_back(light);
	 ++lightnum ;	 
       }

     else if (!strcmp(command, "attenuation")) {
       int num = sscanf(line, "%s %lf %lf %lf", command, attenuation, attenuation + 1, attenuation +2) ;
       assert(num == 4) ;
       assert(!strcmp(command, "attenuation")) ;
     }

     else if (!strcmp(command, "ambient")) {
       float ambient[4] ; ambient[3] = 1.0 ;
       int num = sscanf(line, "%s %f %f %f", command, ambient, ambient+1, ambient+2) ;
       assert(num == 4) ;
       assert(!strcmp(command, "ambient")) ;
       Shade *shading = new Shade();
	   shading->ambient = vec3(ambient[0],ambient[1],ambient[2]);
     }

    /*******************************************************/

    /****************** MATERIALS ************************/

     else if (!strcmp(command, "diffuse")) {
       float diffuse[4] ; diffuse[3] = 1.0 ;
       int num = sscanf(line, "%s %f %f %f", command, diffuse, diffuse+1, diffuse+2) ;
       assert(num == 4) ; assert (!strcmp(command, "diffuse")) ;
       
     }

     else if (!strcmp(command, "specular")) {
       float specular[4] ; specular[3] = 1.0 ;
       int num = sscanf(line, "%s %f %f %f", command, specular, specular+1, specular+2) ;
       assert(num == 4) ; assert (!strcmp(command, "specular")) ;
       
     }

     else if (!strcmp(command, "shininess")) {
       float shininess ;
       int num = sscanf(line, "%s %f", command, &shininess) ;
       assert(num == 2) ; assert (!strcmp(command, "shininess")) ;
       
     }

     else if (!strcmp(command, "emission")) {
       float emission[4] ; emission[3] = 1.0 ;
       int num = sscanf(line, "%s %f %f %f", command, emission, emission+1, emission+2) ;
       assert(num == 4) ; assert (!strcmp(command, "emission")) ;
       
     }
	count++;
	}
	totalobjcount = spherecount+trianglecount+quadcount;

}

