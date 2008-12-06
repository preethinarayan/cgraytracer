#include "SceneObjs.h"
#include "Image.h"
#include "string.h"
#include<stdio.h>
#include <math.h>

#define EP 0.000001
#define TEST_CULL 1


//Ray
Ray::Ray()
{
	begin = vec3(0,0,0);
	end = vec3(0,0,0);
}

//Quadrilateral
Quadrilateral::Quadrilateral()
{
		vertices[0] = vec3(0,0,0);
		vertices[1] = vec3(0,0,0);
		vertices[2] = vec3(0,0,0);
		vertices[3] = vec3(0,0,0);	
}

Quadrilateral::Quadrilateral(vec3& v_00,vec3& v_10,vec3& v_11, vec3& v_01)
	{
		vertices[0] = v_00;
		vertices[1] = v_10;
		vertices[2] = v_11;
		vertices[3] = v_01;
	}
Quadrilateral q;

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
}
Sphere sphere;

//Triangle
Triangle::Triangle()
{
	vertices[0] = vec3(0.0,0.0,0.0);
	vertices[1] = vec3(0.0,0.0,0.0);
	vertices[2] = vec3(0.0,0.0,0.0);
}

Triangle::Triangle(vec3 v1, vec3 v2, vec3 v3)
{
	vertices[0] = v1;
	vertices[1] = v2;
	vertices[2] = v3;
}
Triangle tr;



vec3 subt(vec3 v1, vec3 v2)
{
	return (v1 - v2);
}


/********************Intersections**************************/

bool Quadrilateral::intersect_ray( Quadrilateral& q,Ray& r, float& u, float& v, float& t)
{
  static float eps = float(10e-6);

  // Rejects rays that are parallel to Q, and rays that intersect the plane of
  // Q either on the left of the line V00V01 or on the right of the line V00V10.
	
  vec3 E_01 = q.v_10() - q.v_00();
  vec3 E_03 = q.v_01() - q.v_00();
  vec3 P = cross(P,r.end, E_03);
  float det = dot(det,E_01, P);
  if (std::abs(det) < eps) return false;
  float inv_det = float(1.0) / det;
  vec3 T = r.begin - q.v_00();
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

    vec3 E_23 = q.v_01() - q.v_11();
    vec3 E_21 = q.v_10() - q.v_11();
    vec3 P_prime = cross(P_prime,r.end, E_21);
    float det_prime = dot(det_prime,E_23, P_prime);
    if (std::abs(det_prime) < eps) return false;
    float inv_det_prime = float(1.0) / det_prime;
	vec3 T_prime = r.begin - q.v_11();
    float alpha_prime = dot(alpha_prime,T_prime, P_prime) * inv_det_prime;
    if (alpha_prime < float(0.0)) return false;
    vec3 Q_prime = cross(Q_prime,T_prime, E_23);
	float beta_prime = dot(beta_prime,r.end, Q_prime) * inv_det_prime;
    if (beta_prime < float(0.0)) return false;
  }

  // Compute the ray parameter of the intersection point, and
  // reject the ray if it does not hit Q.

  t = dot(t,E_03, Q) * inv_det;
  if (t < float(0.0)) return false; 

  // Compute the barycentric coordinates of the fourth vertex.
  // These do not depend on the ray, and can be precomputed
  // and stored with the quadrilateral.  

  float alpha_11, beta_11;
  vec3 E_02 = q.v_11() - q.v_00();
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


bool Sphere::intersect_ray(Sphere& sphere,Ray &ray, float &t)
{
	float  root,root1,root2;
	float r = sphere.getRadius();
	vec3 dis = ray.begin - sphere.getCenter();
	float temp = dot(temp, dis, dis);

	float a = dot(a,ray.end, ray.end);
	float b = 2 * dot(b,ray.end, dis);
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
        t = root2;
		return true;
    }
    // else the intersection point is at root1
    else
    {
        t = root1;
        return true;
    }
}


bool Triangle::intersect_ray(Triangle& tr, Ray& ray, float& u, float& v, float& t)
{
	vec3 pvec,tvec,qvec;

	//get vertices of triangle
	vec3 v1 = tr.getVertex1();
	vec3 v2 = tr.getVertex2();
	vec3 v3 = tr.getVertex3();

	//finding the edges
	vec3 edge1, edge2;
	edge1 = subt(v2,v1);
	edge2 = subt(v3,v1);

	pvec = cross(pvec,ray.end,edge2);
	float det = dot(det,edge1,pvec);

#ifdef TEST_CULL
	if(det < EP)
		return false;
	
	//calculate distance from vertex1 to ray
	tvec = subt(ray.begin,v1);
	u = dot(u,tvec,pvec);

	if(u < 0 || u > det)
		return false;

	qvec = cross(qvec,tvec,edge1);
	v = dot(v,ray.end,qvec);

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
	  Sphere s =Sphere(center,radius);
	  slist[spherecount]=s;
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
	Triangle t = Triangle(vertex1,vertex2,vertex3);
	tlist[trianglecount]=t;
	trianglecount++;
	tflag=1;

	}
	
	count++;
	}
	totalobjcount = spherecount+trianglecount+quadcount;

}

