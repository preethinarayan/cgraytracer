#include "StdAfx.h"
#include "Triangle.h"

bool Triangle::setArgs(char **args)
{
	if( !args )
		return false;

	for(int i=0; i<3; i++)
	{
		if(!args[i])
			return false;
		v[i] = atoi(args[i]);
	}
	
	return true;
}

float Triangle::Intersect(Ray_t *R)
{
  // Find the normal
  vec3 n;
  vec3 A = Vert->list[v[0]];
  vec3 B = Vert->list[v[1]];
  vec3 C = Vert->list[v[2]];

  vec3 v1 = B - A;
  vec3 v2 = C - B;
  n = cross(n,v1,v2);
  n.normalize();

  nv_scalar t;
  nv_scalar An;
  nv_scalar P0n;
  nv_scalar P1n;

  An = dot(An,A,n);
  P0n = dot(P0n,R->P0,n);
  P1n = dot(P1n,R->P1,n);

  t = (An - P0n)/P1n;

  // Point
//    R.P1.normalize();

  vec3 P;
 /* P.x = R->P1.x * t;
  P.y = R->P1.y * t;
  P.z = R->P1.z * t;

  P += R->P0;*/
  P = R->P0 + t*R->P1;

  v1 = P - A;
  v2 = B - A;
  vec3 v3 = C - A;

  float determinant;

  determinant = (v2.x * v3.y) - (v2.y * v3.x);

  float a11,a12,a21,a22;

  a11 = v3.y/determinant;
  a22 = v2.x/determinant;
  a21 = -v3.x/determinant;
  a12 = -v2.y/determinant;

  float beta = a11 * v1.x + a12 * v1.y;
  float gamma = a21 * v1.x + a22 * v1.y;

  if((beta >= 0.0 && beta <= 1.0f) && (gamma >= 0.0 && gamma <= 1.0f) && ((beta + gamma) <= 1.0f) )
  {
      return t;
  }
  else
  {
      return -1.0;
  }
}

Triangle::~Triangle(void)
{
}
