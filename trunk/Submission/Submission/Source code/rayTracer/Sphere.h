#pragma once
#include "SceneObjects.h"

class Sphere :
	public SceneObjects
{
	vec3 C;
	float r;
	mat4 transform;
	bool isMirror;

	vec3 getNormal(vec3 pt);
public:
	float Intersect(Ray_t *R);

	vec3 getLiteColor(Light *light,vec3 light_direction,vec3 viewer_direction, vec3 pt);
	vec3 getColor(Ray_t *R, vec3 pt, int depth);
	bool setArgs(char **args, Shading *shading);

	/* transformations */
	void transformNormal(){};
	vec3 transformPoint(vec3 pt);

	/* reflections */
//	Ray_t getReflectedRay(Ray_t *R, vec3 pt);
	Ray_t getReflectedRay(Ray_t *R, vec3 pt);
	vec3 getReflections(Ray_t *R, vec3 pt, int depth);

public:
	Sphere(void);
	~Sphere(void);
};
