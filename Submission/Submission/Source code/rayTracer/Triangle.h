#pragma once
#include "stdafx.h"
#include "sceneobjects.h"

class Triangle :
	public SceneObjects
{
	int v[3];	/* vertex index */
	mat4 transform;
	vec3 verts[3];
	vec3 norms[3];
	bool hasNormals;

	float area( vec3 v1, vec3 v2, vec3 v3);
	bool isPointInTriangle(vec3 P, vec3 v[3]);
	vec3 getNormal(vec3 pt);
public:
	float Intersect(Ray_t *R);

	vec3 getLiteColor(Light *light,vec3 light_direction,vec3 viewer_direction, vec3 pt);
	vec3 getColor(Ray_t *R, vec3 pt, int depth);

	bool setArgs(char **args, Shading *shading);
	bool setArgsNormals(char **args);

	/* Transformations */
	void transformNormal();
	vec3 transformPoint(vec3 pt);

	/* Reflections */
//	Ray_t getReflectedRay(Ray_t *R, vec3 pt){return *R;};
	Ray_t getReflectedRay(Ray_t *R, vec3 pt);
	vec3 getReflections(Ray_t *R, vec3 pt, int depth);

public:
	Triangle(void);
	~Triangle(void);
};
