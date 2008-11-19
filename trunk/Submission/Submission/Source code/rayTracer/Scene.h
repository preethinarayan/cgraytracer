
#pragma once

#include "SceneObjects.h"
// Update Starts
#include "Light.h"
// Update Ends
#include <vector> 

#define vector std::vector
typedef vector<SceneObjects*> SceneObjectsVector;
// Update Starts
typedef vector<Light*> LightsVector;
// Update Ends

class Scene
{
	/* model objects */
	SceneObjectsVector objs;
	int nObjs;
	int currIntersectionObj;

	/* lighting intersections */
	vec3 FindColor(vec3 pt, int depth);
	
	/* viewing intersections */
	bool Intersection(Ray_t *R, vec3 *pt, bool secLargest);

public:

	/* light objects */
	LightsVector lightObjs;
	int nLightObjs;

	Ray_t myR;

public:
	Scene(void)
	{
		nObjs = 0;
		nLightObjs = 0;
	}
	bool addObject(SceneObjects *obj);
	void traceScene();
	vec3 getRayIntersection(Ray_t *R, int depth, bool secLargest);

	bool addLight(Light *light);
	bool Visible(Ray_t *RayToLight,vec3 currentlightposition);

public:
	~Scene(void);
};
