// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once


#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <stdio.h>
#include <tchar.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "nv_math.h"
#include "nv_algebra.h"

#define INF 9999999.00

typedef struct{
	vec3 P0; vec3 P1;
} Ray_t;

/* homogenous Ray */
typedef struct{
	vec4 P0; vec4 P1;
} Ray_hom_t;

#include "Image.h"
#include "Camera.h"
// Update Starts
#include "Shading.h"
// Update Ends
extern Shading *shading;

#include "Transform.h"

extern Transform *Transformation;

#include "Vertexes.h"
// Update Starts
#include "Light.h"
// Update Ends
#include "SceneObjects.h"
#include "Sphere.h"
#include "Triangle.h"
#include "Scene.h"
#include "Parser.h"
#include "Command.h"

extern Command *C;
extern Camera *Cam;
extern Image *I;
extern Scene *S;
extern Vertexes *Vert;

#define pi 3.1428571428571428571428571428571
#define degToRad (nv_pi/180)
// Update Starts
#define max(a,b) (((a) > (b)) ? (a) : (b))
// Update Ends
#define RECURSION_DEPTH 5
// TODO: reference additional headers your program requires here
