#include <stdio.h>
#include <time.h>
#include "Scene.h"
#include "Transform.h"
#include "TGAimage.h"
#include <stdio.h>
#include <math.h>

Scene *scene;
CameraRay *camera;

void sleep(unsigned int mseconds)
{
    clock_t goal = mseconds + clock();
    while (goal > clock());
}

void ReduceToUnit(float vector[3])					// Reduces A Normal Vector (3 Coordinates)
{									// To A Unit Normal Vector With A Length Of One.
	float length;							// Holds Unit Length
	// Calculates The Length Of The Vector
	length = (float)sqrt((vector[0]*vector[0]) + (vector[1]*vector[1]) + (vector[2]*vector[2]));

	if(length == 0.0f)						// Prevents Divide By 0 Error By Providing
		length = 1.0f;						// An Acceptable Value For Vectors To Close To 0.

	vector[0] /= length;						// Dividing Each Element By
	vector[1] /= length;						// The Length Results In A
	vector[2] /= length;						// Unit Normal Vector.
}


void calcNormal(float v[3][3], float out[3])				// Calculates Normal For A Quad Using 3 Points
{
	float v1[3],v2[3];						// Vector 1 (x,y,z) & Vector 2 (x,y,z)
	static const int x = 0;						// Define X Coord
	static const int y = 1;						// Define Y Coord
	static const int z = 2;						// Define Z Coord
	
	// Finds The Vector Between 2 Points By Subtracting
	// The x,y,z Coordinates From One Point To Another.

	// Calculate The Vector From Point 1 To Point 0
	v1[x] = v[0][x] - v[1][x];					// Vector 1.x=Vertex[0].x-Vertex[1].x
	v1[y] = v[0][y] - v[1][y];					// Vector 1.y=Vertex[0].y-Vertex[1].y
	v1[z] = v[0][z] - v[1][z];					// Vector 1.z=Vertex[0].y-Vertex[1].z
	// Calculate The Vector From Point 2 To Point 1
	v2[x] = v[1][x] - v[2][x];					// Vector 2.x=Vertex[0].x-Vertex[1].x
	v2[y] = v[1][y] - v[2][y];					// Vector 2.y=Vertex[0].y-Vertex[1].y
	v2[z] = v[1][z] - v[2][z];					// Vector 2.z=Vertex[0].z-Vertex[1].z

	// Compute The Cross Product To Give Us A Surface Normal
	out[x] = v1[y]*v2[z] - v1[z]*v2[y];				// Cross Product For Y - Z
	out[y] = v1[z]*v2[x] - v1[x]*v2[z];				// Cross Product For X - Z
	out[z] = v1[x]*v2[y] - v1[y]*v2[x];				// Cross Product For X - Y

	ReduceToUnit(out);						// Normalize The Vectors
}


void display() {
	
}

void init()
{
	vec3 eye = vec3(0,0,-5);
	vec3 center = vec3(0,0,0);
	vec3 up = vec3(0,1,0);
	scene = new Scene();
	camera=new CameraRay(eye,center,up,30.0);
	scene->generateRays(camera);
//	Plane sqaure(normal,5.0);
	
}

int main(int argc, char* argv[]) {
	
	
	return 0;
}
