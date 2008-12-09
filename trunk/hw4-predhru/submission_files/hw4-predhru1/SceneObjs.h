#include "nv/nv_math.h"
#include "nv/nv_algebra.h"
#include <math.h>
#include "Light.h"
#include <vector>
#define vector std::vector
#define maxVerts 100

class Ray
{
public:
	vec3 begin; 
	vec3 end;
	Ray();
};

class RayHomo
{
public:
	vec4 begin; 
	vec4 end;
	RayHomo();
};

class Scene;

class CameraRay
{
private:
	vec3 u,v,w;
	
	float fov;

public:
	vec3 eye;
	vec3 center;
	vec3 up;
	CameraRay();
	CameraRay(vec3 eye, vec3 center, vec3 up, float _fov);
	vec3 calcRays(Scene *scene,int i,int j);
	vec3 getEye(){return eye;}
	Ray ray;
	void generateRays(Scene *scene);
	vec3 getRayIntersection(Ray *ray,int depth, bool secLargest,Scene *scene);
	bool Intersection(Ray *R, vec3 *pt, bool secLargest,Scene *scene);
};

class SceneObjs
{
protected:
	/* material properties */
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	vec3 emission;
	float shininess;
	
	/* refractive index etc 
	bool isRefractive;
	float ref_eta1;
	float ref_eta2;
	vec3 transmit;*/

public:
	SceneObjs() { 
	ambient = vec3(0,0,0);
	diffuse= vec3(0,0,0);
	specular= vec3(0,0,0);
	emission= vec3(0,0,0);
	shininess = 0.0;
	}
//	virtual float Intersect(Ray_t *R) = 0;
//	virtual vec3 getColor(Scene* scene, Ray *ray, vec3 pt, int depth) = 0;
//
//	/* transformations routines */	
	vec3 TransformNormal(mat4 trans, vec3 normal);
	virtual vec3 getNormal(vec3 point) = 0;
	//virtual void normalTransform()=0;
//	
////	virtual Ray_t getReflectedRay(Ray *R, vec3 pt)=0;
//
//	Ray getTransmittedRay(Ray *R, vec3 pt, bool *doesRefract);
//	vec3 getRefractions(Ray *R, vec3 pt, int depth);
//	
	Ray inverseTransformRay(Ray &ray, Ray &rtrans, mat4 M);
//	Ray *TransformRay(Ray *R, Ray *R_trans, mat4 M);
	void convertRayToHomogenous(Ray &ray, RayHomo &rtrans);
	vec4 getHomogenous(vec3 v);
	vec3 getDehomogenous(vec4 v);
	virtual bool intersect_ray(Ray &ray,float &u,float &v, float &t)=0;
	virtual vec3 transformPoint(vec3 pt)=0;
	/* lighting routines */
	vec3 getEmission(){return emission;}
	vec3 getAmbient(){return ambient;}
	float spotlight_effect(Light *light, vec3 dir);
	float calculateAttenuation(vec3 attenuation, float distance);
	vec3 getColor(Scene *scene,Ray *ray, vec3 pt, int depth,bool seclargest);
	//virtual Ray_t getReflectedRay(Ray *R, vec3 pt) = 0;
	virtual vec3 getReflection(Ray *ray,int depth,bool secLargest,Scene *scene,vec3 pt) = 0;
	//vec3 getReflection(Ray *ray,int depth,bool secLargest,Scene *scene,vec3 pt);
};


class Quadrilateral : public SceneObjs
{
	private:
		vec3 vertices[4];
		mat4 transformation;
	public:
	Quadrilateral();
	Quadrilateral(vec3& v_00,vec3& v_10,vec3& v_11, vec3& v_01);	
	 vec3& v_00()  { return vertices[0]; }
	 vec3& v_10()  { return vertices[1]; }
	 vec3& v_11()  { return vertices[2]; }
	 vec3& v_01()  { return vertices[3]; }
	 vec3 transformPoint(vec3 pt);
	 bool intersect_ray(Ray& r, float& u, float& v, float& t);
	// vec3 getColor(Scene *scene,Ray *ray, vec3 pt, int depth);
};

class Triangle : public SceneObjs
{
private:
	vec3 vertices[3];
	vec3 norms[3];
	mat4 transformation;
public:
	Triangle();
	Triangle(vec3 v1, vec3 v2, vec3 v3);
	//void setNormals(vec3 n1, vec3 n2, vec3 n3);
	vec3 getVertex1(){return vertices[0];};
	vec3 getVertex2(){return vertices[1];};
	vec3 getVertex3(){return vertices[2];};
	vec3 transformPoint(vec3 pt);
	bool intersect_ray(Ray& ray, float& u, float& v, float& t);
	//vec3 getColor(Scene *scene,Ray *ray, vec3 pt, int depth);
	vec3 getNormal(vec3 point);
	//void normalTransform(){};
	void setparams(Shade *shade);
	float area( vec3 v1, vec3 v2, vec3 v3);
	vec3 getReflection(Ray *ray,int depth,bool secLargest,Scene *scene,vec3 pt);
	//void transNorm()


};

class Sphere : public SceneObjs
{
private:
	float rad;
	vec3 center;
	mat4 transformation;

public:
	Sphere();
	Sphere(vec3 _center,float _rad);
	vec3 getCenter(){return center;}
	float getRadius(){return rad;};
	vec3 transformPoint(vec3 pt);
	bool intersect_ray(Ray& r, float& u,float& v,float& t);
	//vec3 getColor(Scene *scene, Ray *ray, vec3 pt, int depth);
	void setparams(Shade *shade);
	vec3 getNormal(vec3 point);
	//void normalTransform(){};
	vec3 getReflection(Ray *ray,int depth,bool secLargest,Scene *scene,vec3 pt);
};



class Scene
{
public:
	struct Vertex {
	double pos[3];
	} ;
	struct VertexNormal {
	double pos[3];
	double normal[3];
	} ;
	
	Vertex * vert;
	VertexNormal * vertnorm;
	int maxverts, maxvertnorms;
	int curvert, curvertnorm;
	int maxdepth ;
	int lightnum ;
	double attenuation[3];
	FILE *inputfile ;
	int parsed;
	int spherecount,trianglecount,quadcount,totalobjcount;
	vec3 cameraval[3];
	float fov;
	vector<SceneObjs*> sobjects;
	int currIntersectionObj;
	char outputfile[300];	
	int width;
	int height;
	vector<Light*> lights;

	void initScene()
	{
	width = 0;
	height = 0;
	vert = 0;
	vertnorm = 0;
	maxverts = 0, maxvertnorms = 0;
	curvert = 0, curvertnorm = 0 ;
	maxdepth = 5 ;
	lightnum = 0 ;
	attenuation[0] = 1.0;
	attenuation[1] = 0.0;
	attenuation[2] = 0.0;
	parsed = 0 ;
	spherecount=0,trianglecount=0,quadcount=0,totalobjcount=0;
	cameraval[0] = vec3();
	cameraval[1] = vec3();
	cameraval[2] = vec3();
	fov = 0.0;

	}
	

	void parsefile(FILE *fp);
	bool isVisible(Ray *pointToLight,vec3 lightposition);

};