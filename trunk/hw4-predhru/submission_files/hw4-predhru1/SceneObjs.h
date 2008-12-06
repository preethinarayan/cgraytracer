#include "nv/nv_math.h"
#include "nv/nv_algebra.h"

class Ray
{
public:
	vec3 begin; 
	vec3 end;
	Ray();
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

//	virtual float Intersect(Ray_t *R) = 0;
//	virtual vec3 getColor(Ray_t *R, vec3 pt, int depth) = 0;
//
//	/* transformations routines */
//	virtual void transformNormal()=0;
//	virtual vec3 transformPoint(vec3 pt)=0;
//	
//	vec3 getTransformedNormal(mat4 transform, vec3 n);
//	virtual vec3 getNormal(vec3 pt) = 0;
//	
////	virtual Ray_t getReflectedRay(Ray *R, vec3 pt)=0;
//
//	Ray getTransmittedRay(Ray *R, vec3 pt, bool *doesRefract);
//	vec3 getRefractions(Ray *R, vec3 pt, int depth);
//	
//	Ray *inverseTransformRay(Ray *R, Ray *R_trans, mat4 M);
//	Ray *TransformRay(Ray *R, Ray *R_trans, mat4 M);
//	float getRayPlaneIntersection(vec3 v[3], Ray *R);
//	void convertRayToHomogenous(Ray *R, Ray_hom_t *R_hom);
//	vec4 getHomogenous(vec3 v);
//	vec3 getDehomogenous(vec4 v);

	/* lighting routines */
	//virtual vec3 getLiteColor(Light *light, vec3 light_direction,vec3 viewer_direction, vec3 pt) = 0;
	//vec3 getEmission(){return emission;}
	//vec3 getAmbient(){return ambient;}
	//vec3 calculateIllumination(Light *light, vec3 normal, vec3 light_direction, vec3 viewer_direction, vec3 pt);
	//float spotlight_effect(Light *light, vec3 direction_vector);
	//float calculateAttenuation(vec3 attenuation, float d);

	//virtual Ray_t getReflectedRay(Ray *R, vec3 pt) = 0;
	//virtual vec3 getReflections(Ray *R, vec3 pt, int depth) = 0;

};


class Quadrilateral : public SceneObjs
{
	private:
		vec3 vertices[4];
	public:
	Quadrilateral();
	Quadrilateral(vec3& v_00,vec3& v_10,vec3& v_11, vec3& v_01);	
	 vec3& v_00()  { return vertices[0]; }
	 vec3& v_10()  { return vertices[1]; }
	 vec3& v_11()  { return vertices[2]; }
	 vec3& v_01()  { return vertices[3]; }
	 bool intersect_ray(Quadrilateral& q,Ray& r, float& u, float& v, float& t);
};

class Triangle : public SceneObjs
{
private:
	vec3 vertices[3];
public:
	Triangle();
	Triangle(vec3 v1, vec3 v2, vec3 v3);
	vec3 getVertex1(){return vertices[0];};
	vec3 getVertex2(){return vertices[1];};
	vec3 getVertex3(){return vertices[2];};
	bool intersect_ray(Triangle& tr, Ray& ray, float& u, float& v, float& t);
};

class Sphere : public SceneObjs
{
private:
	float rad;
	vec3 center;

public:
	Sphere();
	Sphere(vec3 _center,float _rad);
	vec3 getCenter(){return center;}
	float getRadius(){return rad;};
	bool intersect_ray(Sphere& sphere, Ray& r, float& t);
};

class Scene
{
public:
	struct Vertex {
	double pos[3] ;
	} ;
	struct VertexNormal {
	double pos[3] ;
	double normal[3] ;
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
	Quadrilateral qlist[20];
	Triangle tlist[20];
	Sphere slist[20];


	void initScene()
	{
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

};