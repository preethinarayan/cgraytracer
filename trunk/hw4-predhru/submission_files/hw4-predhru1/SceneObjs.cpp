
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
	SceneObjects(void);
	virtual float Intersect(Ray_t *R) = 0;
	virtual vec3 getColor(Ray_t *R, vec3 pt, int depth) = 0;

	/* transformations routines */
	virtual void transformNormal()=0;
	virtual vec3 transformPoint(vec3 pt)=0;
	
	vec3 getTransformedNormal(mat4 transform, vec3 n);
	virtual vec3 getNormal(vec3 pt) = 0;
	
//	virtual Ray_t getReflectedRay(Ray_t *R, vec3 pt)=0;

	Ray_t getTransmittedRay(Ray_t *R, vec3 pt, bool *doesRefract);
	vec3 getRefractions(Ray_t *R, vec3 pt, int depth);
	
	Ray_t *inverseTransformRay(Ray_t *R, Ray_t *R_trans, mat4 M);
	Ray_t *TransformRay(Ray_t *R, Ray_t *R_trans, mat4 M);
	float getRayPlaneIntersection(vec3 v[3], Ray_t *R);
	void convertRayToHomogenous(Ray_t *R, Ray_hom_t *R_hom);
	vec4 getHomogenous(vec3 v);
	vec3 getDehomogenous(vec4 v);

	/* lighting routines */
	virtual vec3 getLiteColor(Light *light, vec3 light_direction,vec3 viewer_direction, vec3 pt) = 0;
	vec3 getEmission(){return emission;}
	vec3 getAmbient(){return ambient;}
	vec3 calculateIllumination(Light *light, vec3 normal, vec3 light_direction, vec3 viewer_direction, vec3 pt);
	float spotlight_effect(Light *light, vec3 direction_vector);
	float calculateAttenuation(vec3 attenuation, float d);

	virtual Ray_t getReflectedRay(Ray_t *R, vec3 pt) = 0;
	virtual vec3 getReflections(Ray_t *R, vec3 pt, int depth) = 0;

public:
	~SceneObjects(void);
};

