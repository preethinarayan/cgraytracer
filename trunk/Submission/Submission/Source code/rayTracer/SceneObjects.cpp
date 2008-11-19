#include "StdAfx.h"
#include "SceneObjects.h"

/*******************************************************************
 * Lighting
 *******************************************************************/
// Update Starts

vec3 SceneObjects::calculateIllumination(Light *light,vec3 normal, vec3 light_direction, vec3 viewer_direction,vec3 pt)
{
	vec3 color;

	vec3 light_specular = light->specular;
	vec3 light_diffuse = light->diffuse;
	vec3 light_ambient = light->ambient;

	vec3 direction_vector = pt - light->position;
	// distance between light source and vertex
	float d = direction_vector.norm();
	vec3 light_attenuation = light->attenuation;

	vec3 reflection_direction;
	nv_scalar specularfactor;
	nv_scalar diffusefactor;

	reflection_direction = -light_direction + (2 * dot(light_direction,normal) * normal);
	
	specularfactor = max(dot(reflection_direction,viewer_direction),0.0);
	specularfactor = pow(specularfactor,shininess);
	diffusefactor = max(dot(light_direction,normal),0.0);
	
	color = (calculateAttenuation(light_attenuation,d) * spotlight_effect(light,direction_vector)) * ((light_ambient * this->ambient) + (specularfactor * light_specular * this->specular) + (diffusefactor * light_diffuse * this->diffuse));
	return color;
}

float SceneObjects::calculateAttenuation(vec3 attenuation, float d)
{
	float value;
	value = attenuation.x + (attenuation.y * d) + (attenuation.z * d * d);
	value = 1/value;
	return value;
}

float SceneObjects::spotlight_effect(Light *light,vec3 direction_vector)
{
	// If the spot light is actually not a spot light ;)
	if(light->spot_cutoff >= 180.0)
	{
		return 1.0;
	}
	float cone;
	direction_vector.normalize();
	light->spot_direction.normalize();
	
	cone = dot(direction_vector,light->spot_direction);
	// If the vertex is outside the spotlight
	if(cone < cosf(degToRad * light->spot_cutoff))
	{
		return 0.0;
	}
	cone = max(cone,0.0);
	cone = pow(cone, light->spot_exponent);
	return cone;
}

Ray_t SceneObjects::getTransmittedRay(Ray_t *R, vec3 pt, bool *doesRefract)
{
	vec3 N = this->getNormal(pt);
	vec3 I = R->P1;
	I.normalize();

	float eta1, eta2;

	if(dot(N, I) < 0)
	{
		eta1 = ref_eta2;
		eta2 = ref_eta1;
		N = -1.0*N;
	}
	else
	{
		eta1 = ref_eta1;
		eta2 = ref_eta2;
	}
	float n = eta1/eta2;
	float cosI = dot(N, I);
	float sinT2 = n*n*(1.0f - cosI*cosI);
	Ray_t Trans;

	if(sinT2 <= 1.0)
	{
		vec3 T = n*I - (n + sqrt(1.0 - sinT2))*N;

		T.normalize();

		Trans.P0 = pt;			
		Trans.P1 = T;
		Trans.P0 += 0.01*Trans.P1;
		*doesRefract = true;
	}
	else
	{
		*doesRefract = false;
	}
	return Trans;
}

vec3 SceneObjects::getRefractions(Ray_t *R, vec3 pt, int depth)
{
	/* 
	 * perform no further recursion, 
	 * you are the last to ever be colored
	 */
	if(depth == 0)
		return vec3(0.0, 0.0, 0.0);

	/* transform "incoming ray" to OS - inv(M)*R */	
	bool doesRefract;
	Ray_t Trans=getTransmittedRay(R, pt, &doesRefract);

	/* 
	 * send ray to all other objects in the scene
	 */
	vec3 color = vec3(0.0, 0.0, 0.0);
	if(doesRefract)
		color = S->getRayIntersection(&Trans, --depth, false);
	else
		color = S->getRayIntersection(R, --depth, true);
	return color;
}


/*******************************************************************
 * Transformations
 *******************************************************************/

vec4 SceneObjects::getHomogenous(vec3 v)
{ return vec4(v.x, v.y, v.z, 1.0); }

vec3 SceneObjects::getDehomogenous(vec4 v)
{ return vec3(v.x/v.w, v.y/v.w, v.z/v.w); }

void SceneObjects::convertRayToHomogenous(Ray_t *R, Ray_hom_t *R_hom)
{
	R_hom->P0.x=R->P0.x;
	R_hom->P0.y=R->P0.y;
	R_hom->P0.z=R->P0.z;
	R_hom->P0.w=1.0;

	R_hom->P1.x=R->P1.x;
	R_hom->P1.y=R->P1.y;
	R_hom->P1.z=R->P1.z;
	R_hom->P1.w=0.0;

	return;
}

Ray_t *SceneObjects::inverseTransformRay(Ray_t *R, Ray_t *R_trans, mat4 M)
{
	mat4 invM;
	invM=invert(invM, M);

	Ray_hom_t R_hom;
	convertRayToHomogenous(R, &R_hom);

	/* apply transformations */
	R_hom.P0 = invM*R_hom.P0;
	R_hom.P1 = invM*R_hom.P1;

	/* set individual coordinates, as 'w' is 0 for a ray */
	R_trans->P0 = vec3(R_hom.P0);
	R_trans->P1 = vec3(R_hom.P1);

	return R_trans;
}

Ray_t *SceneObjects::TransformRay(Ray_t *R, Ray_t *R_trans, mat4 M)
{
	Ray_hom_t R_hom;
	convertRayToHomogenous(R, &R_hom);

	/* apply transformations */
	R_hom.P0 = M*R_hom.P0;
	R_hom.P1 = M*R_hom.P1;

	/* set individual coordinates, as 'w' is 0 for a ray */
	R_trans->P0 = vec3(R_hom.P0);
	R_trans->P1 = vec3(R_hom.P1);

	return R_trans;
}

vec3 SceneObjects::getTransformedNormal(mat4 transform, vec3 n)
{
	mat4 M, transpM;
	M=invert(M, transform);
	transpM=transpose(transpM, M);
	/* normal transformations : n_trans = (inv(M))'.n */
	vec4 n_trans = getHomogenous(n);
	n_trans = transpM*n_trans;
	return(getDehomogenous(n_trans));
}

/*******************************************************************
 * Intersections
 *******************************************************************/
float SceneObjects::getRayPlaneIntersection(vec3 v[3], Ray_t *R)
{
   // Find the normal
	vec3 n;
	vec3 A = v[0];
	vec3 B = v[1];
	vec3 C = v[2];

	vec3 v1 = B - A;
	vec3 v2 = C - A;
	n = cross(n,v2,v1);
	n.normalize();

	nv_scalar t;
	nv_scalar An;
	nv_scalar P0n;
	nv_scalar P1n;

	An = dot(An,A,n);
	P0n = dot(P0n,R->P0,n);
	P1n = dot(P1n,R->P1,n);

	t = (An - P0n)/P1n;

	return t;
}
/*******************************************************************
 * Housekeeping
 *******************************************************************/
SceneObjects::SceneObjects(void)
{
	ambient = vec3(0,0,0);
	diffuse = vec3(0,0,0);
	specular = vec3(0,0,0);
	emission = vec3(0,0,0);
	shininess = 0;
}

SceneObjects::~SceneObjects(void)
{
}

