#include "StdAfx.h"
#include "Sphere.h"

bool Sphere::setArgs(char **args, Shading *shading)
{
	if( !args )
		return false;

	for(int i=0; i<4; i++)
	{
		if(!args[i])
			return false;
	}

	C.x = atof(args[0]);
	C.y = atof(args[1]);
	C.z = atof(args[2]);
	r = atof(args[3]);
	// Update Starts
	if(shading)
	{
		if(shading->isAmbientSet)
		{
			ambient = shading->ambient;
		}
		if(shading->isSpecularSet)
		{
			specular = shading->specular;
		}
		if(shading->isDiffuseSet)
		{
			diffuse = shading->diffuse;
		}
		if(shading->isEmissionSet)
		{
			emission = shading->emission;
		}
		if(shading->isShininessSet)
		{
			shininess = shading->shininess;
		}
	}
	// Update Ends

	return true;
}


/*********************************************************************************
 * Intersections
 *********************************************************************************/
/* ignore negative 't's */
float Sphere::Intersect(Ray_t *R)
{
	/* apply ray transformations */
	Ray_t R_trans, R_trans2;
	inverseTransformRay(R, &R_trans, transform);

	R_trans2.P0=R_trans.P0;
	R_trans2.P1=R_trans.P1;
	vec3 temp = R_trans.P0-C;	/* used several times */

	float t_scale = nv_norm(R_trans.P1);

	R_trans.P1.normalize();
	/* calc quadratic coefficients */
	float a = 1.0;//dot(R->P1, R->P1);
	float b = 2*dot(R_trans.P1, temp);
	float c = dot(temp, temp) - r*r;

	/* compute determinant of quadratic */
	float D2 = (b*b - 4*a*c);	/* det squared */

	if(D2<0){		/* does not intersect */
		return -1;
	} else if(D2 == 0){
		return(-1*b/(2*a)*nv_norm(R_trans2.P1));
	}
	float t1 = (-1*b+sqrt(D2))/(2*a);
	float t2 = (-1*b-sqrt(D2))/(2*a);

	if(t1<0)		/* reject negatives, if second is negative it is ignored */
		return t2*1/nv_norm(R_trans2.P1);
	else if(t2<0)
		return t1*1/nv_norm(R_trans2.P1);
	else	/* both positive */
	{
		if(t1<t2)
			return t1*1/nv_norm(R_trans2.P1);
		else 
			return t2*1/nv_norm(R_trans2.P1);
	}
}

vec3 Sphere::getNormal(vec3 pt)
{
	vec3 normal = pt - C;
	normal.normalize();
	normal = getTransformedNormal(transform, normal);
	return normal;
}

/*********************************************************************************
 * Reflections
 *********************************************************************************/
//Ray_t Sphere::getReflectedRay(Ray_t *R, vec3 pt)
//{
//	vec3 n = pt - C;
//	/* use only directions - calc reflec direction*/
//	vec3 reflec_dir = R->P1 - 2*(R->P1*n)*n;
//	reflec_dir.normalize();
//
//	/* 
//	 * construct reflected ray
//	 * reflec ray passes through pt incident hits surface
//	 */
//	Ray_t reflec;
//	reflec.P0 = pt;			
//	reflec.P1 = reflec_dir;
//
//	return reflec;
//}

/*Ray_t Sphere::getReflections)
{
	/* transform "incoming ray" to OS - inv(M)*R */	
	
	/*	Ray_t R_trans;
	inverseTransformRay(R, &R_trans, transform);

	Ray_t reflec=getReflectedRay(R, pt);

	/* 
	 * send ray to all other objects in the scene
	 * transform outgoing ray to world space
	 */
/*	Ray_t reflecTrans;
	TransformRay(&reflec, &reflecTrans, transform);

	vec3 color = S->getRayIntersection(&reflecTrans, --depth);*/
//}
/*********************************************************************************
 * Light Interactions
 *********************************************************************************/
vec3 Sphere::getLiteColor( Light *light,vec3 light_direction, vec3 viewer_direction, vec3 pt)
{
	vec3 color;
	vec3 normal = getNormal(pt);
	color = calculateIllumination(light, normal, light_direction, viewer_direction, pt);
	return color;
}

vec3 Sphere::getColor(Ray_t *R, vec3 pt, int depth)
{
	if(depth == 0)
		return vec3(255, 0, 0);

	/* contribution of light sources */
	vec3 color(0.0,0.0,0.0);
	int visible;

	/* though R.P1 will give the same value, I am doing the calculations again just for clarity */
	vec3 viewer_direction = R->P0 - pt;
	viewer_direction.normalize();

	Ray_t RayToLight;
	RayToLight.P0 = pt;
	for(int k = 0 ; k < S->lightObjs.size() ; k++)
	{
		/* get the light direction. lightObjs[i] - current point; ==> P1 */
		RayToLight.P1 = S->lightObjs[k]->position - pt;
		RayToLight.P1.normalize();
		/* To hack the floating point errors */
		RayToLight.P0 += 0.1 * RayToLight.P1;
  	    visible = S->Visible(&RayToLight,S->lightObjs[k]->position)? 1 : 0;
		color += visible 
				* getLiteColor(	S->lightObjs[k],
								RayToLight.P1,
								viewer_direction,
								pt);

	}
	if(S->lightObjs.size() > 0)
	{
		color += emission + (S->lightObjs[0]->getGlobalAmbient() * this->ambient);
	}
	else
	{
		color += emission + this->ambient;
	}

    /* cause for reflections */
    if((this->specular.x != 0 || this->specular.x != 0 || this->specular.x != 0) && shininess == 128)
    {
        vec3 r_color = getReflections(R, pt, depth);
		r_color = r_color / 255.0;
        color += r_color * this->specular;
    }

	if(isRefractive)
	{
		vec3 t_color = getRefractions(R, pt, depth)/255.0;
		color.x += this->transmit.x * t_color.x;
		color.y += this->transmit.y * t_color.y;
		color.z += this->transmit.z * t_color.z;
	}

	color = 255.0 * color;
	if(color.x > 255.0)
		color.x = 255.0;
	if(color.y > 255.0)
		color.y = 255.0;
	if(color.z > 255.0)
		color.z = 255.0;

	return color;
}

/*********************************************************************************
 * Transformations
 *********************************************************************************/
vec3 Sphere::transformPoint(vec3 pt)
{
	vec4 v=getHomogenous(pt);
	v=transform*v;
	return getDehomogenous(v);
}

Sphere::Sphere(void)
{
	transform=Transformation->getCurrTransform(); 
	isMirror=false;

	isRefractive = false;
	if(shading->isRefractionEnabled())
	{
		isRefractive = true;
		ref_eta1 = shading->ref_eta1;
		ref_eta2 = shading->ref_eta2;
		transmit = shading->transmit;
	}
}

Sphere::~Sphere(void)
{
}

Ray_t Sphere::getReflectedRay(Ray_t *R, vec3 pt)
{
    vec3 N = getNormal(pt);
    vec3 I = pt - R->P0;//R->P1;
    I.normalize();
//	reflection_direction = -light_direction + (2 * dot(light_direction,normal) * normal);

    vec3 reflec_dir = -(2* dot(I,N)*N) + I;
    reflec_dir.normalize();

    /*
     * construct reflected ray
     * reflec ray passes through pt incident hits surface
     */
    Ray_t reflec;
    reflec.P0 = pt;  
	reflec.P0 += 0.1*reflec.P1;
    reflec.P1 = reflec_dir;

    return reflec;
}

vec3 Sphere::getReflections(Ray_t *R, vec3 pt, int depth)
{
    /*
     * perform no further recursion,
     * you are the last to ever be colored
     */
    if(depth == 0)
        return vec3(0.0, 0.0, 0.0);

    /* transform "incoming ray" to OS - inv(M)*R */   

	Ray_t reflec=getReflectedRay(R, pt);

    /*
     * send ray to all other objects in the scene
     */
    vec3 color = S->getRayIntersection(&reflec, --depth, false);
    return color;
}


