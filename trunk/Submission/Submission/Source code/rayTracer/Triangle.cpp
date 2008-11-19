#include "StdAfx.h"
#include "Triangle.h"

bool Triangle::setArgs(char **args, Shading *shading)
{
	if( !args )
		return false;

	for(int i=0; i<3; i++)
	{
		if(!args[i])
			return false;
		v[i] = atoi(args[i]);
	}

	verts[0] = Vert->getVertex(v[0]);
	verts[1] = Vert->getVertex(v[1]);
	verts[2] = Vert->getVertex(v[2]);

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
		else
		{
			shininess = 0;
		}
	}
	// Update Ends
	return true;
}


bool Triangle::setArgsNormals(char **args)
{
	if( !args )
		return false;

	for(int i=0; i<3; i++)
	{
		if(!args[i])
			return false;
		v[i] = atoi(args[i]);
	}

	hasNormals = true;

	verts[0] = Vert->getVertexNormal(v[0]);
	verts[1] = Vert->getVertexNormal(v[1]);
	verts[2] = Vert->getVertexNormal(v[2]);

	norms[0] = Vert->getVertexNormalNormal(v[0]);
	norms[1] = Vert->getVertexNormalNormal(v[1]);
	norms[2] = Vert->getVertexNormalNormal(v[2]);

	return true;
}

/*********************************************************************************
 * Intersections
 *********************************************************************************/

float Triangle::area( vec3 v1, vec3 v2, vec3 v3)
{
	vec3 v;
	v = cross(v, (v2-v1), (v3-v1));

	return(v.norm()/2);
}

bool Triangle::isPointInTriangle(vec3 P, vec3 v[3])
{
	vec3 A = verts[0];
	vec3 B = verts[1];
	vec3 C = verts[2];

	float abc = area(A, B, C);
	float bcp = area(B, C, P);
	float cap = area(C, A, P);
	float abp = area(A,B,P);

	float alpha = bcp/abc;
	float beta = cap/abc;
	float gamma = abp/abc;
	float sum = alpha+beta+gamma;

	if(sum <= 1.0001f && sum >= 0.9999f)
	//if(sum == 1.000)
		return true;
	else
		return false;
}

float Triangle::Intersect(Ray_t *R)
{
	/* apply ray transformations */
	Ray_t R_trans;
	inverseTransformRay(R, &R_trans, transform);

	float t=getRayPlaneIntersection(verts, &R_trans);

 	vec3 P;
	P = R_trans.P0 + t*R_trans.P1;
	
	if(isPointInTriangle(P, verts))
	{
		return t;
	}
	else
	{
		return -1.0;
	} 
}

vec3 Triangle::getNormal(vec3 pt)
{
	vec3 A = verts[0];
	vec3 B = verts[1];
	vec3 C = verts[2];

	vec3 v1 = A - B;
	vec3 v2 = C - B;
	vec3 n = cross(n,v1,v2);
	n.normalize();

	n=getTransformedNormal(transform, n);

	return n;
}
/*********************************************************************************
 * Light Interactions
 *********************************************************************************/
vec3 Triangle::getLiteColor(Light *light, vec3 light_direction, vec3 viewer_direction, vec3 pt)
{
	vec3 color(0,0,0);
	vec3 normal = getNormal(pt);
	color = calculateIllumination(light, normal, light_direction, viewer_direction, pt);
	int i = 0;
	return color;
}

vec3 Triangle::getColor(Ray_t *R, vec3 pt, int depth)
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
		RayToLight.P0 += 0.2 * RayToLight.P1;
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
    if(this->specular.x != 0 || this->specular.x != 0 || this->specular.x != 0)
    {
        vec3 r_color = getReflections(R, pt, depth);
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

void Triangle::transformNormal()
{
	if(hasNormals)
	{
		mat4 M, transpM;
		M=invert(M, transform);
		transpM=transpose(transpM, M);
		vec4 n;
		/* normal transformations : n_trans = (inv(M))'.n */
		for(int i=0; i<3; i++)
		{
			n=getHomogenous(norms[i]);
			n=transpM*n;
			norms[i]=getDehomogenous(n);
		}
	}
}

vec3 Triangle::transformPoint(vec3 pt)
{
	vec4 v=getHomogenous(pt);
	v=transform*v;
	return getDehomogenous(v);
}

Triangle::Triangle(void)
{
	transform=Transformation->getCurrTransform(); 
	hasNormals=false;

	isRefractive = false;
	if(shading->isRefractionEnabled())
	{
		isRefractive = true;
		ref_eta1 = shading->ref_eta1;
		ref_eta2 = shading->ref_eta2;
		transmit = shading->transmit;
	}
}

Triangle::~Triangle(void)
{
}

Ray_t Triangle::getReflectedRay(Ray_t *R, vec3 pt)
{
    vec3 N = getNormal(pt);
    vec3 I = pt - R->P0;//R->P1;
    I.normalize();
    vec3 reflec_dir = -2*(I*N)*N + I;
    reflec_dir.normalize();

    /*
     * construct reflected ray
     * reflec ray passes through pt incident hits surface
     */
    Ray_t reflec;
    reflec.P0 = pt;  
    reflec.P1 = reflec_dir;
	reflec.P0 += 0.1*reflec.P1;

    return reflec;
}

vec3 Triangle::getReflections(Ray_t *R, vec3 pt, int depth)
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



