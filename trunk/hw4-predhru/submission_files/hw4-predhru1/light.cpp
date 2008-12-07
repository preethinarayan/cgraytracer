#include "Light.h"
#include "nv/nv_math.h"
#include "nv/nv_algebra.h"

bool Light::setvalues(float _position[3],float _values[3], Shade *shading, int type)
{
	
	position = vec3(_position[0],_position[1],_position[2]);
	ambient = vec3(_values[0],_values[1],_values[2]);

	lightType = type;

	if(shading)
	{
		if(shading->isAmbientSet)
		{
			globalambient = shading->ambient;
		}
		if(shading->isSpecularSet)
		{
			specular = shading->specular;
		}
		if(shading->isDiffuseSet)
		{
			diffuse = shading->diffuse;
		}
		attenuation = shading->attenuation;
		spot_direction = shading->spot_direction;
		spot_cutoff = shading->spot_cutoff;
		spot_exponent = shading->spot_exponent;
	}
	else
	{
		return false;
	}
	return true;
}

Light::Light()
{
	ambient = vec3(0.0,0.0,0.0);
	diffuse = vec3(0.0,0.0,0.0);
	specular = vec3(0.0,0.0,0.0);
	emission = vec3(0.0,0.0,0.0);
	globalambient = vec3(0.2,0.2,0.2);
}

Shade::Shade()
{
	isSpecularSet = false;
	isDiffuseSet = false;
	isAmbientSet = false;
	isEmissionSet = false;
	isShininessSet = false;
	isRefractionEnable = false;
	shininess = 0;
	emission = vec3(0,0,0);
	attenuation = vec3(1,0,0);
	// Default values for spot light as in OpenGL
	spot_direction = vec3(0.0,0.0,-1.0);
	spot_cutoff = 180.0;
	spot_exponent = 0.0;
	for(int i=0;i<8;i++) 
			shadingType[i]=i;
}

bool Shade::setvalues(char **args,int type)
{
	if( !args )
		return false;

	if(type == 4)
	{
		if(!args[0])
			return false;
	}
	else
	{
		int argc = (type == 6)? 5 : 3;
		for(int i=0; i<argc; i++)
		{
			if(!args[i])
				return false;
		}
	}

	switch(type)
	{
	case 0:
		isAmbientSet = true;
		ambient =  vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case 1:
		isSpecularSet = true;
		specular = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case 2:
		isDiffuseSet = true;
		diffuse = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case 3:
		isEmissionSet = true;
		emission = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case 4:
		isShininessSet = true;
		shininess = (float)atof(args[0]);
		break;
	case 5:
		attenuation = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case 6:
		spot_direction = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		spot_cutoff = (float)atof(args[3]);
		spot_exponent = (float)atof(args[4]);
		break;	
	case 7:
		ref_eta1 = (float)atof(args[0]);
		ref_eta2 = (float)atof(args[1]);
		transmit = vec3((float)atof(args[2]),(float)atof(args[3]),(float)atof(args[4]));
		isRefractionEnable=true;
		break;
	}
	return true;
}