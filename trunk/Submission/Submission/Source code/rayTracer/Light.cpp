#include "stdafx.h"
#include "Light.h"

bool Light::setArgs(char **args, Shading *shading, int type)
{
	if( !args )
		return false;

	for(int i=0; i<6; i++)
	{
		if(!args[i])
			return false;
	}

	position = vec3(atof(args[0]),atof(args[1]),atof(args[2]));
	ambient = vec3(atof(args[3]),atof(args[4]),atof(args[5]));

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

Light::~Light()
{
}