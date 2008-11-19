#include "stdafx.h"
#include "Shading.h"

Shading::Shading()
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
}

bool Shading::setArgs(char **args,shadingType type)
{
	if( !args )
		return false;

	if(type == TYPE_SHININESS)
	{
		if(!args[0])
			return false;
	}
	else
	{
		int argc = (type == TYPE_SPOT)? 5 : 3;
		for(int i=0; i<argc; i++)
		{
			if(!args[i])
				return false;
		}
	}

	switch(type)
	{
	case TYPE_AMBIENT:
		isAmbientSet = true;
		ambient =  vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case TYPE_SPECULAR:
		isSpecularSet = true;
		specular = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case TYPE_DIFFUSE:
		isDiffuseSet = true;
		diffuse = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case TYPE_EMISSION:
		isEmissionSet = true;
		emission = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case TYPE_SHININESS:
		isShininessSet = true;
		shininess = (float)atof(args[0]);
		break;
	case TYPE_ATTENUATION:
		attenuation = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		break;
	case TYPE_SPOT:
		spot_direction = vec3((float)atof(args[0]),(float)atof(args[1]),(float)atof(args[2]));
		spot_cutoff = (float)atof(args[3]);
		spot_exponent = (float)atof(args[4]);
		break;	
	case TYPE_REFRACTION:
		ref_eta1 = (float)atof(args[0]);
		ref_eta2 = (float)atof(args[1]);
		transmit = vec3((float)atof(args[2]),(float)atof(args[3]),(float)atof(args[4]));
		isRefractionEnable=true;
		break;
	}
	return true;
}