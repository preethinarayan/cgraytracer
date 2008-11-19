#pragma once

class Light
{
public:
	vec3 position;
	vec3 ambient;
	vec3 emission;
	vec3 specular;
	vec3 diffuse;
	vec3 globalambient;
	vec3 attenuation;

	// Spotlight effects
	// new command - spot x y z spot_cutoff spot_exponent // x y z specifies spot_direction
	vec3 spot_direction;
	float spot_cutoff;
	float spot_exponent;

	
	// 0 for directional; 1 for point
	// Similar to OpenGL (w = 1 for point and w = 0 for directional)
	int lightType;

	Light();
	~Light();
	bool setArgs(char **args, Shading *shading, int type);
	vec3 getGlobalAmbient(){return globalambient;}
};
