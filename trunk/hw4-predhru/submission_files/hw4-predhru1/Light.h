#include "nv/nv_math.h"


// This class is defined as a global container for the shading components
// If after setting any properties of this class, if we encounter any object/light, we will invalidate this class
// after copying the contains of this class to the corresponding object/light.

/*{
	TYPE_AMBIENT, 
	TYPE_SPECULAR, 
	TYPE_DIFFUSE, 
	TYPE_EMISSION, 
	TYPE_SHININESS, 
	TYPE_ATTENUATION, 
	TYPE_SPOT,
	TYPE_REFRACTION
*/

class Shade
{
public:
	vec3 specular;
	vec3 diffuse;
	vec3 ambient;
	vec3 emission;
	float shininess;
	bool isSpecularSet;
	bool isDiffuseSet;
	bool isAmbientSet;
	bool isEmissionSet;
	bool isShininessSet;
	int shadingType[8];
	/* refraction */
	bool isRefractionEnable;
	float ref_eta1;
	float ref_eta2;
	vec3 transmit;


	vec3 attenuation;
	// new command - spot x y z spot_cutoff spot_exponent // x y z specifies spot_direction
	vec3 spot_direction;
	float spot_cutoff;
	float spot_exponent;
public:
	Shade();
	bool setvalues(char **args, int type);
	void disableRefraction(){isRefractionEnable=false;};
	bool isRefractionEnabled(){return isRefractionEnable;};

};

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
	vec3 spot_direction;
	float spot_cutoff;
	float spot_exponent;

	// 0 for directional; 1 for point
	// Similar to OpenGL (w = 1 for point and w = 0 for directional)
	int lightType;

	Light();
	~Light();
	bool setvalues(float position[3],float values[3], Shade *shading, int type);
	vec3 getGlobalAmbient(){return globalambient;}
};



