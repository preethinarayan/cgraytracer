#pragma once

class Camera
{
	vec3 eye;
	vec3 center;
	vec3 up;
	float fov;
	int width;
	int height;
	vec3 u, v, w;

	void initCoordinates();
public:
	Camera();
	vec3 calcRays(int i,int j);
	void setImageParameters(int width,int height);
	bool setArgs(char **args);
	vec3 executeCommand(int i, int j){return (calcRays(i,j));};
	vec3 getEye(){return eye;};
public:
	~Camera(void);
};
