#pragma once

#define MAX_VERT 100
class Vertexes
{
	vec3 v[MAX_VERT];
	vec3 vNorm[MAX_VERT];	/* vertices */
	vec3 norm[MAX_VERT];	/* corresponding normals */
	
public:	
	
	int curr;
	int ncurr;
	int nMax;
	int nNorms;

	Vertexes(void)
	{
		curr = 0;
		ncurr = 0;
	}
	bool setMaxVertexes(char **args);
	bool setMaxVertexNormals(char **args);
	bool setNextVertex(char **args);
	bool setNextVertexNormal(char **args);

	vec3 getVertex(int i){ return v[i]; };
	vec3 getVertexNormal(int i){ return vNorm[i]; };
	vec3 getVertexNormalNormal(int i){ return norm[i]; };

public:
	~Vertexes(void);
};
