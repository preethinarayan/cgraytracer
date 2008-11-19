#include "StdAfx.h"
#include "Vertexes.h"

bool Vertexes::setMaxVertexes(char **args)
{
	if(!args)
		return false;
	nMax = atoi(args[0]);
	return true;
}

bool Vertexes::setMaxVertexNormals(char **args)
{
	if(!args)
		return false;
	nNorms = atoi(args[0]);
	return true;
}

bool Vertexes::setNextVertex(char **args)
{
	if(!args)
		return false;

	assert(curr < MAX_VERT);

	v[curr].x = atof(args[0]);
	v[curr].y = atof(args[1]);
	v[curr].z = atof(args[2]);

	curr++;
	return true;
}

bool Vertexes::setNextVertexNormal(char **args)
{
	if(!args)
		return false;

	assert(ncurr < MAX_VERT);

	vNorm[ncurr].x = atof(args[0]);
	vNorm[ncurr].y = atof(args[1]);
	vNorm[ncurr].z = atof(args[2]);

	if(args[3])	/* update normal information */
	{
		norm[ncurr].x = atof(args[3]);
		norm[ncurr].y = atof(args[4]);
		norm[ncurr].z = atof(args[5]);
	}
	ncurr++;
	return true;
}
