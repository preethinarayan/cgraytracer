#include "StdAfx.h"
#include "Command.h"

Shading *shading;
int n_cmd_tab = 25;
cmds cmd_tab[]={
				{"size",			CMD_SIZE,			2}, 
				{"camera",			CMD_CAMERA,			10},
				{"sphere",			CMD_SPHERE,			4},
				{"maxverts",		CMD_MAX_VERTS,		1},
				{"vertex",			CMD_VERTEX,			3},
				{"tri",				CMD_TRI,			3},
				{"translate",		CMD_TRANSLATE,		3},
				{"rotate",			CMD_ROTATE,			4},
				{"scale",			CMD_SCALE,			3},
				{"pushTransform",	CMD_PUSH,			0},
				{"popTransform",	CMD_POP,			0},
				{"maxvertnorms",	CMD_MAX_VERT_NORMS, 1},
				{"vertexnormal",	CMD_VERTEXNORMAL,	6},
				{"trinormal",		CMD_TRINORMAL,		3},
				{"directional",		CMD_DIRECTIONAL,	6},
				{"point",			CMD_POINT,			6},
				{"attenuation",		CMD_ATTENUATION,	3},
				{"ambient",			CMD_AMBIENT,		3},
				{"diffuse",			CMD_DIFFUSE,		3},
				{"specular",		CMD_SPECULAR,		3},
				{"shininess",		CMD_SHININESS,		1},
				{"emission",		CMD_EMISSION,		3},
				{"spot",			CMD_SPOT,			5},
				{"refraction",		CMD_REFRACTION,		5},
				{"refractDisable",	CMD_REFRACT_DISABLE,0}				
				};
				
Command* Command::pinstance = 0;

// Update Starts
Command::~Command()
{
	if(shading)
	{
		delete shading;
	}
}
// Update Ends

Command* Command::Instance()
{
	if(pinstance==0)
	{
		pinstance = new Command();
	}
	return pinstance;
}

void Command::clearArgs()
{
	int i;
	for(i=0;i<n_args;i++)
	{
		free(args[i]);
	}
	n_args = 0;
}

bool Command::setCommand(char *s)
{
	/* reset command parameters */
	cmd = CMD_NONE;
	clearArgs();
		
	int i;
	for(i=0; i<n_cmd_tab; i++)
		if(strcmp(cmd_tab[i].cmd_string, s)==0) break;

	if(i==n_cmd_tab)
		return false;
	else
		cmd = cmd_tab[i].cmd_name;
	return true;
}

/*
 * fails if number of args exceeds allowable
 * only send this function '\0' when the command is one
 * that requires *no* args, or else it will fail
 */
bool Command::setArg(char *s)
{
	bool ret = false;
		
	int i;
	for(i=0; i<n_cmd_tab; i++)
		if(cmd==cmd_tab[i].cmd_name) break;

	if( cmd_tab[i].cmd_n_args == 0 &&		/* zero arguments */
		n_args == cmd_tab[i].cmd_n_args && s[0]=='\0')
	{
		ret = true;
	}else if(i<n_cmd_tab && 
		n_args<cmd_tab[i].cmd_n_args && s[0]!='\0')	
	{
		args[n_args++] = strdup(s);
		ret = true;
	}
	
	return ret;
}

/* 
 *	this function is the only one really sensitive to the 
 *  intrinsics of how each command may need to be handled
 *  it deals with object creation and necessary initialization
 *  for all commands in configuration files 
 */
bool Command::updateCommandArgs()
{
	bool ret = false;
	bool isArgsSane = false;

	switch(cmd){
	
		case CMD_SIZE:
			
			if(n_args == 2)
			{
				ret = I->setSize(args);
				isArgsSane = true;
			}
			break;

		case CMD_CAMERA:

			if(n_args == 10)
			{
				ret = Cam->setArgs(args);
					//ret = Cam->executeCommand();

				isArgsSane = true;
			}
			break;

		case CMD_SPHERE:

			if(n_args == 4)
			{
				Sphere *Sph = new Sphere();
				if(ret = Sph->setArgs(args,shading))
				{
					ret = S->addObject(Sph);
				}
				else
				{
					delete Sph;
				}
				isArgsSane = true;
			}
			break;

		case CMD_MAX_VERTS:
			if(n_args == 1)
			{
				ret = Vert->setMaxVertexes(args);
				isArgsSane = true;
			}
			break;

		case CMD_VERTEX:
			if(n_args == 3)
			{
				ret = Vert->setNextVertex(args);
				isArgsSane = true;
			}
			break;

		case CMD_VERTEXNORMAL:
			if(n_args == 6)
			{
				ret = Vert->setNextVertex(args);
				isArgsSane = true;
			}
			break;


		case CMD_TRI:
			if(n_args == 3)
			{
				Triangle *Tri = new Triangle();
				// Update Starts
				if(ret = Tri->setArgs(args,shading))
				// Update Ends
				{
					ret = S->addObject(Tri);
				}
				else
				{
					delete Tri;
				}
				isArgsSane = true;
			}
			break;
		
		case CMD_TRANSLATE:
			if(n_args == 3)
			{
				ret=Transformation->translate(args);
				isArgsSane = true;
			}
			break;

		case CMD_ROTATE:
			if(n_args == 4)
			{
				ret=Transformation->rotate(args);
				isArgsSane = true;
			}
			break;

		case CMD_SCALE:
			if(n_args == 3)
			{
				ret=Transformation->scale(args);
				isArgsSane = true;
			}
			break;

		case CMD_PUSH:
			if(n_args == 0)
			{
				ret=Transformation->pushMatrix();
				isArgsSane = true;
			}
			break;

		case CMD_POP:
			if(n_args == 0)
			{
				ret=Transformation->popMatrix();
				isArgsSane = true;
			}
			break;
		// Update Starts
		case CMD_POINT:
			if(n_args == 6)
			{
				Light *light = new Light();
				if(ret = light->setArgs(args, shading,1))
				{
					ret = S->addLight(light);
				}
				else
				{
					delete light;
				}
				isArgsSane = true;
			}
			break;
		case CMD_DIRECTIONAL:
			if(n_args == 6)
			{
				Light *light = new Light();
				if(ret = light->setArgs(args, shading,0))
				{
					ret = S->addLight(light);
				}
				else
				{
					delete light;
				}
				isArgsSane = true;
			}
			break;
		case CMD_AMBIENT:
			if(n_args == 3)
			{
				if(!shading)
				{
					shading = new Shading();
				}
				else
				{
					if(shading->isAmbientSet)
					{
						delete shading;
						shading = new Shading();
					}
				}
				ret = shading->setArgs(args,TYPE_AMBIENT);
				isArgsSane = true;
			}
			break;
		case CMD_SPECULAR:
			if(n_args == 3)
			{
				if(!shading)
				{
					shading = new Shading();
				}
				else
				{
					if(shading->isSpecularSet)
					{
						delete shading;
						shading = new Shading;
					}
				}
				ret = shading->setArgs(args,TYPE_SPECULAR);
				isArgsSane = true;
			}

			break;
		case CMD_DIFFUSE:
			if(n_args == 3)
			{
				if(!shading)
				{
					shading = new Shading();
				}
				else
				{
					if(shading->isDiffuseSet)
					{
						delete shading;
						shading = new Shading();
					}
				}
				ret = shading->setArgs(args,TYPE_DIFFUSE);
				isArgsSane = true;
			}
			break;
		case CMD_SHININESS:
			if(n_args == 1)
			{
				if(!shading)
				{
					shading = new Shading();
				}
				else
				{
					if(shading->isShininessSet)
					{
						delete shading;
						shading = new Shading();
					}
				}
				ret = shading->setArgs(args,TYPE_SHININESS);
				isArgsSane = true;
			}
			break;
		case CMD_EMISSION:
			if(n_args == 3)
			{
				if(!shading)
				{
					shading = new Shading();
				}
				else
				{
					if(shading->isEmissionSet)
					{
						delete shading;
						shading = new Shading();
					}
				}
				ret = shading->setArgs(args,TYPE_EMISSION);
				isArgsSane = true;
			}
			break;
		case CMD_ATTENUATION:
			if(n_args == 3)
			{
				if(!shading)
				{
					shading = new Shading();
				}
				ret = shading->setArgs(args,TYPE_ATTENUATION);
				isArgsSane = true;
			}
			break;
		case CMD_SPOT:
			if(n_args == 5)
			{
				if(!shading)
				{
					shading = new Shading();
				}
				ret = shading->setArgs(args,TYPE_SPOT);
				isArgsSane = true;
			}
			break;

		case CMD_REFRACTION:
			if(n_args == 5)
			{
				ret = shading->setArgs(args, TYPE_REFRACTION);
				isArgsSane = true;
			}
			break;

		case CMD_REFRACT_DISABLE:
			shading->disableRefraction();
			ret = true;
			isArgsSane = true;
			break;


		default:	// really bad...
			fprintf(stderr, "Attempt to set args for invalid command! failing...");

	}

	if(!isArgsSane)
	{
		fprintf(stderr, "Size command insufficient number of parameters, %d", cmd);
		ret = false;
	}
	if(!ret)
	{
		fprintf(stderr,"Update Args failed at CMD : %d", cmd);
	}

	return ret;
}

Command::Command(void)
{
	args = (char **)malloc(sizeof(char *) * N_MAX_ARGS);
	cmd = CMD_NONE;
}
