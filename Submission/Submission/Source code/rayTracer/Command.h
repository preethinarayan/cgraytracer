#pragma once

/* singleton class maintaining command information, functions and proccessing */

#define N_MAX_ARGS 10
enum cmd_t{CMD_NONE, CMD_SIZE, CMD_CAMERA, CMD_SPHERE, CMD_MAX_VERTS,
			CMD_VERTEX, CMD_TRI, CMD_TRANSLATE, CMD_ROTATE, CMD_SCALE, CMD_PUSH,
			CMD_POP, CMD_MAX_VERT_NORMS, CMD_VERTEXNORMAL,
			CMD_TRINORMAL, CMD_DIRECTIONAL, CMD_POINT, 
			CMD_ATTENUATION, CMD_AMBIENT,
			CMD_DIFFUSE, CMD_SPECULAR,
			CMD_SHININESS, CMD_EMISSION,
			CMD_SPOT,
			CMD_REFRACTION,
			CMD_REFRACT_DISABLE};

typedef struct{
	char *cmd_string;
	cmd_t cmd_name;
	int cmd_n_args;
} cmds;

class Command
{
public:
	/* maintains state of only current command and its args */
	cmd_t cmd;
	char **args;
	int n_args;

	static Command* Instance();
	bool setCommand(char *s);
	bool setArg(char *s);
	bool updateCommandArgs();
protected:
	Command(void);
	~Command();
private:
    static Command* pinstance;
	void clearArgs();
};
