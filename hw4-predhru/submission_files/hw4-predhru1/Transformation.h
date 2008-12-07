#include "nv/nv_math.h"

#define MAX 12

/* maintain current transform stack */
class Transformation
{
	mat4 *stack[MAX];
	int top;
	mat4 transform;

public:
	Transformation();
	void push(mat4 Mat);
	mat4 pop();
	bool isStackEmpty(){return top==-1;};
	bool isStackFull(){return top==MAX-1;};
	void calcTransform();
	mat4 getTransform(){return transform;};
	bool pushMat();
	bool popMat();

	bool translate(vec3 trans);
	bool rotate(vec3 Rotate, float angle);
	bool scale(vec3 sc);
public:
	~Transformation(void);
};
