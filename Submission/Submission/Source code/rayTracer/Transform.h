#pragma once
#include "nv_math.h"

#define MAX_DEPTH 10

/* maintain current transform stack */
class Transform
{
	mat4 *stack[MAX_DEPTH];
	int top;
	mat4 currTransform;

	void calcCurrTransform();
	void push(mat4 M);
	mat4 pop();
	bool isEmpty(){return top==-1;};
	bool isFull(){return top==MAX_DEPTH-1;};
	
public:
	Transform(void);
	mat4 getCurrTransform(){return currTransform;};

	/* commands */
	bool pushMatrix();
	bool popMatrix();
	bool translate(char **args);
	bool rotate(char **args);
	bool scale(char **args);
public:
	~Transform(void);
};
