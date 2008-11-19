#include "StdAfx.h"
#include "Transform.h"

mat4 mat_I(1, 0, 0, 0, 
		   0, 1, 0, 0, 
		   0, 0, 1, 0, 
		   0, 0, 0, 1);

void Transform::push(mat4 M)
{
	stack[++top]=new mat4(M);
}

mat4 Transform::pop()
{
	mat4 M=*(stack[top]);
	delete stack[top--];
	return M;
}

void Transform::calcCurrTransform()
{
	int i;
	mat4 ret;
	currTransform = *stack[top];
	for(i=top-1; i>=0; i--)
	{
		/* right multiply the topmost tranforms */
		currTransform = (*stack[i])*currTransform;
	}
}

/* commands */
bool Transform::pushMatrix()
{
	if(isFull())
		return false;
	push(mat_I);
	return true;
}

bool Transform::popMatrix()
{
	if(isEmpty())
		return false;
	pop();
	calcCurrTransform();
	return true;
}

bool Transform::translate(char **args)
{
	if( !args )
		return false;

	for(int i=0; i<3; i++)
	{
		if(!args[i])
			return false;
	}

	mat4 T(	1, 0, 0, 0, 
			0, 1, 0, 0, 
			0, 0, 1, 0, 
			atof(args[0]), atof(args[1]), atof(args[2]), 1);
	push(pop()*T);
	calcCurrTransform();

	return true;
}

bool Transform::rotate(char **args)
{
	if( !args )
		return false;

	for(int i=0; i<4; i++)
	{
		if(!args[i])
			return false;
	}

	mat4 R=mat_I;
	vec3 v(atof(args[0]), atof(args[1]), atof(args[2]));
	R.set_rot(nv_pi*(atof(args[3]))/180, v);
	push(pop()*R);
	calcCurrTransform();

	return true;
}

bool Transform::scale(char **args)
{
	if( !args )
		return false;

	for(int i=0; i<3; i++)
	{
		if(!args[i])
			return false;
	}

	mat4 S(	atof(args[0]), 0, 0, 0, 
			0, atof(args[1]), 0, 0, 
			0, 0, atof(args[2]), 0, 
			0, 0, 0, 1);
	push(pop()*S);
	calcCurrTransform();
	
	return true;
}

Transform::Transform(void)
{
	top=-1;
	push(mat_I);	
	currTransform=mat_I;
}

Transform::~Transform(void)
{
}
