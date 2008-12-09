#include "Transformation.h"
#include <stdio.h>



void Transformation::push(mat4 Mat)
{
	stack[++top]=new mat4(Mat);
	printf("pushed\n");
}

mat4 Transformation::pop()
{
	mat4 Mat=*(stack[top]);
	//deleting from the top of the stack
	delete stack[top--];
	//return the current popped element
	return Mat;
}

void Transformation::calcTransform()
{
	int k;
	mat4 ret;
	//assign current transfor to the top of the stack
	transform = *stack[top];
	for(k=top-1; k>=0; k--)
	{
		//right multiplying each element in the stack to get the current transform
		transform = (*stack[k])*transform;
		//transform = mult	transform,(*stack[k]),transform);
	}
}


bool Transformation::pushMat()
{
	mat4 IDENTITY = mat4(1, 0, 0, 0, 
			  0, 1, 0, 0, 
			  0, 0, 1, 0, 
			  0, 0, 0, 1);
	if(isStackFull())
		return false;
	push(IDENTITY);
	return true;
}

bool Transformation::popMat()
{
	if(isStackEmpty())
		return false;
	pop();
	calcTransform();
	return true;
}

bool Transformation::translate(vec3 trans)
{/*
	if( !args )
		return false;

	for(int i=0; i<3; i++)
	{
		if(!args[i])
			return false;
	}
*/
	//forming the translate matrix
	mat4 T(	1, 0, 0, 0, 
			0, 1, 0, 0, 
			0, 0, 1, 0, 
			trans.x,trans.y,trans.z,1);
			//atof(args[0]), atof(args[1]), atof(args[2]), 1);

	//pushing it into the stack
	push(pop()*T);
	//getting the current transform from the stack elements.
	calcTransform();
	printf("\n In translate of transformation\n");
	return true;
}

bool Transformation::rotate(vec3 Rotate, float angle)
{
	/*
	if( !args )
		return false;

	for(int i=0; i<4; i++)
	{
		if(!args[i])
			return false;
	}
*/
	mat4 IDENTITY = mat4(1, 0, 0, 0, 
			  0, 1, 0, 0, 
			  0, 0, 1, 0, 
			  0, 0, 0, 1);
	mat4 R=IDENTITY;
	vec3 r = Rotate;
	//rotatating the matrix R by 'angle' in the direction
	R.set_rot(nv_pi*(angle)/180, r);
	push(pop()*R);
	calcTransform();

	return true;
}

bool Transformation::scale(vec3 sc)
{
	/*
	if( !args )
		return false;

	for(int i=0; i<3; i++)
	{
		if(!args[i])
			return false;
	}
*/
	//compute the scaling matrix
	mat4 S(	sc.x, 0, 0, 0, 
			0, sc.y, 0, 0, 
			0, 0, sc.z, 0, 
			0, 0, 0, 1);
	//pushing it on the stack
	push(pop()*S);
	//calculating the current transform
	calcTransform();
	
	return true;
}

Transformation::Transformation()
{
	mat4 IDENTITY = mat4(1, 0, 0, 0, 
			  0, 1, 0, 0, 
			  0, 0, 1, 0, 
			  0, 0, 0, 1);
	top=-1;
	push(IDENTITY);	
	transform=IDENTITY;
}

Transformation::~Transformation()
{
}
