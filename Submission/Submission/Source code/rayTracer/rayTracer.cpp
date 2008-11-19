// rayTracer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
Image *I;
Parser *P;
Command *C;
Camera *Cam;
Scene *S;
Vertexes *Vert;
Transform *Transformation;
//Shading *shading;

void initCommands()
{
	S = new Scene();
	/* NOTE: we may need to impose a structure requirement for the command file */
	while(P->getNextCommand())
	{
		assert(C->updateCommandArgs());
	}

}

int main(int argc, char* argv[])
{
	if (argc < 2) 
	{
		fprintf(stderr, "Usage: openglviewer filename\n") ;
		exit(1) ;
	}

	C = Command::Instance();
	I = new Image();
	S = new Scene();
	Cam = new Camera();
	Vert = new Vertexes();
	Transformation = new Transform();
	shading = new Shading();

	printf("%d", strlen(argv[1]));
	P = Parser::Instance(argv[1], I);

	if(!P->initFileHeaders())
	{
		fprintf(stderr, "Input File operation error.. terminating");
		exit(1);
	}

	initCommands();
	Cam->setImageParameters(I->getWidth(),I->getHeight());

	S->traceScene();
	I->writeImage();


	  int i;
//	scanf("%d", &i);
	  delete P;

	  return 0;
}

