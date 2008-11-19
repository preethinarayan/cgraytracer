#include "StdAfx.h"
#include "Image.h"

bool Image::setSize(char **args)
{
	if( !args || !args[0] || !args[1] )
		return false;
	width = atoi(args[0]);
	height = atoi(args[1]);

	/* RGB allocation */
	pix = (int *)malloc(sizeof(int) * width *height * 3);

	return true;
}

bool Image::writeImage()
{
	FILE *fp;
	assert(fp = fopen(OP, "wt")) ;

	/* init headers */
	fprintf(fp, "P3\n");
	fprintf(fp, "%d %d ", width, height);
	fprintf(fp, "255");

	// ppm format width*height
	for(int i=0; i<height; i++)
		for(int j=0; j<width; j++)
		{
			fprintf(fp, " %d %d %d", 
				pix[i*width*3 + j*3 + 0], 
				pix[i*width*3 + j*3 + 1], pix[i*width*3 + j*3 + 2]);
		}
	fclose(fp);
	return true;
}

Image::Image(void)
{
}

Image::~Image(void)
{
}
