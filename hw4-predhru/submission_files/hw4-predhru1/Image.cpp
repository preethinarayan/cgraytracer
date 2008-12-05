#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "Image.h"

bool Image::setSize()
{
	width = 640;
	height = 480;

	/* RGB allocation */
	pixel = (int *)malloc(sizeof(int) * width *height * 3);

	return true;
}

bool Image::writeImage()
{
	FILE *fp;
	assert(fp = fopen(output_file, "wt")) ;

	fprintf(fp, "P3\n");
	fprintf(fp, "%d %d ", width, height);
	fprintf(fp, "255");

	printf("P3\n");
	printf("%d %d ", width, height);
	printf("255");

	for(int i=0; i<height; i++)
		for(int j=0; j<width; j++)
		{
			fprintf(fp, " %d %d %d", pixel[i*width*3 + j*3 + 0], pixel[i*width*3 + j*3 + 1], pixel[i*width*3 + j*3 + 2]);
			//printf(" %d %d %d", pixel[i*width*3 + j*3 + 0], pixel[i*width*3 + j*3 + 1], pixel[i*width*3 + j*3 + 2]);
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
