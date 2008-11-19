#pragma once

#define OP "scene_sphere5.ppm"

class Image
{
	int width;
	int height;
	int *pix;
public:
	Image(void);
	bool setSize(char **args);
	int getWidth(){ return width; };
	int getHeight(){ return height; };
	/* height, width, R, G, B */
	void setPixel(int i, int j, int r, int g, int b){
		pix[i*width*3 + j*3 + 0] = r; 
		pix[i*width*3 + j*3 + 1] = g; 
		pix[i*width*3 + j*3 + 2] = b;
	};
	bool writeImage();
public:
	~Image(void);
};
