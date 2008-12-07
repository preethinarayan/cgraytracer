
class Image
{
	int width;
	int height;
	int *pixel;
public:
	Image(void);
	bool setSize(int _width,int _height);
	int getWidth(){ return width; };
	int getHeight(){ return height; };
	void setPixel(int i, int j, int R, int G, int B)
	{
		pixel[i*width*3 + j*3 + 0] = R; 
		pixel[i*width*3 + j*3 + 1] = G; 
		pixel[i*width*3 + j*3 + 2] = B;
	};
	bool writeImage(char *file);

public:
	~Image(void);
};
