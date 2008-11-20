#include <GL/glut.h>
#include <windows.h>
#include <stdio.h>
#include <math.h>

int mapX;
int mapY;
int mapZ;

const int MapWidth = 384;
const int MapHeight = 384;
GLuint texture[10];

BYTE HeightMap[MapWidth][MapHeight];

GLuint LoadTexture( const char * filename, int width, int height )
{
	GLuint texture;
	unsigned char * data;
	FILE * file;

	//The following code will read in our RAW file
	file = fopen( filename, "rb" );
	if ( file == NULL ) return 0;
	data = (unsigned char *)malloc( width * height * 3 );
	fread( data, width * height * 3, 1, file );
	fclose( file );

	glGenTextures( 1, &texture ); //generate the texture with the loaded data
	glBindTexture( GL_TEXTURE_2D, texture ); //bind the texture to it's array
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE ); //set texture environment parameters

	//here we are setting what textures to use and when. The MIN filter is which quality to show
	//when the texture is near the view, and the MAG filter is which quality to show when the texture
	//is far from the view.

	//The qualities are (in order from worst to best)
	//GL_NEAREST
	//GL_LINEAR
	//GL_LINEAR_MIPMAP_NEAREST
	//GL_LINEAR_MIPMAP_LINEAR

	//And if you go and use extensions, you can use Anisotropic filtering textures which are of an
	//even better quality, but this will do for now.
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR );

	//Here we are setting the parameter to repeat the texture instead of clamping the texture
	//to the edge of our shape. 
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	//Generate the texture with mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data ); 
	free( data ); //free the texture
	return texture; //return whether it was successfull
}

void FreeTexture( GLuint texture )
{
  glDeleteTextures( 1, &texture ); 
}


void LoadHeightMap (char* Filename, int Width, int Height) {

int smoothen;

FILE * File;

File = fopen( Filename, "rb" );

fread( HeightMap, 1, Width * Height * 3, File );

fclose( File );

for (smoothen = 0; smoothen < 3; smoothen++){
for (mapX = 1; mapX < Width; mapX ++){ 
for (mapZ = 1; mapZ < Height * 3; mapZ++){ 
mapY = HeightMap[mapX][mapZ];
mapY += HeightMap[mapX-1][mapZ];
mapY += HeightMap[mapX+1][mapZ];
mapY += HeightMap[mapX][mapZ-1];
mapY += HeightMap[mapX][mapZ+1];
mapY += HeightMap[mapX-1][mapZ-1];
mapY += HeightMap[mapX-1][mapZ+1];
mapY += HeightMap[mapX+1][mapZ-1];
mapY += HeightMap[mapX+1][mapZ+1];
mapY = mapY/9;
HeightMap[mapX][mapZ] = mapY ;
}
}
}
}

void DisplayHeightMap (void) {
int Height;

for (mapX = 1; mapX < MapWidth; mapX +=4){ 
for (mapZ = 1; mapZ < MapHeight * 3; mapZ+=4){ 
	glBegin(GL_TRIANGLE_STRIP);
	Height = HeightMap[mapX][mapZ];
	glTexCoord2f(0,0); 
	glVertex3f(float(mapX),Height,float(mapZ));
	
	Height = HeightMap[mapX][mapZ+4];
	glTexCoord2f(0,1); 
	glVertex3f(float(mapX),Height,float(mapZ+4));

	Height = HeightMap[mapX+4][mapZ];
	glTexCoord2f(1,0); 
	glVertex3f(float(mapX+4),Height,float(mapZ));

	Height = HeightMap[mapX+4][mapZ+4];
	glTexCoord2f(1,1); 
	glVertex3f(float(mapX+4),Height,float(mapZ+4));
	glEnd();
}
}
}

void init (void) {
	texture[0] = LoadTexture("terrain.tga", 256, 256);
	LoadHeightMap ("height.raw", 128, 128);
}

void enable (void) {
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	/*glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);*/	
	glShadeModel (GL_SMOOTH); 
	glEnable(GL_TEXTURE_2D);
}

void reshape (int w, int h) {
	glViewport (0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective (100, (GLfloat)w / (GLfloat)h, 0.01, 500.0);
	glMatrixMode (GL_MODELVIEW);
}

void display (void) {
	glClearColor (0.0,0.0,0.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity(); 
	enable();
	//glScalef(0.2,2.0,1);
	glTranslatef(30, 0, 30);
	glRotatef(150, 0,1,0);
	glColor3f(1,1,1);
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	DisplayHeightMap();
	glutSwapBuffers();
}