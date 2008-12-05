//Lighting is also easily achieved in OpenGL.
//You enable the depth, the lighting, and which light you 
//are using.

//Enabling the lighting is done with:
//glEnable (GL_LIGHTING);
//and enabling certain lights is done with:
//glEnable (GL_LIGHT0);

//Now with your lights, this will only set up the default values.
//Which means that the light will be white.

//One thing to keep in mind with your lighting is that the light 
//is not determined by the object. Each object has points called Normals, 
//these are what determine how the light interacts with the object. 
//Some shapes automatically generate Normals but there are others that
//dont. So you can set up your Normals manually, but I do not know the
//mathematics behind this yet. So I will leave that for later. 

//Later on in the different lighting types tutorial. I will explain 
//how to enable different types of lighting such as Diffuse, Ambient,
//Specular and Spot lights. 

#include <GL/gl.h>
#include <GL/glut.h>

GLfloat angle = 0.0;

void cube (void) {
	glRotatef(angle, 1.0, 0.0, 0.0);
	glRotatef(angle, 0.0, 1.0, 0.0);
	glRotatef(angle, 0.0, 0.0, 1.0);
	glColor3f(1.0, 0.0, 0.0);
	glutSolidCube(2);
}

void init (void) {
	glEnable (GL_DEPTH_TEST);
	glEnable (GL_LIGHTING);
	glEnable (GL_LIGHT0);
}

void display (void) {
	glClearColor (0.0,0.0,0.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();  
	gluLookAt (0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	cube();
	glutSwapBuffers();
	angle ++;
}

void reshape (int w, int h) {
	glViewport (0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective (60, (GLfloat)w / (GLfloat)h, 1.0, 100.0);
	glMatrixMode (GL_MODELVIEW);
}

int main (int argc, char **argv) {
    glutInit (&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize (500, 500);
	glutInitWindowPosition (100, 100);
    glutCreateWindow ("A basic OpenGL Window");
	init ();
    glutDisplayFunc (display);
	glutIdleFunc (display);
	glutReshapeFunc (reshape);
    glutMainLoop ();
    return 0;
}
