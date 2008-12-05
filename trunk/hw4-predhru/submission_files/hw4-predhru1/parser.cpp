#include<stdio.h>
#include "nv/nv_math.h"
#include "nv/nv_algebra.h"
#include "Scene.h"
#include "Transform.h"
#include<math.h>
#include<stdlib.h>


void parsefile (FILE *fp) {
  char line[1000], command[1000] ; // Very bad to prefix array size :-)
  int sizeset = 0 ;	
  
  while (!feof(fp)) {
    fgets(line,sizeof(line),fp) ;
    if (feof(fp)) break ;
    if (line[0] == '#') continue ; // Comment lines
    
    int num = sscanf(line, "%s", command) ;
    if (num != 1) continue ; // Blank line etc.

    // Now, we simply parse the file by looking at the first line for the 
    // various commands
    
    /**************        CAMERA LOCATION **********/
	scene = new Scene();
    if (!strcmp(command, "camera")) {
      double lookfrom[3], lookat[3], up[3], fov ;
      int num = sscanf(line, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
		       command, lookfrom, lookfrom+1, lookfrom+2, 
		       lookat, lookat+1, lookat+2, up, up+1, up+2, &fov) ;
      if (num != 11) {
	fprintf(stderr, "camera from[3] at[3] up[3] fov\n") ;
	exit(1) ;
      

      assert(!strcmp(command,"camera")) ;

	vec3 cam_eye = vec3(lookfrom[0],lookfrom[1],lookfrom[2]);
	vec3 cam_center = vec3(lookat[0],lookat[1],lookat[2]);
	vec3 cam_up = vec3(up[0],up[1],up[2]);
	
	camera=new CameraRay(cam_eye,cam_center,cam_up,30.0);
	  }
    /****************************************/

    /***********  GEOMETRY *******************/
	else if (!strcmp(command, "vertex")) {  // Add a vertex to the stack
	  assert(maxverts) ; assert(curvert < maxverts) ;
	  Vertex v ;
	  int num = sscanf(line, "%s %lf %lf %lf", command, v.pos, v.pos+1, v.pos+2) ;
	  assert(num == 4) ; assert(!strcmp(command,"vertex")) ;
	  vert[curvert] = v ;
	  ++curvert ;
	  scene->generateRays(camera);
		vec3 v1 = vec3(-1,-1,0);
		vec3 v2 = vec3(1,-1,0);
		vec3 v3 = vec3(1,1,0);
		vec3 v4 = vec3(-1,1,0);
		q(v1,v2,v3,v4);	

	}
		

	}
	scene = new Scene();
	scene->generateRays(camera);
}