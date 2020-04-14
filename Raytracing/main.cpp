 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2019 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "grid.h"
#include "maths.h"
#include "sampler.h"

#define CAPTION "Whitted Ray-Tracer"

#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define MAX_DEPTH 4

#define SPP 3

#define LIGHT_SIDE 1.0f

int light_grid = 1; // should be odd, for simplicity reasons

//Antialiasing flag (also turns on the DOF)
bool antialiasing = true;

bool depthOfField = true; //for DOF to work, antialiasing must be true as well

bool background = true;

//Enable OpenGL drawing.  
bool drawModeEnabled = true;

//Draw Mode: 0 - point by point; 1 - line by line; 2 - full frame at once
int draw_mode = 1;

// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;
int RES_X, RES_Y;

int WindowHandle = 0;

Vector offsetIntersection(Vector inter, Vector normal) {
	return  inter + normal * .0001;
}

Color rayTracing( Ray ray, int depth, float ior_1, int off_x, int off_y, bool inside = false)  //index of refraction of medium 1 where the ray is travelling
{
	Object* obj     = NULL;
	Object* min_obj = NULL;

	float t     = FLT_MAX; 
	float min_t = FLT_MAX;

	//iterate through all objects in scene to check for interception
	for (int i = 0; i < scene->getNumObjects(); i++) {

		obj = scene->getObject(i);
		
		if (obj->intercepts(ray, t) && (t < min_t)) {
			min_obj = obj;
			min_t   = t;
		}
	}

	//no intersection -> return background
	if (min_obj == NULL) {
		if (background)	return scene->GetSkyboxColor(ray);
		else return scene->GetBackgroundColor();
	}
	else {
		
		Material* mat = min_obj->GetMaterial();
		Color col  = Color();
		Color diff = Color();
		Color spec = Color();

		Light* light = NULL;

		Vector l_dir, norm, blinn;
		float fs;

		//fixes floating point errors in intersection
		Vector interceptNotPrecise = ray.origin + ray.direction * min_t;
		Vector intercept = offsetIntersection(interceptNotPrecise, min_obj->getNormal(interceptNotPrecise));

		norm = min_obj->getNormal(intercept);

		if (!inside) {
			// cast a shadow ray for every light in the scene
			for (int i = 0; i < scene->getNumLights(); i++) {

				light = scene->getLight(i);

				if (antialiasing) {
					Vector pos = Vector(
						light->position.x + LIGHT_SIDE*(off_x + rand_float()) / SPP, 
						light->position.y + LIGHT_SIDE*(off_y + rand_float()) / SPP,
						light->position.z);
					l_dir = (pos - intercept).normalize();
				}
				else {
					l_dir = (light->position - intercept).normalize();
				}

				Ray feeler = Ray(intercept, l_dir);
				fs = 1;
				// intersect the shadow ray with each object in the scene
				for (int j = 0; j < scene->getNumObjects(); j++) {

					obj = scene->getObject(j);

					if (obj->intercepts(feeler, t)) {
						fs = 0; //is in shadow
						break;
					}
				}

				//add each lights contribution to output 
				blinn = ((l_dir + (ray.getDirection() * -1)) / 2).normalize();

				if (fs != 0) {
					diff += (light->color * mat->GetDiffColor()) * (norm * l_dir);
					spec += (light->color * mat->GetSpecColor()) * pow(blinn * norm, mat->GetShine());
				}
			}
		}

		col += diff * mat->GetDiffuse() + spec * mat->GetSpecular();

		if (depth <= 0) return col;

		// refraction and reflected components
		norm = !inside ? norm : norm * -1;
		float Kr;

		Vector v = ray.getDirection() * -1;
		Vector vn = (norm * (v * norm));
		Vector vt = vn - v;
		Color refrCol = Color(), reflCol = Color();

		if (mat->GetTransmittance() == 0) {
			Kr = mat->GetSpecular();
		}
		else {
			float Rs = 1, Rp = 1;

			float n = !inside ? ior_1 / mat->GetRefrIndex() : ior_1 / 1; //MAGIC NUMBER

			float cosOi = vn.length();
			float sinOt = (n) * vt.length(), cosOt;
			float insqrt = 1 - pow(sinOt, 2);

			if (insqrt >= 0) {
				cosOt = sqrt(insqrt);

				Vector refractDir = (vt.normalize() * sinOt + norm * (-cosOt)).normalize();
				Vector interceptin = offsetIntersection(interceptNotPrecise, refractDir);

				Ray refractedRay = Ray(interceptin, refractDir);

				float newior = !inside ? mat->GetRefrIndex() : 1; //MAGIC NUMBER
				//rayTracing(...)
				refrCol = rayTracing(refractedRay, depth - 1, newior, off_x, off_y, !inside);

				Rs = pow(fabs((ior_1 * cosOi - newior * cosOt) / (ior_1 * cosOi + newior * cosOt)), 2);
				Rp = pow(fabs((ior_1 * cosOt - newior * cosOi) / (ior_1 * cosOt + newior * cosOi)), 2);
			}

			Kr = 1 / 2 * (Rs + Rp);

		}

		//if reflective
		if (mat->GetReflection() > 0) {
			//throw ray in direction of reflection
			Vector rdir = norm * ((ray.getDirection() * -1) * norm) * 2 + ray.getDirection(); // 2(V*n)*n-V; V=-ray

			Ray rray = Ray(intercept, rdir);

			//rayTracing(...)
			reflCol = rayTracing(rray, depth - 1, ior_1, off_x, off_y, inside);
		}

		col += reflCol * Kr + refrCol * (1 - Kr);

		return col.clamp();
	}
}

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs


void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	
	if (draw_mode == 0) glDrawArrays(GL_POINTS, 0, 1);
	else if (draw_mode == 1) glDrawArrays(GL_POINTS, 0, RES_X);
	else glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos=0;
	int index_col=0;
	unsigned int counter = 0;

	set_rand_seed(time(NULL) * time(NULL));

	if ((light_grid > 1) && (!antialiasing)) {
		int limit = scene->getNumLights();
		for (int k = 0; k < limit; k++) {
			Light* light = scene->getLight(k);
			light->color = light->color / (light_grid * light_grid);
			float step = LIGHT_SIDE / light_grid;
			float start = floor((LIGHT_SIDE * 10.0f) / 2) /10;

			for (float i = -start; i < (start + step); i += step) {
				for (float j = -0.2f; j < (0.2f + step); j += step) {
					if (!(i == 0 && j == 0)) {
						Vector pos = Vector(light->position.x + i, light->position.y + j, light->position.z);
						scene->addLight(new Light(pos, light->color));
					}
				}
			}
		}
	}

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color = Color(); 
			Vector pixel;  //viewport coordinates
			Vector lens;   //lens coords

			if (antialiasing) {
				for (int i = 0; i < SPP; i++) {
					for (int j = 0; j < SPP; j++) {
						pixel.x = x + (i + rand_float()) / SPP;
						pixel.y = y + (j + rand_float()) / SPP;

						Ray *ray = nullptr;
						if (depthOfField) {
							lens.x = (i + rand_float()) / SPP;
							lens.y = (j + rand_float()) / SPP;
							ray = &scene->GetCamera()->PrimaryRay(lens, pixel);
						}
						else ray = &scene->GetCamera()->PrimaryRay(pixel);

						color += rayTracing(*ray, 5, 1.0, i, j);
					}
				}

				color = color / (SPP * SPP);
			}
			else {
				pixel.x = x + 0.5;
				pixel.y = y + 0.5;

				Ray ray = scene->GetCamera()->PrimaryRay(pixel);
				color += rayTracing(ray, 5, 1.0, 0, 0);
			}

			

			//color = scene->GetBackgroundColor(); //just for the template

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();


				if (draw_mode == 0) {  // drawing point by point
					drawPoints();
					index_pos = 0;
					index_col = 0;
				}
			}
		}
		if (draw_mode == 1 && drawModeEnabled) {  // drawing line by line
			drawPoints();
			index_pos = 0;
			index_col = 0;
		}
	}
	if (draw_mode == 2 && drawModeEnabled)        //full frame at once
		drawPoints();
		 
	printf("Drawing finished!\n"); 	

	if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
		printf("Error saving Image file\n");
		exit(0);
	}
	printf("Image file created\n");
	glFlush();
}

// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

	case 27:
		glutLeaveMainLoop();
		break;

	}
}

/////////////////////////////////////////////////////////////////////// SETUP

void setupCallbacks() 
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
}

void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(640,100);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


void init(int argc, char* argv[])
{
	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
	
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	while (true) {
		cout << "Input the Scene Name: ";
		cin >> input_user;
		strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
		strcat_s(scene_name, sizeof(scene_name), input_user);

		ifstream file(scene_name, ios::in);
		if (file.fail()) {
			printf("\nError opening P3F file.\n");
		}
		else
			break;
	}

	scene = new Scene();
	scene->load_p3f(scene_name);
	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);
}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int ch;
	if (!drawModeEnabled) {

		do {
			init_scene();
			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
		
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		init_scene();
		if (draw_mode == 0) { // draw image point by point
			size_vertices = 2 * sizeof(float);
			size_colors = 3 * sizeof(float);
			printf("DRAWING MODE: POINT BY POINT\n\n");
		}
		else if (draw_mode == 1) { // draw image line by line
			size_vertices = 2 * RES_X * sizeof(float);
			size_colors = 3 * RES_X * sizeof(float);
			printf("DRAWING MODE: LINE BY LINE\n\n");
		}
		else if (draw_mode == 2) { // draw full frame at once
			size_vertices = 2 * RES_X*RES_Y * sizeof(float);
			size_colors = 3 * RES_X*RES_Y * sizeof(float);
			printf("DRAWING MODE: FULL IMAGE\n\n");
		}
		else {
			printf("Draw mode not valid \n");
			exit(0);
		}
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);

		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
	   
		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////