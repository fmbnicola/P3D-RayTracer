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
#include "bvh.cpp"
#include "maths.h"
#include "sampler.h"
#include "constants.h"


#pragma region MACROS

#define CAPTION "Group 6 Ray/Path Tracer"

#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#pragma endregion MACROS


// ray Counter (to use for Mailboxing)
uint64_t rayCounter = 0;

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
Grid grid;
BVH bvh;

int RES_X, RES_Y;

int WindowHandle = 0;

double erand48(unsigned short xsubi[3]) {
	return (double)rand() / (double) RAND_MAX;
}

/////////////////////////////////////////////////////////////////////// RAYTRACING

//Auxiliary function -> calculates adjusted intersection point
Vector offsetIntersection(Vector inter, Vector normal) {
	return  inter + normal * .0001;
}

float remap(float min1, float max1, float min2, float max2, float value) {

	return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

//Main ray tracing function (index of refraction of medium 1 where the ray is travelling)
Color rayTracing( Ray ray, int depth, float ior_1, int off_x, int off_y, bool inside = false)
{
	Object* obj     = NULL;
	Object* min_obj = NULL;
	Vector hit_p;

	float t     = FLT_MAX; 
	float min_t = FLT_MAX;

	#pragma region ======== GEOMETRY INTERSECTION ========

	if (acl_str == accel_struct::UGrid) {
		//Traverse grid one cell at a time
		if (!grid.Traverse(ray, &min_obj, hit_p)) {
			min_obj = NULL;
		}
	}
	else if (acl_str == accel_struct::Bvh) {
		if (!bvh.intersect_bvh(ray, &min_obj, hit_p)) {
			min_obj = NULL;
		}
	}
	else {
		//iterate through all objects in scene to check for interception
		for (int i = 0; i < scene->getNumObjects(); i++) {

			obj = scene->getObject(i);
		
			if (obj->intercepts(ray, t) && (t < min_t)) {
				min_obj = obj;
				min_t   = t;
			}
		}
	}

	//Depth map
	if (acl_str == accel_struct::None && DEPTH_MAP) {
		
		//cerr << min_t << "\n";

		float min_depth = 5;
		float max_depth = 20;
		float depth = remap(min_depth, max_depth, 1.f, 0.f, min_t);

		Color color = Color(depth, depth, depth).clamp();

		return color;
	}
	
	#pragma endregion
	
	//no intersection -> return background
	if (min_obj == NULL) {
		if (SKYBOX)	return scene->GetSkyboxColor(ray);
		else return scene->GetBackgroundColor();
	}
	//interception -> calculate color
	else {
		Material* mat = min_obj->GetMaterial();
		Color col  = Color();
		Color diff = Color();
		Color spec = Color();

		//debug option, for checking intersections
		if (TEST_INTERSECT)	return Color(1, 0, 0);

		Light* light = NULL;

		Vector l_dir, norm, blinn;
		float fs;

		//fixes floating point errors in intersection
		Vector interceptNotPrecise = (acl_str == accel_struct::None) ? ray.origin + ray.direction * min_t : hit_p;
		Vector intercept = offsetIntersection(interceptNotPrecise, min_obj->getNormal(interceptNotPrecise));

		norm = min_obj->getNormal(intercept);

		#pragma region ======== SHADOWS ======== 

		//disregard shadows for rays traveling inside mesh
		if (!inside) {

			// cast a shadow ray for every light in the scene
			for (int i = 0; i < scene->getNumLights(); i++) {

				light = scene->getLight(i);

				//for antialising + soft shadows cast the multiple rays in the direction of each light (with jitering)
				if (ANTIALIASING && SOFT_SHADOWS) {
					Vector pos = Vector(
						light->position.x + LIGHT_SIDE*(off_x + rand_float()) / SPP, 
						light->position.y + LIGHT_SIDE*(off_y + rand_float()) / SPP,
						light->position.z);
					l_dir = (pos - intercept).normalize();
				}
				else {
					l_dir = (light->position - intercept).normalize();
				}

				// Shadow Feelers 
				Ray feeler = Ray(intercept, l_dir);
				feeler.id = ++rayCounter;
				fs = 1;

				//check for interceptions of feelers
				if (acl_str == accel_struct::UGrid) {
					if (grid.Traverse(feeler)) {
						fs = 0; //is in shadow
					}
				}
				if (acl_str == accel_struct::Bvh) {
					if (bvh.bool_intersect_bvh(feeler)) {
						fs = 0;
					}
				}
				else {
					for (int j = 0; j < scene->getNumObjects(); j++) {

						obj = scene->getObject(j);

						if (obj->intercepts(feeler, t)) {
							fs = 0; //is in shadow
							break;
						}
					}
				}

				//if not in shadow -> add add each lights contribution to output (specular and diffuse components) 
				blinn = ((l_dir + (ray.getDirection() * -1)) / 2).normalize();

				if (fs != 0) {
					diff += (light->color * mat->GetDiffColor()) * (max(0,norm * l_dir));
					spec += (light->color * mat->GetSpecColor()) * pow(max(0,blinn * norm), mat->GetShine()); 
				}
			}
		}

		#pragma endregion

		//add diffuse and specular components of the material to output
		col += diff * mat->GetDiffuse() + spec * mat->GetSpecular();

		if (depth <= 0) return col.clamp();

		#pragma region ======== REFRACTION ======== 

		norm = !inside ? norm : norm * -1;
		float Kr;

		Vector v = ray.getDirection() * -1; //view
		Vector vn = (norm * (v * norm)); //viewnormal
		Vector vt = vn - v; //viewtangent

		Color refrCol = Color(), reflCol = Color();

		if (mat->GetTransmittance() == 0) {

			//opaque material
			Kr = mat->GetSpecular();
		}
		else {
			float Rs = 1, Rp = 1;

			//refraction index (snell law)
			float n = !inside ? ior_1 / mat->GetRefrIndex() : ior_1 / 1;

			float cosOi = vn.length();
			float sinOt = (n) * vt.length(), cosOt;
			float insqrt = 1 - pow(sinOt, 2);

			if (insqrt >= 0) {
				cosOt = sqrt(insqrt);

				//Refraction Secondary Rays
				Vector refractDir = (vt.normalize() * sinOt + norm * (-cosOt)).normalize();
				Vector interceptin = offsetIntersection(interceptNotPrecise, refractDir);

				Ray refractedRay = Ray(interceptin, refractDir);
				refractedRay.id = ++rayCounter;

				float newior = !inside ? mat->GetRefrIndex() : 1; //MAGIC NUMBER
				//rayTracing(...)
				refrCol = rayTracing(refractedRay, depth - 1, newior, off_x, off_y, !inside);

				//Frenel Equations
				Rs = pow(fabs((ior_1 * cosOi - newior * cosOt) / (ior_1 * cosOi + newior * cosOt)), 2); //s-polarized (perpendicular)
				Rp = pow(fabs((ior_1 * cosOt - newior * cosOi) / (ior_1 * cosOt + newior * cosOi)), 2); //p-polarized (parallel)
			}

			//ratio of reflected ligth (mirror reflection attenuation)
			Kr = 1 / 2 * (Rs + Rp);
		}

		#pragma endregion

		#pragma region ======== REFLECTION ======== 

		//if reflective
		if (mat->GetReflection() > 0) {
			
			//throw ray in direction of reflection
			Vector rdir = norm * ((ray.getDirection() * -1) * norm) * 2 + ray.getDirection(); // 2(V*n)*n-V; V=-ray

			Ray rray = Ray(intercept, rdir);
			rray.id = ++rayCounter;

			//get color contribution from ray
			reflCol = rayTracing(rray, depth - 1, ior_1, off_x, off_y, inside);
		}		

		#pragma endregion

		//Add Reflection and refraction color to output 
		col += reflCol * Kr + refrCol * (1 - Kr);

		return col.clamp();
	}
}

/////////////////////////////////////////////////////////////////////// PATHTRACING

Color Radiance(Ray ray, int depth, float ior_1, int off_x, int off_y, unsigned short* seed, bool inside = false) {

	Object* obj = NULL;
	Object* min_obj = NULL;
	Vector hit_p;

	float t = FLT_MAX;
	float min_t = FLT_MAX;

	#pragma region === GEOMETRY INTERSECTION ===

	if (acl_str == accel_struct::UGrid) {
		if (!grid.Traverse(ray, &min_obj, hit_p)) 
			min_obj == NULL;
	}
	else if (acl_str == accel_struct::Bvh) {
		if (!bvh.intersect_bvh(ray, &min_obj, hit_p))
			min_obj == NULL;
	}
	else {
		//iterate through all objects in scene to check for interception
		for (int i = 0; i < scene->getNumObjects(); i++) {

			obj = scene->getObject(i);

			if (obj->intercepts(ray, t) && (t < min_t)) {
				min_obj = obj;
				min_t = t;
			}
		}
	}
	
	#pragma endregion

	#pragma region === GET MATERIAL PROPERTIES ===

	//if no intersection return background
	if (min_obj == NULL || depth == 0) {
		if (SKYBOX) {
			return scene->GetSkyboxColor(ray);
		}
		return scene->GetBackgroundColor();
	}


	//debug option, for checking intersections
	if (TEST_INTERSECT)	return Color(1, 0, 0);

	Vector norm, norml;
	float fs;

	Vector interceptNotPrecise = (acl_str == accel_struct::None) ? ray.origin + ray.direction * min_t : hit_p; 

	norm = min_obj->getNormal(interceptNotPrecise);
	//properly oriented normal
	norml = (norm * ray.direction < 0) ? norm : norm * -1; //FIXME : make sure this is right

	//fixes floating point errors in intersection
	Vector intercept_out = offsetIntersection(interceptNotPrecise, norm);
	Vector intercept_in = offsetIntersection(interceptNotPrecise, norm * -1);

	Material* mat = min_obj->GetMaterial();
	Color f = mat->GetDiffColor();
	
#pragma endregion

	//Russian Roulette
	float p = MAX3(f.r(), f.g(), f.b());

	if (--depth <= (int) MAX_DEPTH - 5) {
		if (rand_float() < p) {
			f = f * (1 / p);
		} else {
			return mat->GetEmission();	
		}
	}

	//Ideal diffuse reflection
	if (mat->GetDiffuse() == 1.0f) {
		float r1 = 2 * PI * rand_float();
		float r2 = rand_float();
		float r2s = sqrt(r2);

		Vector w = norml;
		Vector u = (((fabs(w.x) > .1) ? Vector(0, 1, 0) : Vector(1, 0, 0)) % w).normalize();

		Vector v = w % u;
		Vector d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalize();

		Ray new_r = Ray(intercept_out, d);

		Color e = Color();
		obj = NULL;

		for (int i = 0; i < scene->getNumObjects(); i++) {
#pragma region === ITERATE THROUGH LIGHTS ===
			obj = scene->getObject(i);
			Color emi = obj->GetMaterial()->GetEmission();

			//only care about the lights
			if (emi.sum() <= 0) continue;

			Sphere* light = dynamic_cast<Sphere*> (obj);

			//coordinate system for sampling
			Vector sw = light->GetCenter() - intercept_out;
			Vector su = ((fabs(sw.x) > .1 ? Vector(0, 1, 0) : Vector(1, 0, 0)) % sw).normalize();
			Vector sv = sw % su;

			//light center and radius
			Vector center = light->GetCenter();
			float rad = light->GetRadius();

			//mas angle
			double cos_a_max = sqrt(1 - pow(rad, 2) / ((intercept_out - center) * (intercept_out - center)));

			//sample direction based on random numbers (according to Realist RayTracing)
			double eps1 = erand48(seed);
			double eps2 = erand48(seed);
			double cos_a = 1 - eps1 + eps1 * cos_a_max;
			double sin_a = sqrt(1 - cos_a * cos_a);
			double phi = 2 * PI * eps2;

			Vector l = su * cos(phi) * sin_a + sv * sin(phi) * sin_a + sw * cos_a;
			l = l.normalize();

			Ray feeler = Ray(intercept_out, l);
			feeler.id = ++rayCounter;

			//find shadow feeler interception
			Object* obj2;
			Object* min_obj2 = NULL;
			Vector hit_p2;
			float t2 = FLT_MAX;
			float min_t2 = FLT_MAX;

			if (acl_str == accel_struct::UGrid) {
				if (!grid.Traverse(feeler, &min_obj2, hit_p2)) {
					min_obj2 = NULL;
				}
			}
			else if (acl_str == accel_struct::Bvh) {
				if (!bvh.intersect_bvh(feeler, &min_obj2, hit_p2)) {
					min_obj2 = NULL;
				}
			}
			else {
				//iterate through all objects in scene to check for interception
				for (int j = 0; j < scene->getNumObjects(); j++) {
					obj2 = scene->getObject(j);

					if (obj2->intercepts(feeler, t2) && (t2 < min_t2)) {
						min_obj2 = obj2;
						min_t2 = t2;
					}
				}
			}

			//if intersected (and not with the light itself)
			if (min_obj2 != NULL && min_obj2 == obj) { //FIXME: make sure this is right
				double omega = 2 * PI * (1 - cos_a_max);
				e = e + f * (emi * (l * norml) * omega) * (1 / PI);
			}

		}

		return mat->GetEmission() + e +  f * Radiance(new_r, depth, ior_1, off_x, off_y, seed);
	}
	else if (mat->GetSpecular() == 1.0f) {
		Ray new_r = Ray(intercept_out, ray.direction - norm * (2 * (norm * ray.direction)));
		return mat->GetEmission() + f * Radiance(new_r, depth, ior_1, off_x, off_y, seed);
	}

	Ray reflRay = Ray(intercept_out, ray.direction - norm * 2 * (norm * ray.direction)); // ideal dieletric Reflection
	bool into = norm * norml > 0; // ray from outside?
	double nc = 1.0f;
	double nt = mat->GetRefrIndex();
	double nnt = into ? nc / nt : nt / nc;
	double ddn = ray.direction * norml;
	double cos2t = 1 - nnt * nnt * (1 - ddn * ddn);

	if (cos2t < 0) { // Total internal reflection
		return mat->GetEmission() + f * Radiance(reflRay, depth, ior_1, off_x, off_y, seed);
	}

	Vector tdir = (ray.direction * nnt - norm * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t)))).normalize();
	double a = nt - nc;
	double b = nt + nc;
	double R0 = (a * a) / (b * b);
	double c = 1 - (into ? -ddn : tdir * norm);
	double Re = R0 + (1 - R0) * c * c * c * c * c;
	double Tr = 1 - Re;
	double P = 0.25 + 0.5 * Re;
	double RP = Re / P;
	double TP = Tr / (1 - P);

	Color col = depth <= (MAX_DEPTH - 2) ? (erand48(seed) < P) ?
		Radiance(reflRay, depth, ior_1, off_x, off_y, seed) * RP :
		Radiance(Ray(intercept_out, tdir), depth, ior_1, off_x, off_y, seed) * TP :
		Radiance(reflRay, depth, ior_1, off_x, off_y, seed) * Re + 
			Radiance(Ray(intercept_in, tdir), depth, ior_1, off_x, off_y, seed) * Tr;

	return mat->GetEmission() + f * col;
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

	// Set up the grid with all objects from the scene
	if (acl_str == accel_struct::UGrid) {

		grid = Grid();

		for (int o = 0; o < scene->getNumObjects(); o++) {
			grid.addObject(scene->getObject(o));
		}

		grid.Build();
	}

	if (acl_str == accel_struct::Bvh) {
		vector<Object*> objs;

		for (int o = 0; o < scene->getNumObjects(); o++) {
			objs.push_back(scene->getObject(o));
		}

		bvh.build(objs);
	}

	set_rand_seed(time(NULL) * time(NULL));

	//For softshadows without antialiasing we replicate each light multiple times
	if (!ANTIALIASING && SOFT_SHADOWS) {
		
		vector<Light*> new_lights;
		float step = LIGHT_SIDE / SPP;
		float start = -LIGHT_SIDE / 2 + step / 2;
		float end = LIGHT_SIDE / 2;

		int limit = scene->getNumLights();
		for (int k = 0; k < limit; k++) {
			Light* light = scene->getLight(k);
			Color avg_col = light->color / (SPP * SPP);

			for (float i = start; i < end; i += step) {
				for (float j = start; j < end; j += step) {
					Vector pos = Vector(light->position.x + i, light->position.y + j, light->position.z);
					new_lights.push_back(new Light(pos, avg_col));
				}
			}
		}
		scene->setLights(new_lights);
	}

	for (int y = 0; y < RES_Y; y++)
	{
		unsigned short seed[3] = { 0,0,y * y * y }; //Generate seed for radiance

		for (int x = 0; x < RES_X; x++)
		{
			Color color = Color(); 
			Vector pixel;  //viewport coordinates
			Vector lens;   //lens coords

			//Antialiasing -> shoot multiple rays per pixel
			if (ANTIALIASING) {
				for (int i = 0; i < SPP; i++) {
					for (int j = 0; j < SPP; j++) {
						Ray* ray = nullptr;

						if (s_mode == sample_mode::jitter) {
							pixel.x = x + (i + rand_float()) / SPP;
							pixel.y = y + (j + rand_float()) / SPP;
						}
						else if (s_mode == sample_mode::tent) {
							double r1 = 2 * erand48(seed), dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
							double r2 = 2 * erand48(seed), dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);

							pixel.x = x + (0.5 + dx) / SPP;
							pixel.y = y + (0.5 + dy) / SPP;
						}

						//DOF -> Rays are not shot from the same point but instead from a "lens"
						if (DEPTH_OF_FIELD) {
							//Sample disk -> alternative to jitering that displaces rays in a circle
							if (SAMPLE_DISK) lens = sample_unit_disk();
							else {
								lens.x = (i + rand_float()) / SPP;
								lens.y = (j + rand_float()) / SPP;
							}
							ray = &scene->GetCamera()->PrimaryRay(lens, pixel);
						}
						else {
							ray = &scene->GetCamera()->PrimaryRay(pixel);
						}

						ray->id = ++rayCounter;

						if (PATHTRACING) {
							color += Radiance(*ray, MAX_DEPTH, 1.0, i, j, seed);
						}
						else{
							color += rayTracing(*ray, MAX_DEPTH, 1.0, i, j);
						}
						
					}
				}
				color = color / (SPP * SPP);

			}
			//No Antialiasing -> single ray per pixel
			else {
				pixel.x = x + 0.5;
				pixel.y = y + 0.5;

				Ray ray = scene->GetCamera()->PrimaryRay(pixel);
				ray.id = ++rayCounter;

				color += rayTracing(ray, MAX_DEPTH, 1.0, 0, 0);
			}

			double invGamma = 1 / GAMMA; //clara: 0.0 - 1.0  ; escura: 1.8 - 2.2
			color = Color(pow(color.r(), invGamma), pow(color.g(), invGamma), pow(color.b(), invGamma));

			//Create Image
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