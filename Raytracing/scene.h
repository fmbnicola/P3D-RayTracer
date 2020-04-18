#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <cmath>
#include <IL/il.h>
using namespace std;

#include "camera.h"
#include "color.h"
#include "vector.h"
#include "ray.h"
#include "boundingBox.h"

#define MIN(a, b)		( ( a ) < ( b ) ? ( a ) : ( b ) )
#define MAX(a, b)		( ( a ) > ( b ) ? ( a ) : ( b ) )
#define MIN3(a, b, c)		( ( a ) < ( b ) \
? ( ( a ) < ( c ) ? ( a ) : ( c ) ) \
: ( ( b ) < ( c ) ? ( b ) : ( c ) ) )
#define MAX3(a, b, c)		( ( a ) > ( b ) \
? ( ( a ) > ( c ) ? ( a ) : ( c ) ) \
: ( ( b ) > ( c ) ? ( b ) : ( c ) ) )

//Skybox images constant symbolics
typedef enum { RIGHT, LEFT, TOP, BOTTOM, FRONT, BACK } CubeMap;


#define EPSILON			0.0001f


class Material
{
public:
	
	Material() :
		m_diffColor(Color(0.2f, 0.2f, 0.2f)), m_Diff( 0.2f ), m_specColor(Color(1.0f, 1.0f, 1.0f)), m_Spec( 0.8f ), m_Shine(20), m_Refl( 1.0f ), m_T( 0.0f ), m_RIndex( 1.0f ){};

	Material (Color& c, float Kd, Color& cs, float Ks, float Shine, float T, float ior) {
		m_diffColor = c; m_Diff = Kd; m_specColor = cs; m_Spec = Ks; m_Shine = Shine; m_Refl = Ks; m_T = T; m_RIndex = ior;
	}

	void SetDiffColor( Color& a_Color ) { m_diffColor = a_Color; }
	Color GetDiffColor() { return m_diffColor; }
	void SetSpecColor(Color& a_Color) { m_specColor = a_Color; }
	Color GetSpecColor() { return m_specColor; }
	void SetDiffuse( float a_Diff ) { m_Diff = a_Diff; }
	void SetSpecular( float a_Spec ) { m_Spec = a_Spec; }
	void SetShine( float a_Shine ) { m_Shine = a_Shine; }
	void SetReflection( float a_Refl ) { m_Refl = a_Refl; }
	void SetTransmittance( float a_T ) { m_T = a_T; }
	float GetSpecular() { return m_Spec; }
	float GetDiffuse() { return m_Diff; }
	float GetShine() { return m_Shine; }
	float GetReflection() { return m_Refl; }
	float GetTransmittance() { return m_T; }
	void SetRefrIndex( float a_ior ) { m_RIndex = a_ior; }
	float GetRefrIndex() { return m_RIndex; }
private:
	Color m_diffColor, m_specColor;
	float m_Refl, m_T;
	float m_Diff, m_Shine, m_Spec;
	float m_RIndex;
};

class Light
{
public:

	Light( Vector& pos, Color& col ): position(pos), color(col) {};
	
	Vector position;
	Color color;
};

class Object
{
public:

	Material* GetMaterial() { return m_Material; }
	void SetMaterial( Material *a_Mat ) { m_Material = a_Mat; }
	virtual bool intercepts( Ray& r, float& dist ) = 0;
	virtual Vector getNormal( Vector point ) = 0;
	virtual AABB GetBoundingBox() { return AABB(); }

protected:
	Material* m_Material;
	AABB* bbox = NULL;
	
};

class Plane : public Object
{
protected:
  Vector	 PN;
  Vector 	 A;

public:
		 Plane		(Vector& PNc, Vector A);
		 Plane		(Vector& P0, Vector& P1, Vector& P2);

		 bool intercepts( Ray& r, float& dist );
         Vector getNormal(Vector point);
};

class Triangle : public Object
{
	
public:
	Triangle	(Vector& P0, Vector& P1, Vector& P2);
	bool intercepts( Ray& r, float& t);
	Vector getNormal(Vector point);
	AABB GetBoundingBox(void);
	
protected:
	Vector points[3];
	Vector normal;
	Vector Min, Max;
};


class Sphere : public Object
{
public:
	Sphere( Vector& a_center, float a_radius ) : 
		center( a_center ), SqRadius( a_radius * a_radius ), 
		radius( a_radius ) {};

	bool intercepts( Ray& r, float& t);
	Vector getNormal(Vector point);
	AABB GetBoundingBox(void);

private:
	Vector center;
	float radius, SqRadius;
};

class aaBox : public Object   //Axis aligned box: another geometric object
{
public:
	aaBox(Vector& minPoint, Vector& maxPoint);
	AABB GetBoundingBox(void);
	bool intercepts(Ray& r, float& t);
	Vector getNormal(Vector point);

private:
	Vector min;
	Vector max;

	Vector Normal;
};


class Scene
{
public:
	Scene();
	virtual ~Scene();
	
	Camera* GetCamera() { return camera; }
	Color GetBackgroundColor() { return bgColor; }
	Color GetSkyboxColor(Ray& r);
	bool GetSkyBoxFlg() { return SkyBoxFlg; }
	
	void SetBackgroundColor(Color a_bgColor) { bgColor = a_bgColor; }
	void LoadSkybox(const char*);
	void SetSkyBoxFlg(bool a_skybox_flg) { SkyBoxFlg = a_skybox_flg; }
	void SetCamera(Camera *a_camera) {camera = a_camera; }

	int getNumObjects( );
	void addObject( Object* o );
	Object* getObject( unsigned int index );
	
	int getNumLights( );
	void addLight( Light* l );
	Light* getLight( unsigned int index );

	bool load_p3f(const char *name);  //Load NFF file method
	
private:
	vector<Object *> objects;
	vector<Light *> lights;

	Camera* camera;
	Color bgColor;  //Background color

	bool SkyBoxFlg = false;

	struct {
		ILubyte *img;
		unsigned int resX;
		unsigned int resY;
		unsigned int BPP; //bytes per pixel
	} skybox_img[6];

};

#endif