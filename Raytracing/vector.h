#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>
#include <cmath>
#include <cfloat>

using namespace std;

class Vector
{
public:
	Vector(){};
	Vector(float x, float y, float z);
	Vector(const Vector& v);

	float length();
	float sqrdLength();

	float getIndex(int op) {
		return (op == 0) ? x : (op == 1) ? y : z;
	}

	Vector&	normalize();
	Vector operator=(const Vector& v);
	Vector operator+( const Vector& v );
	Vector operator-( const Vector& v );
	Vector operator*( float f );
	float  operator*(const Vector& v);   //inner product
	Vector operator/( float f );
	Vector operator%( const Vector& v); //external product
	Vector&	operator-=	(const Vector& v);
	Vector&	operator-=	(const float v);
	Vector&	operator*=	(const float v);
	Vector&	operator+=	(const float v);

	float x;
	float y;
	float z;

     friend inline
  istream&	operator >>	(istream& s, Vector& v)
	{ return s >> v.x >> v.y >> v.z; }
  
};

#endif