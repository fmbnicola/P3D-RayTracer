#include "vector.h"
#include <cmath>

Vector::Vector(float a_x, float a_y, float a_z) : x(a_x), y(a_y), z(a_z)
{

}


float Vector::length()
{
	return sqrt( x * x + y * y + z * z );
}


float Vector::sqrdLength()
{
	return (x * x + y * y + z * z);
}


// --------------------------------------------------------------------- copy constructor
Vector::Vector(const Vector& v)
{
	x = v.x; y = v.y; z = v.z;
}

// --------------------------------------------------------------------- assignment operator
Vector Vector::operator= (const Vector& rhs) {
	if (this == &rhs)
		return (*this);
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	return (*this);
}

Vector Vector::operator+(const  Vector& v )
{
	return Vector( x + v.x, y + v.y, z + v.z );
}


Vector Vector::operator-(const Vector& v )
{
	return Vector( x - v.x, y - v.y, z - v.z );
}


Vector Vector::operator*( float f )
{
	return Vector( x * f, y * f, z * f );
}

float Vector::operator*(const  Vector& v)
{
	return x * v.x + y * v.y + z * v.z;
}

Vector Vector::operator/( float f )
{
	return Vector( x / f, y / f, z / f );
}

Vector&	Vector::normalize	()
{
				   float l=1.0/this->length();
				   x *= l; y *= l; z *= l;
				   return *this;
}

Vector&	Vector::operator-=(const Vector& v)
{ x-=v.x; y-=v.y; z-=v.z; return *this; }

Vector&	Vector::operator-=(const float v)
{ x-=v; y-=v; z-=v; return *this; }

Vector&	Vector::operator+=(const float v)
{ x+=v; y+=v; z+=v; return *this; }

Vector&	Vector::operator*=(const float v)
{ x*=v; y*=v; z*=v; return *this; }

Vector Vector::operator%( const Vector& v)
{
	float uX = x;
	float uY = y;
	float uZ = z;

	float vX = v.x;
	float vY = v.y;
	float vZ = v.z;

	float sX = uY * vZ - uZ * vY;
	float sY = uZ * vX - uX * vZ;
	float sZ = uX * vY - uY * vX;

	return Vector( sX, sY, sZ );
}