#ifndef COLOR_H
#define COLOR_H

#include <iostream>
#include <cmath>
#include <cfloat>
using namespace std;

#define CLAMP(a, b, c)		(((b) < (a)) ? (a) : (((b) > (c)) ? (c) : (b)))

class Color
{
private:

 float R, G, B;

public:
		Color		()
		     		: R(0.0), G(0.0), B(0.0)
		     		{}
		Color		(float r, float g, float b)
				: R(r), G(g), B(b)
				{}

  float        r		() const
	          		{ return R; }
  float        r		(float r)
	          		{ return (R = r); }
  float        g		() const
	          		{ return G; }
  float        g		(float g)
	          		{ return (G = g); }
  float        b		() const
	          		{ return B; }
  float        b		(float b)
	          		{ return (B = b); }

  Color 	clamp		() const
        			{
        			   return Color(CLAMP(0.0, R, 1.0),
        					CLAMP(0.0, G, 1.0),
        					CLAMP(0.0, B, 1.0));
        			}


  Color 	operator *	(float c) 
        			{ return Color(R*c, G*c, B*c); }


  Color		operator *=	(float c)
        			{ R*=c; G*=c; B*=c; return *this; }

  Color 	operator +	( Color c) 
        			{ return Color(R+c.R, G+c.G, B+c.B); }
  Color 	operator *	(Color c) 
        			{ return Color(R*c.R, G*c.G, B*c.B); }

  Color		operator +=	( Color c)
        			{ R+=c.R; G+=c.G; B+=c.B; return *this; }
  Color		operator *=	(Color c)
				{ R*=c.R; G*=c.G; B*=c.B; return *this; }

   friend inline
  istream&	operator >>	(istream& s, Color& c)
	{ return s >> c.R >> c.G >> c.B; }
};


#endif