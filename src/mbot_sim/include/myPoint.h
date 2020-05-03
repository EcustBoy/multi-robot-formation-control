#ifndef SR_MY_POINT
#define SR_MY_POINT
#include <math.h> 


/*! \brief 2D Point structure
 * 
 * Simple structure with double @x @y values.
 */

 class MyPoint
{
public:
	
	
	 
	MyPoint(double xPos, double yPos);		
	~MyPoint();
	
	double getAngle(MyPoint* target);	
	double getDistance(MyPoint* target);
	MyPoint operator+(const MyPoint &other) const;
	MyPoint operator-(const MyPoint &other) const;
	MyPoint times(double r);
	
	/*! Absolute value of vector.
	 */ 
	 double getAbs();
	
//variables
	double x; //!<Position in x.
	double y; //!<Position in y.
};

#endif
