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
	
	/*! \brief A constructor.
	 * 
	 * @param x Position in x.
	 * @param y Position in y.
	 */
	MyPoint(double xPos, double yPos);
	
	/*! \brief A destructor.
	 */
	~MyPoint();
	
	/*! \brief Returns angle between this and target point.
	 * 
	 * @param target Target point.
	 */
	
	double getAngle(MyPoint* target);
	
	/*! \brief Returns distance between this and target point.
	 * 
	 * @param target Target point.
	 */
	
	double getDistance(MyPoint* target);
	
	/*! Function +.
	 */
	MyPoint operator+(const MyPoint &other) const;
	
	/*! Function -.
	 */
	MyPoint operator-(const MyPoint &other) const;
	
	/*! Function *real_number.
	 */
	MyPoint times(double r);
	
	/*! Absolute value of vector.
	 */ 
	 double getAbs();
	
//variables
	double x; //!<Position in x.
	double y; //!<Position in y.
};

#endif
