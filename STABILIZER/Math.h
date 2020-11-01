#ifndef __MATH_H__
#define __MATH_H__

#include <stdint.h>
#include <math.h>
#include "rotations.h"
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"

#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
#ifndef PI
 # define PI M_PI_F
#endif
#ifndef M_PI_2
 # define M_PI_2 1.570796326794897f
#endif
//Single precision conversions
#define DEG_TO_RAD_SPREC 0.017453292519943295769236907684886f
#define RAD_TO_DEG_SPREC 57.295779513082320876798154814105f

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS 9.80665f

#ifndef sqrtf
	#define sqrtf sqrt
#endif

// degrees -> radians
float radians_custom(float deg);

// radians -> degrees
float degrees_custom(float rad);

// a varient of asin() that always gives a valid answer.
float           safe_asin(float v);

// a varient of sqrt() that always gives a valid answer.
float           safe_sqrt(float v);

// square
float sq_custom(float v);

// constrain a value
float   constrain_float(float amt, float low, float high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);

// sqrt of sum of squares
float pythagorous2(float a, float b);
float pythagorous3(float a, float b, float c);

/*
  wrap an angle in centi-degrees
 */
int32_t wrap_360_cd(int32_t error);
int32_t wrap_180_cd(int32_t error);
float wrap_360_cd_float(float angle);
float wrap_180_cd_float(float angle);


#define ToRad(x) radians_custom(x)	// *pi/180
#define ToDeg(x) degrees_custom(x)	// *180/pi


#endif // __MATH_H__

