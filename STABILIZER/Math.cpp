#include "Math.h"

// degrees -> radians
float radians_custom(float deg) {
	return deg * DEG_TO_RAD_SPREC;
}

// radians -> degrees
float degrees_custom(float rad) {
	return rad * RAD_TO_DEG_SPREC;
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return PI/2;
    }
    if (v <= -1.0f) {
        return -PI/2;
    }
    return asinf(v);
}

// square
float sq_custom(float v) {
	return v*v;
}

// 2D vector length
float pythagorous2(float a, float b) {
	return sqrtf(sq_custom(a)+sq_custom(b));
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
	return sqrtf(sq_custom(a)+sq_custom(b)+sq_custom(c));
}

// constrain a value
float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*
  wrap an angle in centi-degrees to 0..35999
 */
int32_t wrap_360_cd(int32_t error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    while (error >= 36000) error -= 36000;
    while (error < 0) error += 36000;
    return error;
}

/*
  wrap an angle in centi-degrees to -18000..18000
 */
int32_t wrap_180_cd(int32_t error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    while (error > 18000) { error -= 36000; }
    while (error < -18000) { error += 36000; }
    return error;
}

/*
  wrap an angle in centi-degrees to 0..35999
 */
float wrap_360_cd_float(float angle)
{
    if (angle >= 72000.0f || angle < -36000.0f) {
        // for larger number use fmodulus
        angle = fmod(angle, 36000.0f);
    }
    if (angle >= 36000.0f) angle -= 36000.0f;
    if (angle < 0.0f) angle += 36000.0f;
    return angle;
}

/*
  wrap an angle in centi-degrees to -18000..18000
 */
float wrap_180_cd_float(float angle)
{
    if (angle > 54000.0f || angle < -54000.0f) {
        // for large numbers use modulus
        angle = fmod(angle,36000.0f);
    }
    if (angle > 18000.0f) { angle -= 36000.0f; }
    if (angle < -18000.0f) { angle += 36000.0f; }
    return angle;
}



