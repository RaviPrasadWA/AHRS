#ifndef AP_PARAM_H
#define AP_PARAM_H

#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "Math.h"

enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_VECTOR6F,
    AP_PARAM_MATRIX3F,
    AP_PARAM_GROUP
};

class AP_Param
{
};

template<typename T, ap_var_type PT>
class AP_ParamT 
{
public:
    static const ap_var_type        vtype = PT;

    /// Value getter
    ///
    const T &get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(const T &v) {
        _value = v;
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator const T &() const {
        return _value;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_ParamT<T,PT>& operator= (const T &v) {
        _value = v;
        return *this;
    }

    /// bit ops on parameters
    ///
    AP_ParamT<T,PT>& operator |=(const T &v) {
        _value |= v;
        return *this;
    }

    AP_ParamT<T,PT>& operator &=(const T &v) {
        _value &= v;
        return *this;
    }

    AP_ParamT<T,PT>& operator +=(const T &v) {
        _value += v;
        return *this;
    }

    AP_ParamT<T,PT>& operator -=(const T &v) {
        _value -= v;
        return *this;
    }

    /// AP_ParamT types can implement AP_Param::cast_to_float
    ///
    float cast_to_float(void) const {
        return (float)_value;
    }

protected:
    T _value;
};


/// Template class for non-scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
/// @tparam PT			AP_PARAM_* type
///
template<typename T, ap_var_type PT>
class AP_ParamV 
{
public:

    static const ap_var_type        vtype = PT;

    /// Value getter
    ///
    const T &get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(const T &v) {
        _value = v;
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator const T &() const {
        return _value;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_ParamV<T,PT>& operator=(const T &v) {
        _value = v;
        return *this;
    }

protected:
    T        _value;
};


/// Template class for array variables.
///
/// Objects created using this template behave like arrays of the type T,
/// but are stored like single variables.
///
/// @tparam T           The scalar type of the variable
/// @tparam N           number of elements
/// @tparam PT          the AP_PARAM_* type
///
template<typename T, uint8_t N, ap_var_type PT>
class AP_ParamA 
{
public:

    static const ap_var_type vtype = PT;

    /// Array operator accesses members.
    ///
    /// @note It would be nice to range-check i here, but then what would we return?
    ///
    const T & operator[](uint8_t i) {
        return _value[i];
    }

    const T & operator[](int8_t i) {
        return _value[(uint8_t)i];
    }

    /// Value getter
    ///
    /// @note   Returns zero for index values out of range.
    ///
    T get(uint8_t i) const {
        if (i < N) {
            return _value[i];
        } else {
            return (T)0;
        }
    }

    /// Value setter
    ///
    /// @note   Attempts to set an index out of range are discarded.
    ///
    void  set(uint8_t i, const T &v) {
        if (i < N) {
            _value[i] = v;
        }
    }

protected:
    T _value[N];
};



/// Convenience macro for defining instances of the AP_ParamT template.
///
// declare a scalar type
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _pt is the enum ap_var_type type
#define AP_PARAMDEF(_t, _suffix, _pt)   typedef AP_ParamT<_t, _pt> AP_ ## _suffix;

AP_PARAMDEF(float, Float, AP_PARAM_FLOAT);    // defines AP_Float
AP_PARAMDEF(int8_t, Int8, AP_PARAM_INT8);     // defines AP_Int8
AP_PARAMDEF(int16_t, Int16, AP_PARAM_INT16);  // defines AP_Int16
AP_PARAMDEF(int32_t, Int32, AP_PARAM_INT32);  // defines AP_Int32

// declare an array type
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _size is the size of the array
// _pt is the enum ap_var_type type
#define AP_PARAMDEFA(_t, _suffix, _size, _pt)   typedef AP_ParamA<_t, _size, _pt> AP_ ## _suffix;
AP_PARAMDEFA(float, Vector6f, 6, AP_PARAM_VECTOR6F);

// declare a non-scalar type
// this is used in AP_Math.h
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _pt is the enum ap_var_type type
#define AP_PARAMDEFV(_t, _suffix, _pt)   typedef AP_ParamV<_t, _pt> AP_ ## _suffix;

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Matrix3f, Matrix3f, AP_PARAM_MATRIX3F);
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);



#endif // AP_PARAM_H
