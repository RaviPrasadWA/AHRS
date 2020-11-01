#ifndef __PID_H__
#define __PID_H__

#include "Math.h"
#include "AP_Param.h"
#include <stdlib.h>
#include <math.h>            

#define PID_D_TERM_FILTER 0.556864f    // 20Hz Cutoff Frequency

class PID
{
	public:
		PID(
        const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const int16_t & initial_imax = 0.0):
	        _integrator(0),
	        _last_input(0),
	        _last_derivative(0),
	        _d_lpf_alpha(PID_D_TERM_FILTER)
        {
        	_kp = initial_p;
	        _ki = initial_i;
	        _kd = initial_d;
	        _imax = abs(initial_imax);

			// derivative is invalid on startup
			_last_derivative = NAN;
        }

     	float       get_pid(float error, float dt);
	float       get_pi(float error, float dt);
	float       get_p(float error) const;
	float       get_i(float error, float dt);
	float       get_d(float error, float dt);

	/// Reset the PID integrator
	///
	void        reset_I();

	    
	    
	/// Sets filter Alpha for D-term LPF
	void        set_d_lpf_alpha(int16_t cutoff_frequency, float time_step);


	/// Overload the function call operator to permit relatively easy initialisation
	void operator() ( const float    p,
                          const float    i,
                          const float    d,
                          const int16_t  imaxval ) 
                         {
                           _kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
                         }

	 // accessors
	 float       kP() const { return _kp.get(); }
	 float       kI() const { return _ki.get(); }
	 float       kD() const { return _kd.get(); }
	 int16_t     imax() const { return _imax.get(); }
	 void        kP(const float v) { _kp.set(v); }
	 void        kI(const float v) { _ki.set(v); }
	 void        kD(const float v) { _kd.set(v); }
	 void        imax(const int16_t v) { _imax.set(abs(v)); }
	 float       get_integrator() const { return _integrator; }
	 void        set_integrator(float i) { _integrator = i; }


	protected:
	  AP_Float        _kp;
	  AP_Float        _ki;
	  AP_Float        _kd;
	  AP_Int16        _imax;

	  float           _integrator;        ///< integrator value
	  float           _last_input;        ///< last input for derivative
	  float           _last_derivative;   ///< last derivative for low-pass filter
	  float           _d_lpf_alpha;       ///< alpha used in D-term LPF

};

class HELICOPTER_PID : public PID
{
  public:
    HELICOPTER_PID( const float &   initial_p = 0.0,
                    const float &   initial_i = 0.0,
                    const float &   initial_d = 0.0,
                    const int16_t & initial_imax = 0.0,
                    const float &   initial_ff = 0.0) :
                      PID(initial_p, initial_i, initial_d, initial_imax)
                      { _ff = initial_ff; }
                      
    /// get_ff - return FeedForward Term 
    float       get_ff(float requested_rate) const;
    
    /// get_leaky_i - replacement for get_i but output is leaded at leak_rate
    float       get_leaky_i(float error, float dt, float leak_rate);
    
    // accessors
    float       ff() const { return _ff.get(); }
    void        ff(const float v) { _ff.set(v); }
  private:
    AP_Float        _ff;
};

class S_P 
{
  public:
    S_P(const float &   initial_p = 0.0)
    {
      _kp = initial_p;
    }
    
    float       get_p(float error) const;
    void operator() (const float p) { _kp = p; }
    float       kP() const { return _kp.get(); }
    void        kP(const float v) { _kp.set(v); }
    
   private:
    AP_Float  _kp;
};


#endif //__PID_H__
