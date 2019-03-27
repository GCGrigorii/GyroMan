/************************************************************************/
/* AVR PID Library source file                                          */
/* @author Gazizov A.T. gazizov@tpu.eu                                  */
/************************************************************************/
#include "stdint.h"
#include "pid.h"

/** Structure for storing PID data between calculations */
typedef struct PID_DATA {
  int16_t lastProcessValue;
  int32_t integralTerm;
  double kp;
  double ki;
  double kd;
  int16_t MAX_OUT;
  int16_t MIN_OUT;
} pidData_t;

struct PID_DATA pidData;

/* PID controller initialization. Should be called only once.
 *  @param kp - proportional coefficient.
 *  @param ki - integral coefficient.
 *  @param kd - differential coefficient.
 */
void pid_Init(double kp, double ki, double kd)
{
  pidData.integralTerm     = INITIAL_INTEGRAL_TERM;
  pidData.lastProcessValue = INITIAL_PROCESS_VALUE;
  pid_setParams(kp, ki, kd);
  pid_setOutputLimits(DEFAULT_MIN_OUT, DEFAULT_MAX_OUT);
}

/* Main PID controller function. It should be called regularly, each SAMPLE_TIME sec.
 * @param setPoint - level desired at output of the control object.
 * @param processValue - level obtained from the control object.
 * @returns input to control object.
 */
double pid_Controller(int16_t setPoint, int16_t processValue)
{
  double error, p_term, d_term;
  double out;

  error = setPoint - processValue;

  pidData.integralTerm += pidData.ki * error;
  if (pidData.integralTerm > pidData.MAX_OUT) {
    pidData.integralTerm = pidData.MAX_OUT;
	out = pidData.MAX_OUT;
  } else {
	d_term = pidData.kd * (processValue - pidData.lastProcessValue);
	p_term = pidData.kp * error;
	out = (p_term + pidData.integralTerm - d_term);
	if (out > pidData.MAX_OUT) {
		out = pidData.MAX_OUT;
		} else if (out < pidData.MIN_OUT){
		out = pidData.MIN_OUT;
	}
  }
  pidData.lastProcessValue = processValue;

  return out;
}

/** Restricts PID controller output.
 *  @param Min - minimal allowed output value.
 *  @param Min - maximal allowed output value.
 */
void pid_setOutputLimits(int16_t Min, int16_t Max)
{
	if(Min > Max) return;
	pidData.MIN_OUT = Min;
	pidData.MAX_OUT = Max;
}

/** Set PID controller params with consideration of sampling time.
 *  @param Kp - proportional coefficient.
 *  @param Ki - integral coefficient.
 *  @param Kd - differential coefficient.
 */
void pid_setParams(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pidData.kp = Kp;

   pidData.ki = Ki * SAMPLE_TIME;
   pidData.kd = Kd / SAMPLE_TIME;
}
