/************************************************************************/
/* AVR PID Library header file                                          */
/* @author Gazizov A.T. gazizov@tpu.eu                                  */
/************************************************************************/

/** Range of allowed PID output range. Needed to avoid sign/overflow problems. */
#define DEFAULT_MIN_OUT  -255
#define DEFAULT_MAX_OUT  255

/** Sample time in seconds.
 *  How often PID control function (pid_Controller) will be called.
 *  Important setting dependent on timer implementation in your program.
 */
#define SAMPLE_TIME 0.05F

/** Parameters for initializing */
#define INITIAL_INTEGRAL_TERM 0
#define INITIAL_PROCESS_VALUE 0

/** Library functions declaration */
void pid_Init(double p_factor, double i_factor, double d_factor);
void pid_setParams(double Kp, double Ki, double Kd);
void pid_setOutputLimits(int16_t Min, int16_t Max);
double pid_Controller(double setPoint, double processValue);
