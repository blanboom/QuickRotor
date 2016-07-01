#ifndef INC_FLIGHT_CONTROL_H_

#ifdef __cplusplus
extern "C"{
#endif

#define INC_FLIGHT_CONTROL_H_

extern float yaw_set, pitch_set, roll_set, velocity_set;
extern float pitch_Kp;
extern float pitch_Ki;
extern float pitch_Kd;
extern float roll_Kp;
extern float roll_Ki;
extern float roll_Kd;
extern float yaw_Kp;
extern float yaw_Ki;
extern float yaw_Kd;

void initIMU(void);
void updateYPR(void);
void computePID(void);
void calculateVelocities(void);
void updateMotors(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* INC_FLIGHT_CONTROL_H_ */
