#ifndef INC_FLIGHT_CONTROL_H_

#ifdef __cplusplus
extern "C"{
#endif

#define INC_FLIGHT_CONTROL_H_

void initIMU(void);
void updateYPR(void);
void computePID(void);
void calculateVelocities(void);
void updateMotors(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* INC_FLIGHT_CONTROL_H_ */
