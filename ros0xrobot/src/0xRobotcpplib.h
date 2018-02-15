
#ifdef WIN32

#include <Windows.h>
#include <iostream>
#include <iomanip>

#else

#include <stdio.h>													/* Standard input/output definitions */
#include <string.h>													/* String function definitions */
#include <unistd.h>													/* UNIX standard function definitions */
#include <fcntl.h>													/* File control definitions */
#include <errno.h>													/* Error number definitions */
#include <termios.h>												/* POSIX terminal control definitions */
#include <stdbool.h>
#include <sys/time.h>
#include <sys/types.h>
#include <math.h>
typedef		bool	BOOL;
#endif

#define TRUE	1
#define FALSE	0

#define		ERROR_CONNECT			1
#define		ERROR_DISCONNECT		2
#define		ERROR_TRANSMIT			3
#define		ERROR_RECEIVE			4
#define		ERROR_RECEIVE_TIMEOUT	5
#define		ERROR_CHECKSUM			6
#define		ERROR_FAILURE_RESPONSE	7
#define		ERROR_ROBOT_CONNECT		8


typedef	unsigned char	uint8;
typedef char			int8;
typedef unsigned short	uint16;
typedef short			int16;
typedef unsigned int	uint32;
typedef int			    int32;
typedef unsigned long	DWORD;

#ifdef WIN32

    #ifdef FIREBIRDCPPLIBRARY_EXPORTS
        #define FIREBIRDCPPLIBRARY_API __declspec(dllexport)
    #else
        #define FIREBIRDCPPLIBRARY_API __declspec(dllimport)
    #endif

#else

    #ifdef FIREBIRDCPPLIBRARY_EXPORTS
        #define FIREBIRDCPPLIBRARY_API
    #else
        #define FIREBIRDCPPLIBRARY_API
    #endif

#endif

class FIREBIRDCPPLIBRARY_API FireBirdcpp
{
private:
public:
		uint8 Error_Status;
		void *comm_handle;
		void *connect_comm(const char* Port);
		BOOL disconnect_comm(void *hSerial);
		void DelaymSec(void *hSerial,DWORD Delayms);
		BOOL initPeripherals(void *hSerial);
		BOOL buzzerOn(void *hSerial);
		BOOL buzzerOff(void *hSerial);
		BOOL getIRProximitySensorArray(void *hSerial, uint8 *allIRProximityData);
		BOOL getIRProximitySensor(void *hSerial,uint8 sensorNumber,uint8 *irProximityValue);
		BOOL getlineSensorArray(void *hSerial,uint8 *allLineSensorData);
		BOOL getlineSensor(void *hSerial,uint8 sensorNumber,uint8 *lineSensorValue);
		BOOL getIRDistanceSensorArray(void *hSerial, uint8 *allIRDistanceData);
		BOOL getIRDistanceSensor(void *hSerial,uint8 sensorNumber, uint8 *irdistanceValue);
		BOOL getSonarSensorArray(void *hSerial,uint8 *allSonarData);
		BOOL getSonar(void *hSerial, uint8 sensorNumber,uint8 *sonarValue);
		BOOL getServopodSensor(void *hSerial, uint16 *servopodsensorvalue);
		BOOL getBatteryVoltage(void *hSerial, uint8 *batteryVoltage);
		BOOL getBatteryCurrent(void *hSerial,uint8 *batteryCurrent);
		BOOL getBatteryTemprature(void *hSerial, uint8 *batteryTemprature);
		BOOL getAccelerometerXYZ(void *hSerial, int16 *accelerometerXYZ);
		BOOL getGyroXYZ(void *hSerial, int16 *GyroXYZ);
		BOOL getMagnetometerXYZ(void *hSerial, int16 *magnetometerXYZ);
		BOOL getAccelerometerX(void *hSerial, int16 *accelerationX);
		BOOL getAccelerometerY(void *hSerial, int16 *accelerationY);
		BOOL getAccelerometerZ(void *hSerial, int16 *accelerationZ);
		BOOL getGyroX(void *hSerial, int16 *gyroX);
		BOOL getGyroY(void *hSerial, int16 *gyroY);
		BOOL getGyroZ(void *hSerial, int16 *gyroZ);
		BOOL getMagnetometerX(void *hSerial,int16 *magnetometerX);
		BOOL getMagnetometerY(void *hSerial,int16 *magnetometerY);
		BOOL getMagnetometerZ(void *hSerial,int16 *magnetometerZ);
		BOOL setLineSensorON(void *hSerial);
		BOOL setLineSensorOFF(void *hSerial);
		BOOL setIRProximitySensorON(void *hSerial);
		BOOL setIRProximitySensorOFF(void *hSerial);
		BOOL setIRDistanceSensorON(void *hSerial);
		BOOL setIRDistanceSensorOFF(void *hSerial);
		BOOL setSonarSensorON(void *hSerial);
		BOOL setSonarSensorOFF(void *hSerial);
		BOOL setServopodSensorON(void *hSerial);
		BOOL setServopodSensorOFF(void *hSerial);
		BOOL getLineSensorStatus(void *hSerial, uint8 *status);
		BOOL getIRProximitySensorStatus(void *hSerial, uint8 *status);
		BOOL getIRDistanceStatus(void *hSerial, uint8 *status);
		BOOL getSonarStatus(void *hSerial, uint8 *status);
		BOOL getServopodSensorStatus(void *hSerial, uint8 *status);
		BOOL setLeftMotorVelocity(void *hSerial, signed short int velocity);
		BOOL getLeftMotorVelocity(void *hSerial, signed short int *leftMotorVelocity);
		BOOL setRightMotorVelocity(void *hSerial, signed short int velocity);
		BOOL getRightMotorVelocity(void *hSerial, signed short int *rightMotorVelocity);
		BOOL forward(void *hSerial);
		BOOL backward(void *hSerial);
		BOOL right(void *hSerial);
		BOOL left(void *hSerial);
		BOOL stop(void *hSerial);
		BOOL getRightMotorCount(void *hSerial, int32 *rightMotorCount);
		BOOL getLeftMotorCount(void *hSerial, int32 *leftMotorCount);
		BOOL resetMotorEncoderCount(void *hSerial);
		BOOL setPosition(void *hSerial,uint32 leftposition,uint8 leftVelocity,uint32 rightposition,uint8 rightVelocity);
		BOOL setAcceleration(void *hSerial, uint8 acceleration);
		BOOL getAcceleration(void *hSerial, uint8 *acceleration);
		BOOL setMode(void *hSerial, uint8 mode);
		BOOL getMode(void *hSerial, uint8 *mode);
		BOOL setSafety(void *hSerial, uint8 safety);
		BOOL setLinearVelocity_meterspersec(void *hSerial, float meterPersec);
		BOOL setLinearPosition(void *hSerial, float left_dist, float left_vel, float right_dist, float right_vel);
		BOOL setRobotAngularPosition(void *hSerial, float radians, float radianspersec);
		BOOL setVelocity_meterspersec(void *hSerial,float Leftmeterspersec, float rightmeterspersec);
		BOOL setVelocity_radianspersec(void *hSerial,float leftradianspersec, float rightradianspersec);
		BOOL lcdCursor(void *hSerial,uint8 row, uint8 column,uint8 *Error_status);
		BOOL lcdHome(void *hSerial);
		BOOL lcdClear(void *hSerial);
		BOOL lcdPrint(void *hSerial, uint8 row, uint8 column, uint32 value,uint8 digits,uint8 *Error_status);
		BOOL lcdWriteChar(void *hSerial, char letter);
		BOOL lcdWriteString(void *hSerial, uint8 row, uint8 column,char *str, uint8 *Error_status);
		BOOL setPanServo(void *hSerial, uint8 PANangle);
		BOOL setTiltServo(void *hSerial, uint8 TILTtangle);
		BOOL setAuxServo(void *hSerial, uint8 AUXangle);
		BOOL getPanServo(void *hSerial, uint8 *PANdata);
		BOOL getTiltServo(void *hSerial, uint8 *TILTdata);
		BOOL getAuxServo(void *hSerial, uint8 *AUXdata);
		BOOL getADC(void *hSerial, uint16 *AdcData);
		BOOL setRoboticArmServo(void *hSerial, uint8 JointNO, uint8 Angle, uint8 Velocity);
		BOOL setLeftMotorVelocity_meterspersec(void *hSerial, float velocity);
		BOOL setRightMotorVelocity_meterspersec(void *hSerial, float velocity);
		BOOL getLeftMotorVelocity_meterspersec(void *hSerial, float *leftMotorVelocity);
		BOOL getRightMotorVelocity_meterspersec(void *hSerial, float *rightMotorVelocity);
		BOOL setRobotAngularVelocityRadianpersec(void *hSerial, float radianpersec);
		BOOL getRobotAngularVelocityRadianpersec(void *hSerial, float *radianspersec);
		BOOL setLeftMotorVelocity_radianspersec(void *hSerial, float velocity);
		BOOL setRightMotorVelocity_radianspersec(void *hSerial, float velocity);
		BOOL getLeftMotorVelocity_radianspersec(void *hSerial, float *leftMotorVelocity);
		BOOL getRightMotorVelocity_radianspersec(void *hSerial, float *rightMotorVelocity);
		BOOL setRobotID(void *hSerial, uint8 R_id);
		BOOL getRobotID(void *hSerial, uint8 *R_id);
		BOOL getHardwareVersion(void *hSerial, uint8 *HW_version);
		BOOL getSoftwareVersion(void *hSerial, uint8 *SW_version);
		BOOL getADCPotentiometer(void *hSerial, uint16 *adc_Pot);
		BOOL setGPIOPannelLED(void *hSerial, uint8 led_data);
		BOOL getGPIOPannelLED(void *hSerial, uint8 *led_status);
		BOOL getGPIOPannelswitches(void *hSerial, uint8 *switch_status);
		BOOL setSafetyTimeout(void *hSerial, uint8 safety_timeout);
		BOOL getSafetyTimeout(void *hSerial, uint8 *safety_timeout);
		BOOL getErrorStatus(uint8 *error);
		BOOL getLeftmotorCurrent(void *hSerial, float *Left_Amp);
		BOOL getRightmotorCurrent(void *hSerial, float *Right_Amp);
		BOOL getLeftmotorVoltage(void *hSerial, float *Left_V);
		BOOL getRightmotorVoltage(void *hSerial, float *Right_V);
		BOOL setWheelDiameter_mm(void *hSerial, float diameter);
		BOOL getWheelDiameter_mm(void *hSerial, float *Diameter);
		BOOL setRobotAxlelength_mm(void *hSerial, float axle_length);
		BOOL getRobotAxlelength_mm(void *hSerial, float *Axle_Length);
		BOOL setRobotMaxVelocity_meterspersec(void *hSerial, float max_velocity);
		BOOL getRobotMaxVelocity_meterspersec(void *hSerial, float *Max_Velocity);
		BOOL getDeltaPosition(void *hSerial, int32 deltaLeft, int32 deltaRight, double theta, float distancepercount, double axelLength, double *deltaX, double *deltaY, double *deltaTheta);
		BOOL imuInit(void *hSerial, int16 *gyroXYZ_Zero);
		float getHeading(void *hSerial, int16 * magValue);
		float getTiltHeading(void *hSerial, float * magValue, float * accelValue);
		double getVelocityX(void *hSerial, float deltaX, uint8 updateFreq);
		double getVelocityY(void *hSerial, float deltaY, uint8 updateFreq);
		double getVelocityTheta(void *hSerial, float deltaTheta, uint8 updateFreq);
		BOOL setVelocity2D(void *hSerial,float velocityX, float angularZ, float axelLength);
		BOOL getSonarSensorRangeArray(void *hSerial, float *allSonarDataX, float *allSonarDataY);
		BOOL getIRSensorRangeArray(void *hSerial, float *allIRDataX, float *allIRDataY);
};

typedef FireBirdcpp lib0xRobotCpp;


