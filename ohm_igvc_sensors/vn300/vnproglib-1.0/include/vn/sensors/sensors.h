// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#ifndef _VNSENSORS_SENSORS_H_
#define _VNSENSORS_SENSORS_H_

#if defined(_MSC_VER)
	/* We are using Visual Studio. Suppress warnings about unsafe library functions. */
	#define _CRT_SECURE_NO_WARNINGS
	#define _SCL_SECURE_NO_WARNINGS
#endif

#include <string>
#include <vector>

#include "vn/int.h"
#include "vn/common/isimpleport.h"
#include "vn/util/nocopy.h"
#include "vn/protocol/uart.h"

namespace vn {
namespace sensors {

/// \brief Represents an error from a VectorNav sensor.
struct sensor_error : public std::exception
{
private:
	sensor_error();

public:

	/// \brief Creates a new sensor_error based on the error value provided by
	/// the sensor.
	explicit sensor_error(protocol::uart::SensorError e);

	/// \brief Copy constructor.
	sensor_error(const sensor_error& e);

	~sensor_error() throw();

	/// \brief Returns a description of the exception.
	///
	/// \return A description of the exception.
	char const* what() const throw();

	/// \brief The associated sensor error.
	protocol::uart::SensorError error;

private:
	char *_errorMessage;
};

/// \defgroup registerStructures Register Structures
/// \brief These structures represent the various registers on a VecotorNav
/// sensor.
///
/// \{

/// \brief Structure representing a Binary Output register.
///
/// The field outputGroup available on the sensor's register is not necessary
/// in this structure since all read/writes operations will automatically
/// determine this from the settings for the individual groups within this
/// structure.
struct BinaryOutputRegister
{
	protocol::uart::AsyncMode asyncMode;			///< The asyncMode field.
	uint16_t rateDivisor;							///< The rateDivisor field.
	protocol::uart::CommonGroup commonField;		///< Group 1 (Common)
	protocol::uart::TimeGroup timeField;			///< Group 2 (Time)
	protocol::uart::ImuGroup imuField;				///< Group 3 (IMU)
	protocol::uart::GpsGroup gpsField;				///< Group 4 (GPS)
	protocol::uart::AttitudeGroup attitudeField;	///< Group 5 (Attitude)
	protocol::uart::InsGroup insField;				///< Group 6 (INS)

	BinaryOutputRegister() :
		asyncMode(protocol::uart::ASYNCMODE_NONE),
		rateDivisor(0),
		commonField(protocol::uart::COMMONGROUP_NONE),
		timeField(protocol::uart::TIMEGROUP_NONE),
		imuField(protocol::uart::IMUGROUP_NONE),
		gpsField(protocol::uart::GPSGROUP_NONE),
		attitudeField(protocol::uart::ATTITUDEGROUP_NONE),
		insField(protocol::uart::INSGROUP_NONE)
	{ }

	/// \brief Creates an initializes a new BinaryOutputRegister structure.
	///
	/// \param[in] asyncModeIn Value to initialize the asyncMode field with.
	/// \param[in] rateDivisorIn Value to initialize the rateDivisor field with.
	/// \param[in] commonFieldIn Value to initialize field 1 (Common) with.
	/// \param[in] timeFieldIn Value to initialize field 2 (Time) with.
	/// \param[in] imuFieldIn Value to initialize field 3 (IMU) with.
	/// \param[in] gpsFieldIn Value to initialize field 4 (GPS) with.
	/// \param[in] attitudeFieldIn Value to initialize field 5 (Attitude) with.
	/// \param[in] insFieldIn Value to initialize field 6 (INS) with.
	BinaryOutputRegister(
		protocol::uart::AsyncMode asyncModeIn,
		uint16_t rateDivisorIn,
		protocol::uart::CommonGroup commonFieldIn,
		protocol::uart::TimeGroup timeFieldIn,
		protocol::uart::ImuGroup imuFieldIn,
		protocol::uart::GpsGroup gpsFieldIn,
		protocol::uart::AttitudeGroup attitudeFieldIn,
		protocol::uart::InsGroup insFieldIn) :
		asyncMode(asyncModeIn),
		rateDivisor(rateDivisorIn),
		commonField(commonFieldIn),
		timeField(timeFieldIn),
		imuField(imuFieldIn),
		gpsField(gpsFieldIn),
		attitudeField(attitudeFieldIn),
		insField(insFieldIn)
	{ }

};

/// \brief Structure representing the Synchronization Control register.
struct SynchronizationControlRegister
{
	protocol::uart::SyncInMode syncInMode; ///< The syncInMode field.
	protocol::uart::SyncInEdge syncInEdge; ///< The syncInEdge field.
	uint16_t syncInSkipFactor; ///< The syncInSkipFactor field.
	protocol::uart::SyncOutMode syncOutMode; ///< The syncOutMode field.
	protocol::uart::SyncOutPolarity syncOutPolarity; ///< The syncOutPolarity field.
	uint16_t syncOutSkipFactor; ///< The syncOutSkipFactor field.
	uint32_t syncOutPulseWidth; ///< The syncOutPulseWidth field.

	SynchronizationControlRegister() { }

	/// \brief Creates an initializes a new SynchronizationControlRegister structure.
	///
	/// \param[in] syncInModeIn Value to initialize the syncInMode field with.
	/// \param[in] syncInEdgeIn Value to initialize the syncInEdge field with.
	/// \param[in] syncInSkipFactorIn Value to initialize the syncInSkipFactor field with.
	/// \param[in] syncOutModeIn Value to initialize the syncOutMode field with.
	/// \param[in] syncOutPolarityIn Value to initialize the syncOutPolarity field with.
	/// \param[in] syncOutSkipFactorIn Value to initialize the syncOutSkipFactor field with.
	/// \param[in] syncOutPulseWidthIn Value to initialize the syncOutPulseWidth field with.
	SynchronizationControlRegister(
		protocol::uart::SyncInMode syncInModeIn,
		protocol::uart::SyncInEdge syncInEdgeIn,
		uint16_t syncInSkipFactorIn,
		protocol::uart::SyncOutMode syncOutModeIn,
		protocol::uart::SyncOutPolarity syncOutPolarityIn,
		uint16_t syncOutSkipFactorIn,
		uint32_t syncOutPulseWidthIn) :
		syncInMode(syncInModeIn),
		syncInEdge(syncInEdgeIn),
		syncInSkipFactor(syncInSkipFactorIn),
		syncOutMode(syncOutModeIn),
		syncOutPolarity(syncOutPolarityIn),
		syncOutSkipFactor(syncOutSkipFactorIn),
		syncOutPulseWidth(syncOutPulseWidthIn)
	{ }

};

/// \brief Structure representing the Communication Protocol Control register.
struct CommunicationProtocolControlRegister
{
	protocol::uart::CountMode serialCount; ///< The serialCount field.
	protocol::uart::StatusMode serialStatus; ///< The serialStatus field.
	protocol::uart::CountMode spiCount; ///< The spiCount field.
	protocol::uart::StatusMode spiStatus; ///< The spiStatus field.
	protocol::uart::ChecksumMode serialChecksum; ///< The serialChecksum field.
	protocol::uart::ChecksumMode spiChecksum; ///< The spiChecksum field.
	protocol::uart::ErrorMode errorMode; ///< The errorMode field.

	CommunicationProtocolControlRegister() { }

	/// \brief Creates an initializes a new CommunicationProtocolControlRegister structure.
	///
	/// \param[in] serialCountIn Value to initialize the serialCount field with.
	/// \param[in] serialStatusIn Value to initialize the serialStatus field with.
	/// \param[in] spiCountIn Value to initialize the spiCount field with.
	/// \param[in] spiStatusIn Value to initialize the spiStatus field with.
	/// \param[in] serialChecksumIn Value to initialize the serialChecksum field with.
	/// \param[in] spiChecksumIn Value to initialize the spiChecksum field with.
	/// \param[in] errorModeIn Value to initialize the errorMode field with.
	CommunicationProtocolControlRegister(
		protocol::uart::CountMode serialCountIn,
		protocol::uart::StatusMode serialStatusIn,
		protocol::uart::CountMode spiCountIn,
		protocol::uart::StatusMode spiStatusIn,
		protocol::uart::ChecksumMode serialChecksumIn,
		protocol::uart::ChecksumMode spiChecksumIn,
		protocol::uart::ErrorMode errorModeIn) :
		serialCount(serialCountIn),
		serialStatus(serialStatusIn),
		spiCount(spiCountIn),
		spiStatus(spiStatusIn),
		serialChecksum(serialChecksumIn),
		spiChecksum(spiChecksumIn),
		errorMode(errorModeIn)
	{ }

};

/// \brief Structure representing the Synchronization Status register.
struct SynchronizationStatusRegister
{
	uint32_t syncInCount; ///< The syncInCount field.
	uint32_t syncInTime; ///< The syncInTime field.
	uint32_t syncOutCount; ///< The syncOutCount field.

	SynchronizationStatusRegister() { }

	/// \brief Creates an initializes a new SynchronizationStatusRegister structure.
	///
	/// \param[in] syncInCountIn Value to initialize the syncInCount field with.
	/// \param[in] syncInTimeIn Value to initialize the syncInTime field with.
	/// \param[in] syncOutCountIn Value to initialize the syncOutCount field with.
	SynchronizationStatusRegister(
		uint32_t syncInCountIn,
		uint32_t syncInTimeIn,
		uint32_t syncOutCountIn) :
		syncInCount(syncInCountIn),
		syncInTime(syncInTimeIn),
		syncOutCount(syncOutCountIn)
	{ }

};

/// \brief Structure representing the IMU Measurements register.
struct ImuMeasurementsRegister
{
	math::vec3f mag; ///< The mag field.
	math::vec3f accel; ///< The accel field.
	math::vec3f gyro; ///< The gyro field.
	float temp; ///< The temp field.
	float pressure; ///< The pressure field.

	ImuMeasurementsRegister() { }

	/// \brief Creates an initializes a new ImuMeasurementsRegister structure.
	///
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	/// \param[in] tempIn Value to initialize the temp field with.
	/// \param[in] pressureIn Value to initialize the pressure field with.
	ImuMeasurementsRegister(
		math::vec3f magIn,
		math::vec3f accelIn,
		math::vec3f gyroIn,
		float tempIn,
		float pressureIn) :
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn),
		temp(tempIn),
		pressure(pressureIn)
	{ }

};

/// \brief Structure representing the Delta Theta and Delta Velocity register.
struct DeltaThetaAndDeltaVelocityRegister
{
	float deltaTime; ///< The deltaTime field.
	math::vec3f deltaTheta; ///< The deltaTheta field.
	math::vec3f deltaVelocity; ///< The deltaVelocity field.

	DeltaThetaAndDeltaVelocityRegister() { }

	/// \brief Creates an initializes a new DeltaThetaAndDeltaVelocityRegister structure.
	///
	/// \param[in] deltaTimeIn Value to initialize the deltaTime field with.
	/// \param[in] deltaThetaIn Value to initialize the deltaTheta field with.
	/// \param[in] deltaVelocityIn Value to initialize the deltaVelocity field with.
	DeltaThetaAndDeltaVelocityRegister(
		float deltaTimeIn,
		math::vec3f deltaThetaIn,
		math::vec3f deltaVelocityIn) :
		deltaTime(deltaTimeIn),
		deltaTheta(deltaThetaIn),
		deltaVelocity(deltaVelocityIn)
	{ }

};

/// \brief Structure representing the Magnetometer Compensation register.
struct MagnetometerCompensationRegister
{
	math::mat3f c; ///< The c field.
	math::vec3f b; ///< The b field.

	MagnetometerCompensationRegister() { }

	/// \brief Creates an initializes a new MagnetometerCompensationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	MagnetometerCompensationRegister(
		math::mat3f cIn,
		math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the Acceleration Compensation register.
struct AccelerationCompensationRegister
{
	math::mat3f c; ///< The c field.
	math::vec3f b; ///< The b field.

	AccelerationCompensationRegister() { }

	/// \brief Creates an initializes a new AccelerationCompensationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	AccelerationCompensationRegister(
		math::mat3f cIn,
		math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the Gyro Compensation register.
struct GyroCompensationRegister
{
	math::mat3f c; ///< The c field.
	math::vec3f b; ///< The b field.

	GyroCompensationRegister() { }

	/// \brief Creates an initializes a new GyroCompensationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	GyroCompensationRegister(
		math::mat3f cIn,
		math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the IMU Filtering Configuration register.
struct ImuFilteringConfigurationRegister
{
	uint16_t magWindowSize; ///< The magWindowSize field.
	uint16_t accelWindowSize; ///< The accelWindowSize field.
	uint16_t gyroWindowSize; ///< The gyroWindowSize field.
	uint16_t tempWindowSize; ///< The tempWindowSize field.
	uint16_t presWindowSize; ///< The presWindowSize field.
	protocol::uart::FilterMode magFilterMode; ///< The magFilterMode field.
	protocol::uart::FilterMode accelFilterMode; ///< The accelFilterMode field.
	protocol::uart::FilterMode gyroFilterMode; ///< The gyroFilterMode field.
	protocol::uart::FilterMode tempFilterMode; ///< The tempFilterMode field.
	protocol::uart::FilterMode presFilterMode; ///< The presFilterMode field.

	ImuFilteringConfigurationRegister() { }

	/// \brief Creates an initializes a new ImuFilteringConfigurationRegister structure.
	///
	/// \param[in] magWindowSizeIn Value to initialize the magWindowSize field with.
	/// \param[in] accelWindowSizeIn Value to initialize the accelWindowSize field with.
	/// \param[in] gyroWindowSizeIn Value to initialize the gyroWindowSize field with.
	/// \param[in] tempWindowSizeIn Value to initialize the tempWindowSize field with.
	/// \param[in] presWindowSizeIn Value to initialize the presWindowSize field with.
	/// \param[in] magFilterModeIn Value to initialize the magFilterMode field with.
	/// \param[in] accelFilterModeIn Value to initialize the accelFilterMode field with.
	/// \param[in] gyroFilterModeIn Value to initialize the gyroFilterMode field with.
	/// \param[in] tempFilterModeIn Value to initialize the tempFilterMode field with.
	/// \param[in] presFilterModeIn Value to initialize the presFilterMode field with.
	ImuFilteringConfigurationRegister(
		uint16_t magWindowSizeIn,
		uint16_t accelWindowSizeIn,
		uint16_t gyroWindowSizeIn,
		uint16_t tempWindowSizeIn,
		uint16_t presWindowSizeIn,
		protocol::uart::FilterMode magFilterModeIn,
		protocol::uart::FilterMode accelFilterModeIn,
		protocol::uart::FilterMode gyroFilterModeIn,
		protocol::uart::FilterMode tempFilterModeIn,
		protocol::uart::FilterMode presFilterModeIn) :
		magWindowSize(magWindowSizeIn),
		accelWindowSize(accelWindowSizeIn),
		gyroWindowSize(gyroWindowSizeIn),
		tempWindowSize(tempWindowSizeIn),
		presWindowSize(presWindowSizeIn),
		magFilterMode(magFilterModeIn),
		accelFilterMode(accelFilterModeIn),
		gyroFilterMode(gyroFilterModeIn),
		tempFilterMode(tempFilterModeIn),
		presFilterMode(presFilterModeIn)
	{ }

};

/// \brief Structure representing the Delta Theta and Delta Velocity Configuration register.
struct DeltaThetaAndDeltaVelocityConfigurationRegister
{
	protocol::uart::IntegrationFrame integrationFrame; ///< The integrationFrame field.
	protocol::uart::CompensationMode gyroCompensation; ///< The gyroCompensation field.
	protocol::uart::CompensationMode accelCompensation; ///< The accelCompensation field.

	DeltaThetaAndDeltaVelocityConfigurationRegister() { }

	/// \brief Creates an initializes a new DeltaThetaAndDeltaVelocityConfigurationRegister structure.
	///
	/// \param[in] integrationFrameIn Value to initialize the integrationFrame field with.
	/// \param[in] gyroCompensationIn Value to initialize the gyroCompensation field with.
	/// \param[in] accelCompensationIn Value to initialize the accelCompensation field with.
	DeltaThetaAndDeltaVelocityConfigurationRegister(
		protocol::uart::IntegrationFrame integrationFrameIn,
		protocol::uart::CompensationMode gyroCompensationIn,
		protocol::uart::CompensationMode accelCompensationIn) :
		integrationFrame(integrationFrameIn),
		gyroCompensation(gyroCompensationIn),
		accelCompensation(accelCompensationIn)
	{ }

};

/// \brief Structure representing the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
struct YawPitchRollMagneticAccelerationAndAngularRatesRegister
{
	math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	math::vec3f mag; ///< The mag field.
	math::vec3f accel; ///< The accel field.
	math::vec3f gyro; ///< The gyro field.

	YawPitchRollMagneticAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new YawPitchRollMagneticAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	YawPitchRollMagneticAccelerationAndAngularRatesRegister(
		math::vec3f yawPitchRollIn,
		math::vec3f magIn,
		math::vec3f accelIn,
		math::vec3f gyroIn) :
		yawPitchRoll(yawPitchRollIn),
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Quaternion, Magnetic, Acceleration and Angular Rates register.
struct QuaternionMagneticAccelerationAndAngularRatesRegister
{
	math::vec4f quat; ///< The quat field.
	math::vec3f mag; ///< The mag field.
	math::vec3f accel; ///< The accel field.
	math::vec3f gyro; ///< The gyro field.

	QuaternionMagneticAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new QuaternionMagneticAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] quatIn Value to initialize the quat field with.
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	QuaternionMagneticAccelerationAndAngularRatesRegister(
		math::vec4f quatIn,
		math::vec3f magIn,
		math::vec3f accelIn,
		math::vec3f gyroIn) :
		quat(quatIn),
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Magnetic, Acceleration and Angular Rates register.
struct MagneticAccelerationAndAngularRatesRegister
{
	math::vec3f mag; ///< The mag field.
	math::vec3f accel; ///< The accel field.
	math::vec3f gyro; ///< The gyro field.

	MagneticAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new MagneticAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	MagneticAccelerationAndAngularRatesRegister(
		math::vec3f magIn,
		math::vec3f accelIn,
		math::vec3f gyroIn) :
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
struct YawPitchRollTrueBodyAccelerationAndAngularRatesRegister
{
	math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	math::vec3f bodyAccel; ///< The bodyAccel field.
	math::vec3f gyro; ///< The gyro field.

	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new YawPitchRollTrueBodyAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] bodyAccelIn Value to initialize the bodyAccel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister(
		math::vec3f yawPitchRollIn,
		math::vec3f bodyAccelIn,
		math::vec3f gyroIn) :
		yawPitchRoll(yawPitchRollIn),
		bodyAccel(bodyAccelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
struct YawPitchRollTrueInertialAccelerationAndAngularRatesRegister
{
	math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	math::vec3f inertialAccel; ///< The inertialAccel field.
	math::vec3f gyro; ///< The gyro field.

	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new YawPitchRollTrueInertialAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] inertialAccelIn Value to initialize the inertialAccel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister(
		math::vec3f yawPitchRollIn,
		math::vec3f inertialAccelIn,
		math::vec3f gyroIn) :
		yawPitchRoll(yawPitchRollIn),
		inertialAccel(inertialAccelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the VPE Basic Control register.
struct VpeBasicControlRegister
{
	protocol::uart::VpeEnable enable; ///< The enable field.
	protocol::uart::HeadingMode headingMode; ///< The headingMode field.
	protocol::uart::VpeMode filteringMode; ///< The filteringMode field.
	protocol::uart::VpeMode tuningMode; ///< The tuningMode field.

	VpeBasicControlRegister() { }

	/// \brief Creates an initializes a new VpeBasicControlRegister structure.
	///
	/// \param[in] enableIn Value to initialize the enable field with.
	/// \param[in] headingModeIn Value to initialize the headingMode field with.
	/// \param[in] filteringModeIn Value to initialize the filteringMode field with.
	/// \param[in] tuningModeIn Value to initialize the tuningMode field with.
	VpeBasicControlRegister(
		protocol::uart::VpeEnable enableIn,
		protocol::uart::HeadingMode headingModeIn,
		protocol::uart::VpeMode filteringModeIn,
		protocol::uart::VpeMode tuningModeIn) :
		enable(enableIn),
		headingMode(headingModeIn),
		filteringMode(filteringModeIn),
		tuningMode(tuningModeIn)
	{ }

};

/// \brief Structure representing the VPE Magnetometer Basic Tuning register.
struct VpeMagnetometerBasicTuningRegister
{
	math::vec3f baseTuning; ///< The baseTuning field.
	math::vec3f adaptiveTuning; ///< The adaptiveTuning field.
	math::vec3f adaptiveFiltering; ///< The adaptiveFiltering field.

	VpeMagnetometerBasicTuningRegister() { }

	/// \brief Creates an initializes a new VpeMagnetometerBasicTuningRegister structure.
	///
	/// \param[in] baseTuningIn Value to initialize the baseTuning field with.
	/// \param[in] adaptiveTuningIn Value to initialize the adaptiveTuning field with.
	/// \param[in] adaptiveFilteringIn Value to initialize the adaptiveFiltering field with.
	VpeMagnetometerBasicTuningRegister(
		math::vec3f baseTuningIn,
		math::vec3f adaptiveTuningIn,
		math::vec3f adaptiveFilteringIn) :
		baseTuning(baseTuningIn),
		adaptiveTuning(adaptiveTuningIn),
		adaptiveFiltering(adaptiveFilteringIn)
	{ }

};

/// \brief Structure representing the VPE Accelerometer Basic Tuning register.
struct VpeAccelerometerBasicTuningRegister
{
	math::vec3f baseTuning; ///< The baseTuning field.
	math::vec3f adaptiveTuning; ///< The adaptiveTuning field.
	math::vec3f adaptiveFiltering; ///< The adaptiveFiltering field.

	VpeAccelerometerBasicTuningRegister() { }

	/// \brief Creates an initializes a new VpeAccelerometerBasicTuningRegister structure.
	///
	/// \param[in] baseTuningIn Value to initialize the baseTuning field with.
	/// \param[in] adaptiveTuningIn Value to initialize the adaptiveTuning field with.
	/// \param[in] adaptiveFilteringIn Value to initialize the adaptiveFiltering field with.
	VpeAccelerometerBasicTuningRegister(
		math::vec3f baseTuningIn,
		math::vec3f adaptiveTuningIn,
		math::vec3f adaptiveFilteringIn) :
		baseTuning(baseTuningIn),
		adaptiveTuning(adaptiveTuningIn),
		adaptiveFiltering(adaptiveFilteringIn)
	{ }

};

/// \brief Structure representing the Magnetometer Calibration Control register.
struct MagnetometerCalibrationControlRegister
{
	protocol::uart::HsiMode hsiMode; ///< The hsiMode field.
	protocol::uart::HsiOutput hsiOutput; ///< The hsiOutput field.
	uint8_t convergeRate; ///< The convergeRate field.

	MagnetometerCalibrationControlRegister() { }

	/// \brief Creates an initializes a new MagnetometerCalibrationControlRegister structure.
	///
	/// \param[in] hsiModeIn Value to initialize the hsiMode field with.
	/// \param[in] hsiOutputIn Value to initialize the hsiOutput field with.
	/// \param[in] convergeRateIn Value to initialize the convergeRate field with.
	MagnetometerCalibrationControlRegister(
		protocol::uart::HsiMode hsiModeIn,
		protocol::uart::HsiOutput hsiOutputIn,
		uint8_t convergeRateIn) :
		hsiMode(hsiModeIn),
		hsiOutput(hsiOutputIn),
		convergeRate(convergeRateIn)
	{ }

};

/// \brief Structure representing the Calculated Magnetometer Calibration register.
struct CalculatedMagnetometerCalibrationRegister
{
	math::mat3f c; ///< The c field.
	math::vec3f b; ///< The b field.

	CalculatedMagnetometerCalibrationRegister() { }

	/// \brief Creates an initializes a new CalculatedMagnetometerCalibrationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	CalculatedMagnetometerCalibrationRegister(
		math::mat3f cIn,
		math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the Velocity Compensation Control register.
struct VelocityCompensationControlRegister
{
	uint8_t mode; ///< The mode field.
	float velocityTuning; ///< The velocityTuning field.
	float rateTuning; ///< The rateTuning field.

	VelocityCompensationControlRegister() { }

	/// \brief Creates an initializes a new VelocityCompensationControlRegister structure.
	///
	/// \param[in] modeIn Value to initialize the mode field with.
	/// \param[in] velocityTuningIn Value to initialize the velocityTuning field with.
	/// \param[in] rateTuningIn Value to initialize the rateTuning field with.
	VelocityCompensationControlRegister(
		uint8_t modeIn,
		float velocityTuningIn,
		float rateTuningIn) :
		mode(modeIn),
		velocityTuning(velocityTuningIn),
		rateTuning(rateTuningIn)
	{ }

};

/// \brief Structure representing the Velocity Compensation Status register.
struct VelocityCompensationStatusRegister
{
	float x; ///< The x field.
	float xDot; ///< The xDot field.
	math::vec3f accelOffset; ///< The accelOffset field.
	math::vec3f omega; ///< The omega field.

	VelocityCompensationStatusRegister() { }

	/// \brief Creates an initializes a new VelocityCompensationStatusRegister structure.
	///
	/// \param[in] xIn Value to initialize the x field with.
	/// \param[in] xDotIn Value to initialize the xDot field with.
	/// \param[in] accelOffsetIn Value to initialize the accelOffset field with.
	/// \param[in] omegaIn Value to initialize the omega field with.
	VelocityCompensationStatusRegister(
		float xIn,
		float xDotIn,
		math::vec3f accelOffsetIn,
		math::vec3f omegaIn) :
		x(xIn),
		xDot(xDotIn),
		accelOffset(accelOffsetIn),
		omega(omegaIn)
	{ }

};

/// \brief Structure representing the Magnetic and Gravity Reference Vectors register.
struct MagneticAndGravityReferenceVectorsRegister
{
	math::vec3f magRef; ///< The magRef field.
	math::vec3f accRef; ///< The accRef field.

	MagneticAndGravityReferenceVectorsRegister() { }

	/// \brief Creates an initializes a new MagneticAndGravityReferenceVectorsRegister structure.
	///
	/// \param[in] magRefIn Value to initialize the magRef field with.
	/// \param[in] accRefIn Value to initialize the accRef field with.
	MagneticAndGravityReferenceVectorsRegister(
		math::vec3f magRefIn,
		math::vec3f accRefIn) :
		magRef(magRefIn),
		accRef(accRefIn)
	{ }

};

/// \brief Structure representing the Reference Vector Configuration register.
struct ReferenceVectorConfigurationRegister
{
	bool useMagModel; ///< The useMagModel field.
	bool useGravityModel; ///< The useGravityModel field.
	uint32_t recalcThreshold; ///< The recalcThreshold field.
	float year; ///< The year field.
	math::vec3d position; ///< The position field.

	ReferenceVectorConfigurationRegister() { }

	/// \brief Creates an initializes a new ReferenceVectorConfigurationRegister structure.
	///
	/// \param[in] useMagModelIn Value to initialize the useMagModel field with.
	/// \param[in] useGravityModelIn Value to initialize the useGravityModel field with.
	/// \param[in] recalcThresholdIn Value to initialize the recalcThreshold field with.
	/// \param[in] yearIn Value to initialize the year field with.
	/// \param[in] positionIn Value to initialize the position field with.
	ReferenceVectorConfigurationRegister(
		bool useMagModelIn,
		bool useGravityModelIn,
		uint32_t recalcThresholdIn,
		float yearIn,
		math::vec3d positionIn) :
		useMagModel(useMagModelIn),
		useGravityModel(useGravityModelIn),
		recalcThreshold(recalcThresholdIn),
		year(yearIn),
		position(positionIn)
	{ }

};

/// \brief Structure representing the GPS Solution - LLA register.
struct GpsSolutionLlaRegister
{
	double time; ///< The time field.
	uint16_t week; ///< The week field.
	protocol::uart::GpsFix gpsFix; ///< The gpsFix field.
	uint8_t numSats; ///< The numSats field.
	math::vec3d lla; ///< The lla field.
	math::vec3f nedVel; ///< The nedVel field.
	math::vec3f nedAcc; ///< The nedAcc field.
	float speedAcc; ///< The speedAcc field.
	float timeAcc; ///< The timeAcc field.

	GpsSolutionLlaRegister() { }

	/// \brief Creates an initializes a new GpsSolutionLlaRegister structure.
	///
	/// \param[in] timeIn Value to initialize the time field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] gpsFixIn Value to initialize the gpsFix field with.
	/// \param[in] numSatsIn Value to initialize the numSats field with.
	/// \param[in] llaIn Value to initialize the lla field with.
	/// \param[in] nedVelIn Value to initialize the nedVel field with.
	/// \param[in] nedAccIn Value to initialize the nedAcc field with.
	/// \param[in] speedAccIn Value to initialize the speedAcc field with.
	/// \param[in] timeAccIn Value to initialize the timeAcc field with.
	GpsSolutionLlaRegister(
		double timeIn,
		uint16_t weekIn,
		protocol::uart::GpsFix gpsFixIn,
		uint8_t numSatsIn,
		math::vec3d llaIn,
		math::vec3f nedVelIn,
		math::vec3f nedAccIn,
		float speedAccIn,
		float timeAccIn) :
		time(timeIn),
		week(weekIn),
		gpsFix(gpsFixIn),
		numSats(numSatsIn),
		lla(llaIn),
		nedVel(nedVelIn),
		nedAcc(nedAccIn),
		speedAcc(speedAccIn),
		timeAcc(timeAccIn)
	{ }

};

/// \brief Structure representing the GPS Solution - ECEF register.
struct GpsSolutionEcefRegister
{
	double tow; ///< The tow field.
	uint16_t week; ///< The week field.
	protocol::uart::GpsFix gpsFix; ///< The gpsFix field.
	uint8_t numSats; ///< The numSats field.
	math::vec3d position; ///< The position field.
	math::vec3f velocity; ///< The velocity field.
	math::vec3f posAcc; ///< The posAcc field.
	float speedAcc; ///< The speedAcc field.
	float timeAcc; ///< The timeAcc field.

	GpsSolutionEcefRegister() { }

	/// \brief Creates an initializes a new GpsSolutionEcefRegister structure.
	///
	/// \param[in] towIn Value to initialize the tow field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] gpsFixIn Value to initialize the gpsFix field with.
	/// \param[in] numSatsIn Value to initialize the numSats field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] posAccIn Value to initialize the posAcc field with.
	/// \param[in] speedAccIn Value to initialize the speedAcc field with.
	/// \param[in] timeAccIn Value to initialize the timeAcc field with.
	GpsSolutionEcefRegister(
		double towIn,
		uint16_t weekIn,
		protocol::uart::GpsFix gpsFixIn,
		uint8_t numSatsIn,
		math::vec3d positionIn,
		math::vec3f velocityIn,
		math::vec3f posAccIn,
		float speedAccIn,
		float timeAccIn) :
		tow(towIn),
		week(weekIn),
		gpsFix(gpsFixIn),
		numSats(numSatsIn),
		position(positionIn),
		velocity(velocityIn),
		posAcc(posAccIn),
		speedAcc(speedAccIn),
		timeAcc(timeAccIn)
	{ }

};

/// \brief Structure representing the GPS Configuration register.
struct GpsConfigurationRegister
{
	protocol::uart::GpsMode mode; ///< The mode field.
	protocol::uart::PpsSource ppsSource; ///< The ppsSource field.

	GpsConfigurationRegister() { }

	/// \brief Creates an initializes a new GpsConfigurationRegister structure.
	///
	/// \param[in] modeIn Value to initialize the mode field with.
	/// \param[in] ppsSourceIn Value to initialize the ppsSource field with.
	GpsConfigurationRegister(
		protocol::uart::GpsMode modeIn,
		protocol::uart::PpsSource ppsSourceIn) :
		mode(modeIn),
		ppsSource(ppsSourceIn)
	{ }

};

/// \brief Structure representing the GPS Compass Baseline register.
struct GpsCompassBaselineRegister
{
	math::vec3f position; ///< The position field.
	math::vec3f uncertainty; ///< The uncertainty field.

	GpsCompassBaselineRegister() { }

	/// \brief Creates an initializes a new GpsCompassBaselineRegister structure.
	///
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] uncertaintyIn Value to initialize the uncertainty field with.
	GpsCompassBaselineRegister(
		math::vec3f positionIn,
		math::vec3f uncertaintyIn) :
		position(positionIn),
		uncertainty(uncertaintyIn)
	{ }

};

/// \brief Structure representing the GPS Compass Estimated Baseline register.
struct GpsCompassEstimatedBaselineRegister
{
	bool estBaselineUsed; ///< The estBaselineUsed field.
	uint16_t numMeas; ///< The numMeas field.
	math::vec3f position; ///< The position field.
	math::vec3f uncertainty; ///< The uncertainty field.

	GpsCompassEstimatedBaselineRegister() { }

	/// \brief Creates an initializes a new GpsCompassEstimatedBaselineRegister structure.
	///
	/// \param[in] estBaselineUsedIn Value to initialize the estBaselineUsed field with.
	/// \param[in] numMeasIn Value to initialize the numMeas field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] uncertaintyIn Value to initialize the uncertainty field with.
	GpsCompassEstimatedBaselineRegister(
		bool estBaselineUsedIn,
		uint16_t numMeasIn,
		math::vec3f positionIn,
		math::vec3f uncertaintyIn) :
		estBaselineUsed(estBaselineUsedIn),
		numMeas(numMeasIn),
		position(positionIn),
		uncertainty(uncertaintyIn)
	{ }

};

/// \brief Structure representing the INS Solution - LLA register.
struct InsSolutionLlaRegister
{
	double time; ///< The time field.
	uint16_t week; ///< The week field.
	uint16_t status; ///< The status field.
	math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	math::vec3d position; ///< The position field.
	math::vec3f nedVel; ///< The nedVel field.
	float attUncertainty; ///< The attUncertainty field.
	float posUncertainty; ///< The posUncertainty field.
	float velUncertainty; ///< The velUncertainty field.

	InsSolutionLlaRegister() { }

	/// \brief Creates an initializes a new InsSolutionLlaRegister structure.
	///
	/// \param[in] timeIn Value to initialize the time field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] statusIn Value to initialize the status field with.
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] nedVelIn Value to initialize the nedVel field with.
	/// \param[in] attUncertaintyIn Value to initialize the attUncertainty field with.
	/// \param[in] posUncertaintyIn Value to initialize the posUncertainty field with.
	/// \param[in] velUncertaintyIn Value to initialize the velUncertainty field with.
	InsSolutionLlaRegister(
		double timeIn,
		uint16_t weekIn,
		uint16_t statusIn,
		math::vec3f yawPitchRollIn,
		math::vec3d positionIn,
		math::vec3f nedVelIn,
		float attUncertaintyIn,
		float posUncertaintyIn,
		float velUncertaintyIn) :
		time(timeIn),
		week(weekIn),
		status(statusIn),
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		nedVel(nedVelIn),
		attUncertainty(attUncertaintyIn),
		posUncertainty(posUncertaintyIn),
		velUncertainty(velUncertaintyIn)
	{ }

};

/// \brief Structure representing the INS Solution - ECEF register.
struct InsSolutionEcefRegister
{
	double time; ///< The time field.
	uint16_t week; ///< The week field.
	uint16_t status; ///< The status field.
	math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	math::vec3d position; ///< The position field.
	math::vec3f velocity; ///< The velocity field.
	float attUncertainty; ///< The attUncertainty field.
	float posUncertainty; ///< The posUncertainty field.
	float velUncertainty; ///< The velUncertainty field.

	InsSolutionEcefRegister() { }

	/// \brief Creates an initializes a new InsSolutionEcefRegister structure.
	///
	/// \param[in] timeIn Value to initialize the time field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] statusIn Value to initialize the status field with.
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] attUncertaintyIn Value to initialize the attUncertainty field with.
	/// \param[in] posUncertaintyIn Value to initialize the posUncertainty field with.
	/// \param[in] velUncertaintyIn Value to initialize the velUncertainty field with.
	InsSolutionEcefRegister(
		double timeIn,
		uint16_t weekIn,
		uint16_t statusIn,
		math::vec3f yawPitchRollIn,
		math::vec3d positionIn,
		math::vec3f velocityIn,
		float attUncertaintyIn,
		float posUncertaintyIn,
		float velUncertaintyIn) :
		time(timeIn),
		week(weekIn),
		status(statusIn),
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		velocity(velocityIn),
		attUncertainty(attUncertaintyIn),
		posUncertainty(posUncertaintyIn),
		velUncertainty(velUncertaintyIn)
	{ }

};

/// \brief Structure representing the INS State - LLA register.
struct InsStateLlaRegister
{
	math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	math::vec3d position; ///< The position field.
	math::vec3f velocity; ///< The velocity field.
	math::vec3f accel; ///< The accel field.
	math::vec3f angularRate; ///< The angularRate field.

	InsStateLlaRegister() { }

	/// \brief Creates an initializes a new InsStateLlaRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] angularRateIn Value to initialize the angularRate field with.
	InsStateLlaRegister(
		math::vec3f yawPitchRollIn,
		math::vec3d positionIn,
		math::vec3f velocityIn,
		math::vec3f accelIn,
		math::vec3f angularRateIn) :
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		velocity(velocityIn),
		accel(accelIn),
		angularRate(angularRateIn)
	{ }

};

/// \brief Structure representing the INS State - ECEF register.
struct InsStateEcefRegister
{
	math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	math::vec3d position; ///< The position field.
	math::vec3f velocity; ///< The velocity field.
	math::vec3f accel; ///< The accel field.
	math::vec3f angularRate; ///< The angularRate field.

	InsStateEcefRegister() { }

	/// \brief Creates an initializes a new InsStateEcefRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] angularRateIn Value to initialize the angularRate field with.
	InsStateEcefRegister(
		math::vec3f yawPitchRollIn,
		math::vec3d positionIn,
		math::vec3f velocityIn,
		math::vec3f accelIn,
		math::vec3f angularRateIn) :
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		velocity(velocityIn),
		accel(accelIn),
		angularRate(angularRateIn)
	{ }

};

/// \brief Structure representing the INS Basic Configuration register for a VN-200 sensor.
struct InsBasicConfigurationRegisterVn200
{
	protocol::uart::Scenario scenario; ///< The scenario field.
	bool ahrsAiding; ///< The ahrsAiding field.

	InsBasicConfigurationRegisterVn200() { }

	/// \brief Creates an initializes a new InsBasicConfigurationRegisterVn200 structure.
	///
	/// \param[in] scenarioIn Value to initialize the scenario field with.
	/// \param[in] ahrsAidingIn Value to initialize the ahrsAiding field with.
	InsBasicConfigurationRegisterVn200(
		protocol::uart::Scenario scenarioIn,
		bool ahrsAidingIn) :
		scenario(scenarioIn),
		ahrsAiding(ahrsAidingIn)
	{ }

};

/// \brief Structure representing the INS Basic Configuration register for a VN-300 sensor.
struct InsBasicConfigurationRegisterVn300
{
	protocol::uart::Scenario scenario; ///< The scenario field.
	bool ahrsAiding; ///< The ahrsAiding field.
	bool estBaseline; ///< The estBaseline field.

	InsBasicConfigurationRegisterVn300() { }

	/// \brief Creates an initializes a new InsBasicConfigurationRegisterVn300 structure.
	///
	/// \param[in] scenarioIn Value to initialize the scenario field with.
	/// \param[in] ahrsAidingIn Value to initialize the ahrsAiding field with.
	/// \param[in] estBaselineIn Value to initialize the estBaseline field with.
	InsBasicConfigurationRegisterVn300(
		protocol::uart::Scenario scenarioIn,
		bool ahrsAidingIn,
		bool estBaselineIn) :
		scenario(scenarioIn),
		ahrsAiding(ahrsAidingIn),
		estBaseline(estBaselineIn)
	{ }

};

/// \brief Structure representing the Startup Filter Bias Estimate register.
struct StartupFilterBiasEstimateRegister
{
	math::vec3f gyroBias; ///< The gyroBias field.
	math::vec3f accelBias; ///< The accelBias field.
	float pressureBias; ///< The pressureBias field.

	StartupFilterBiasEstimateRegister() { }

	/// \brief Creates an initializes a new StartupFilterBiasEstimateRegister structure.
	///
	/// \param[in] gyroBiasIn Value to initialize the gyroBias field with.
	/// \param[in] accelBiasIn Value to initialize the accelBias field with.
	/// \param[in] pressureBiasIn Value to initialize the pressureBias field with.
	StartupFilterBiasEstimateRegister(
		math::vec3f gyroBiasIn,
		math::vec3f accelBiasIn,
		float pressureBiasIn) :
		gyroBias(gyroBiasIn),
		accelBias(accelBiasIn),
		pressureBias(pressureBiasIn)
	{ }

};

/// \}

/// \brief Helpful class for working with VectorNav sensors.
class VnSensor : private util::NoCopy
{

public:

	enum Family
	{
		VnSensor_Family_Unknown,	///< Unknown device family.
		VnSensor_Family_Vn100,		///< A device of the VectorNav VN-100 sensor family.
		VnSensor_Family_Vn200,		///< A device of the VectorNav VN-200 sensor family.
		VnSensor_Family_Vn300		///< A device of the VectorNav VN-300 sensor family.
	};

	/// \brief Defines a callback handler that can received notification when
	/// the VnSensor receives raw data from its port.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerRawDataReceivedHandler.
	/// \param[in] rawData Pointer to the raw data.
	/// \param[in] length The number of bytes of raw data.
	/// \param[in] runningIndex The running index of the received data.
	typedef void(*RawDataReceivedHandler)(void* userData, const char* rawData, size_t length, size_t runningIndex);

	/// \brief Defines the signature for a method that can receive
	/// notifications of new possible packets found.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerPossiblePacketFoundHandler.
	/// \param[in] possiblePacket The possible packet that was found.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	typedef void(*PossiblePacketFoundHandler)(void* userData, protocol::uart::Packet& possiblePacket, size_t packetStartRunningIndex);

	/// \brief Defines the signature for a method that can receive
	/// notifications of when a new asynchronous data packet (ASCII or BINARY)
	/// is received.
	///
	/// This packet will have already had and pertinent error checking
	/// performed and determined to be an asynchronous packet.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerAsyncPacketReceivedHandler.
	/// \param[in] asyncPacket The asynchronous packet received.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	typedef void(*AsyncPacketReceivedHandler)(void* userData, protocol::uart::Packet& asyncPacket, size_t packetStartRunningIndex);

	/// \brief Defines the signature for a method that can receive
	/// notifications when an error message is received.
	///
	/// This packet will have already had and pertinent error checking
	/// performed and determined to be an asynchronous packet.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerErrorPacketReceivedHandler.
	/// \param[in] errorPacket The error packet received.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	typedef void(*ErrorPacketReceivedHandler)(void* userData, protocol::uart::Packet& errorPacket, size_t packetStartRunningIndex);

	/// \brief The list of baudrates supported by VectorNav sensors.
	static std::vector<uint32_t> supportedBaudrates();

	VnSensor();

	~VnSensor();

	/// \defgroup vnSensorProperties VnSensor Properties
	/// \brief This group of methods interface with the VnSensor properties.
	///
	/// \{

	/// \brief Gets the current error detection mode used for commands sent to
	/// the sensor. Default is protocol::uart::ErrorDetectionMode::CHECKSUM.
	///
	/// \return The error detection mode used for packets sent to the sensor.
	protocol::uart::ErrorDetectionMode sendErrorDetectionMode();

	/// \brief Sets the error detection mode used by the class for commands
	/// sent to the sensor.
	///
	/// \param[in] mode The new error detection mode for packets sent to the
	/// sensor.
	void setSendErrorDetectionMode(protocol::uart::ErrorDetectionMode mode);

	/// \brief Indicates if the VnSensor is connected.
	///
	/// \return <c>true</c> if the VnSensor is connected; otherwise <c>false</c>.
	bool isConnected();

	/// \brief Gets the amount of time in milliseconds to wait for a response
	/// during read/writes with the sensor.
	///
	/// \return The response timeout in milliseconds.
	uint16_t responseTimeoutMs();

	/// \brief Sets the amount of time in milliseconds to wait for a response
	/// during read/writes with the sensor.
	///
	/// \param[in] timeout The number of milliseconds for response timeouts.
	void setResponseTimeoutMs(uint16_t timeout);

	/// \brief The delay in milliseconds between retransmitting commands.
	///
	/// \return The retransmit delay in milliseconds.
	uint16_t retransmitDelayMs();

	/// \brief Sets the delay in milliseconds between command retransmits.
	///
	/// \param[in] delay The retransmit delay in milliseconds.
	void setRetransmitDelayMs(uint16_t delay);

	/// \}

	/// \brief Checks if we are able to send and receive communication with a sensor.
	///
	/// \return <c>true</c> if we can communicate with the sensor; otherwise <c>false</c>.
	bool verifySensorConnectivity();

	/// \brief Connects to a VectorNav sensor.
	///
	/// \param[in] portName The name of the serial port to connect to.
	/// \param[in] baudrate The baudrate to connect at.
	void connect(const std::string &portName, uint32_t baudrate);

	/// \brief Allows connecting to a VectorNav sensor over a
	///     \ref vn::common::ISimplePort.
	///
	/// The caller is responsible for properly destroying the
	/// \ref vn::common::ISimplePort object when this method is used. Also, if
	/// the provided \ref vn::common::ISimplePort is already open, then when
	/// the method \ref disconnect is called, \ref VnSensor will not attempt to
	/// close the port. However, if the \ref vn::common::ISimplePort is not
	/// open, then any subsequent calls to \ref disconnect will close the port.
	///
	/// \param[in] simplePort An \ref vn::common::ISimplePort. May be either
	///     currently open or closed.
	void connect(common::ISimplePort* simplePort);

	/// \brief Disconnects from the VectorNav sensor.
	///
	/// \exception invalid_operation Thrown if the VnSensor is not
	///     connected.
	void disconnect();

	/// \brief Sends the provided command and returns the response from the sensor.
	///
	/// If the command does not have an asterisk '*', then a checksum will be performed
	/// and appended based on the current error detection mode. Also, if the line-ending
	/// \\r\\n is not present, these will be added also.
	///
	/// \param[in] toSend The command to send to the sensor.
	/// \return The response received from the sensor.
	std::string transaction(std::string toSend);

	/// \brief Writes a raw data string to the sensor, normally appending an
	/// appropriate error detection checksum.
	///
	/// No checksum is necessary as any missing checksum will be provided. For
	/// example, the following toSend data will be correctly received by the
	/// sensor.
	///  - <c>$VNRRG,1*42\\r\\n</c>
	///  - <c>$VNRRG,1*42</c>
	///  - <c>$VNRRG,1*</c>
	///  - <c>$VNRRG,1</c>
	///  - <c>VNRRG,1</c>
	/// 
	/// If waitForReply is <c>true</c>, then the method will wait for a
	/// response and return the received response. Otherwise, if <c>false</c>,
	/// the method will send the data and return immediately with an empty
	/// string.
	///
	/// \param[in] toSend The data to send. The method will automatically
	///     append a checksum/CRC if none is provided.
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response and return the received response.
	/// \param[in] errorDetectionMode Indicates the error detection mode to
	///     append to any packets to send.
	/// \return The received response if waitForReply is <c>true</c>; otherwise
	///     this will be an empty string.
	std::string send(
		std::string toSend,
		bool waitForReply = true,
		protocol::uart::ErrorDetectionMode errorDetectionMode = protocol::uart::ERRORDETECTIONMODE_CHECKSUM);

	/// \brief Issues a Write Settings command to the VectorNav Sensor.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void writeSettings(bool waitForReply = true);

	/// \brief Issues a Restore Factory Settings command to the VectorNav
	/// sensor.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void restoreFactorySettings(bool waitForReply = true);

	/// \brief Issues a Reset command to the VectorNav sensor.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void reset(bool waitForReply = true);

	/// \brief This method will connect to the specified serial port, query the
	/// attached sensor, and determine the type of device.
	///
	/// \param[in] portName The COM port name to connect to.
	/// \param[in] baudrate The baudrate to connect at.
	//static void determineDeviceFamily(std::string portName, uint32_t baudrate);

	/// \brief This method will query the attached sensor and determine the
	/// device family it belongs to.
	///
	/// \return The determined device family.
	Family determineDeviceFamily();

	/// \brief This method determines which device family a VectorNav sensor
	/// belongs to based on the provided model number.
	///
	/// \return The determined device family.
	static Family determineDeviceFamily(std::string modelNumber);

	/// \defgroup vnSensorEvents VnSensor Events
	/// \brief This group of methods allow registering/unregistering for events
	/// of the VnSensor.
	///
	/// \{

	/// \brief Registers a callback method for notification when raw data is
	/// received.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterRawDataReceivedHandler();

	/// \brief Registers a callback method for notification when a new possible
	/// packet is found.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterPossiblePacketFoundHandler();

	/// \brief Registers a callback method for notification when a new
	/// asynchronous data packet is received.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterAsyncPacketReceivedHandler();

	/// \brief Registers a callback method for notification when an error
	/// packet is received.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterErrorPacketReceivedHandler();

	/// \}

	/// \defgroup registerAccessMethods Register Access Methods
	/// \brief This group of methods provide access to read and write to the
	/// sensor's registers.
	///
	/// \{

	/// \brief Reads the Binary Output 1 register.
	///
	/// \return The register's values.
	BinaryOutputRegister readBinaryOutput1();

	/// \brief Writes to the Binary Output 1 register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeBinaryOutput1(BinaryOutputRegister &fields, bool waitForReply = true);

	/// \brief Reads the Binary Output 2 register.
	///
	/// \return The register's values.
	BinaryOutputRegister readBinaryOutput2();

	/// \brief Writes to the Binary Output 2 register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeBinaryOutput2(BinaryOutputRegister &fields, bool waitForReply = true);

	/// \brief Reads the Binary Output 3 register.
	///
	/// \return The register's values.
	BinaryOutputRegister readBinaryOutput3();

	/// \brief Writes to the Binary Output 3 register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeBinaryOutput3(BinaryOutputRegister &fields, bool waitForReply = true);

	/// \brief Reads the User Tag register.
	///
	/// \return The register's values.
	std::string readUserTag();

	/// \brief Writes to the User Tag register.
	///
	/// \param[in] tag The register's Tag field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeUserTag(const std::string &tag, bool waitForReply = true);

	/// \brief Reads the Model Number register.
	///
	/// \return The register's values.
	std::string readModelNumber();

	/// \brief Reads the Hardware Revision register.
	///
	/// \return The register's values.
	uint32_t readHardwareRevision();

	/// \brief Reads the Serial Number register.
	///
	/// \return The register's values.
	uint32_t readSerialNumber();

	/// \brief Reads the Firmware Version register.
	///
	/// \return The register's values.
	std::string readFirmwareVersion();

	/// \brief Reads the Serial Baud Rate register.
	///
	/// \return The register's values.
	uint32_t readSerialBaudRate();

	/// \brief Writes to the Serial Baud Rate register.
	///
	/// \param[in] baudrate The register's Baud Rate field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSerialBaudRate(const uint32_t &baudrate, bool waitForReply = true);

	/// \brief Reads the Async Data Output Type register.
	///
	/// \return The register's values.
	protocol::uart::AsciiAsync readAsyncDataOutputType();

	/// \brief Writes to the Async Data Output Type register.
	///
	/// \param[in] ador The register's ADOR field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAsyncDataOutputType(protocol::uart::AsciiAsync ador, bool waitForReply = true);

	/// \brief Reads the Async Data Output Frequency register.
	///
	/// \return The register's values.
	uint32_t readAsyncDataOutputFrequency();

	/// \brief Writes to the Async Data Output Frequency register.
	///
	/// \param[in] adof The register's ADOF field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAsyncDataOutputFrequency(const uint32_t &adof, bool waitForReply = true);

	/// \brief Reads the Synchronization Control register.
	///
	/// \return The register's values.
	SynchronizationControlRegister readSynchronizationControl();

	/// \brief Writes to the Synchronization Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationControl(SynchronizationControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Synchronization Control register.
	///
	/// \param[in] syncInMode Value for the SyncInMode field.
	/// \param[in] syncInEdge Value for the SyncInEdge field.
	/// \param[in] syncInSkipFactor Value for the SyncInSkipFactor field.
	/// \param[in] syncOutMode Value for the SyncOutMode field.
	/// \param[in] syncOutPolarity Value for the SyncOutPolarity field.
	/// \param[in] syncOutSkipFactor Value for the SyncOutSkipFactor field.
	/// \param[in] syncOutPulseWidth Value for the SyncOutPulseWidth field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationControl(
		protocol::uart::SyncInMode syncInMode,
		protocol::uart::SyncInEdge syncInEdge,
		const uint16_t &syncInSkipFactor,
		protocol::uart::SyncOutMode syncOutMode,
		protocol::uart::SyncOutPolarity syncOutPolarity,
		const uint16_t &syncOutSkipFactor,
		const uint32_t &syncOutPulseWidth,
		bool waitForReply = true);

	/// \brief Reads the Communication Protocol Control register.
	///
	/// \return The register's values.
	CommunicationProtocolControlRegister readCommunicationProtocolControl();

	/// \brief Writes to the Communication Protocol Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeCommunicationProtocolControl(CommunicationProtocolControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Communication Protocol Control register.
	///
	/// \param[in] serialCount Value for the SerialCount field.
	/// \param[in] serialStatus Value for the SerialStatus field.
	/// \param[in] spiCount Value for the SPICount field.
	/// \param[in] spiStatus Value for the SPIStatus field.
	/// \param[in] serialChecksum Value for the SerialChecksum field.
	/// \param[in] spiChecksum Value for the SPIChecksum field.
	/// \param[in] errorMode Value for the ErrorMode field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeCommunicationProtocolControl(
		protocol::uart::CountMode serialCount,
		protocol::uart::StatusMode serialStatus,
		protocol::uart::CountMode spiCount,
		protocol::uart::StatusMode spiStatus,
		protocol::uart::ChecksumMode serialChecksum,
		protocol::uart::ChecksumMode spiChecksum,
		protocol::uart::ErrorMode errorMode,
		bool waitForReply = true);

	/// \brief Reads the Synchronization Status register.
	///
	/// \return The register's values.
	SynchronizationStatusRegister readSynchronizationStatus();

	/// \brief Writes to the Synchronization Status register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationStatus(SynchronizationStatusRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Synchronization Status register.
	///
	/// \param[in] syncInCount Value for the SyncInCount field.
	/// \param[in] syncInTime Value for the SyncInTime field.
	/// \param[in] syncOutCount Value for the SyncOutCount field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationStatus(
		const uint32_t &syncInCount,
		const uint32_t &syncInTime,
		const uint32_t &syncOutCount,
		bool waitForReply = true);

	/// \brief Reads the IMU Measurements register.
	///
	/// \return The register's values.
	ImuMeasurementsRegister readImuMeasurements();

	/// \brief Reads the Delta Theta and Delta Velocity register.
	///
	/// \return The register's values.
	DeltaThetaAndDeltaVelocityRegister readDeltaThetaAndDeltaVelocity();

	/// \brief Reads the Magnetometer Compensation register.
	///
	/// \return The register's values.
	MagnetometerCompensationRegister readMagnetometerCompensation();

	/// \brief Writes to the Magnetometer Compensation register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCompensation(MagnetometerCompensationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Magnetometer Compensation register.
	///
	/// \param[in] c Value for the C field.
	/// \param[in] b Value for the B field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCompensation(
		const math::mat3f &c,
		const math::vec3f &b,
		bool waitForReply = true);

	/// \brief Reads the Acceleration Compensation register.
	///
	/// \return The register's values.
	AccelerationCompensationRegister readAccelerationCompensation();

	/// \brief Writes to the Acceleration Compensation register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAccelerationCompensation(AccelerationCompensationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Acceleration Compensation register.
	///
	/// \param[in] c Value for the C field.
	/// \param[in] b Value for the B field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAccelerationCompensation(
		const math::mat3f &c,
		const math::vec3f &b,
		bool waitForReply = true);

	/// \brief Reads the Gyro Compensation register.
	///
	/// \return The register's values.
	GyroCompensationRegister readGyroCompensation();

	/// \brief Writes to the Gyro Compensation register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGyroCompensation(GyroCompensationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Gyro Compensation register.
	///
	/// \param[in] c Value for the C field.
	/// \param[in] b Value for the B field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGyroCompensation(
		const math::mat3f &c,
		const math::vec3f &b,
		bool waitForReply = true);

	/// \brief Reads the Reference Frame Rotation register.
	///
	/// \return The register's values.
	math::mat3f readReferenceFrameRotation();

	/// \brief Writes to the Reference Frame Rotation register.
	///
	/// \param[in] c The register's C field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeReferenceFrameRotation(const math::mat3f &c, bool waitForReply = true);

	/// \brief Reads the IMU Filtering Configuration register.
	///
	/// \return The register's values.
	ImuFilteringConfigurationRegister readImuFilteringConfiguration();

	/// \brief Writes to the IMU Filtering Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeImuFilteringConfiguration(ImuFilteringConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the IMU Filtering Configuration register.
	///
	/// \param[in] magWindowSize Value for the MagWindowSize field.
	/// \param[in] accelWindowSize Value for the AccelWindowSize field.
	/// \param[in] gyroWindowSize Value for the GyroWindowSize field.
	/// \param[in] tempWindowSize Value for the TempWindowSize field.
	/// \param[in] presWindowSize Value for the PresWindowSize field.
	/// \param[in] magFilterMode Value for the MagFilterMode field.
	/// \param[in] accelFilterMode Value for the AccelFilterMode field.
	/// \param[in] gyroFilterMode Value for the GyroFilterMode field.
	/// \param[in] tempFilterMode Value for the TempFilterMode field.
	/// \param[in] presFilterMode Value for the PresFilterMode field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeImuFilteringConfiguration(
		const uint16_t &magWindowSize,
		const uint16_t &accelWindowSize,
		const uint16_t &gyroWindowSize,
		const uint16_t &tempWindowSize,
		const uint16_t &presWindowSize,
		protocol::uart::FilterMode magFilterMode,
		protocol::uart::FilterMode accelFilterMode,
		protocol::uart::FilterMode gyroFilterMode,
		protocol::uart::FilterMode tempFilterMode,
		protocol::uart::FilterMode presFilterMode,
		bool waitForReply = true);

	/// \brief Reads the Delta Theta and Delta Velocity Configuration register.
	///
	/// \return The register's values.
	DeltaThetaAndDeltaVelocityConfigurationRegister readDeltaThetaAndDeltaVelocityConfiguration();

	/// \brief Writes to the Delta Theta and Delta Velocity Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeDeltaThetaAndDeltaVelocityConfiguration(DeltaThetaAndDeltaVelocityConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Delta Theta and Delta Velocity Configuration register.
	///
	/// \param[in] integrationFrame Value for the IntegrationFrame field.
	/// \param[in] gyroCompensation Value for the GyroCompensation field.
	/// \param[in] accelCompensation Value for the AccelCompensation field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeDeltaThetaAndDeltaVelocityConfiguration(
		protocol::uart::IntegrationFrame integrationFrame,
		protocol::uart::CompensationMode gyroCompensation,
		protocol::uart::CompensationMode accelCompensation,
		bool waitForReply = true);

	/// \brief Reads the Yaw Pitch Roll register.
	///
	/// \return The register's values.
	math::vec3f readYawPitchRoll();

	/// \brief Reads the Attitude Quaternion register.
	///
	/// \return The register's values.
	math::vec4f readAttitudeQuaternion();

	/// \brief Reads the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	YawPitchRollMagneticAccelerationAndAngularRatesRegister readYawPitchRollMagneticAccelerationAndAngularRates();

	/// \brief Reads the Quaternion, Magnetic, Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	QuaternionMagneticAccelerationAndAngularRatesRegister readQuaternionMagneticAccelerationAndAngularRates();

	/// \brief Reads the Magnetic Measurements register.
	///
	/// \return The register's values.
	math::vec3f readMagneticMeasurements();

	/// \brief Reads the Acceleration Measurements register.
	///
	/// \return The register's values.
	math::vec3f readAccelerationMeasurements();

	/// \brief Reads the Angular Rate Measurements register.
	///
	/// \return The register's values.
	math::vec3f readAngularRateMeasurements();

	/// \brief Reads the Magnetic, Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	MagneticAccelerationAndAngularRatesRegister readMagneticAccelerationAndAngularRates();

	/// \brief Reads the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister readYawPitchRollTrueBodyAccelerationAndAngularRates();

	/// \brief Reads the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister readYawPitchRollTrueInertialAccelerationAndAngularRates();

	/// \brief Reads the VPE Basic Control register.
	///
	/// \return The register's values.
	VpeBasicControlRegister readVpeBasicControl();

	/// \brief Writes to the VPE Basic Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeBasicControl(VpeBasicControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Basic Control register.
	///
	/// \param[in] enable Value for the Enable field.
	/// \param[in] headingMode Value for the HeadingMode field.
	/// \param[in] filteringMode Value for the FilteringMode field.
	/// \param[in] tuningMode Value for the TuningMode field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeBasicControl(
		protocol::uart::VpeEnable enable,
		protocol::uart::HeadingMode headingMode,
		protocol::uart::VpeMode filteringMode,
		protocol::uart::VpeMode tuningMode,
		bool waitForReply = true);

	/// \brief Reads the VPE Magnetometer Basic Tuning register.
	///
	/// \return The register's values.
	VpeMagnetometerBasicTuningRegister readVpeMagnetometerBasicTuning();

	/// \brief Writes to the VPE Magnetometer Basic Tuning register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeMagnetometerBasicTuning(VpeMagnetometerBasicTuningRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Magnetometer Basic Tuning register.
	///
	/// \param[in] baseTuning Value for the BaseTuning field.
	/// \param[in] adaptiveTuning Value for the AdaptiveTuning field.
	/// \param[in] adaptiveFiltering Value for the AdaptiveFiltering field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeMagnetometerBasicTuning(
		const math::vec3f &baseTuning,
		const math::vec3f &adaptiveTuning,
		const math::vec3f &adaptiveFiltering,
		bool waitForReply = true);

	/// \brief Reads the VPE Accelerometer Basic Tuning register.
	///
	/// \return The register's values.
	VpeAccelerometerBasicTuningRegister readVpeAccelerometerBasicTuning();

	/// \brief Writes to the VPE Accelerometer Basic Tuning register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeAccelerometerBasicTuning(VpeAccelerometerBasicTuningRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Accelerometer Basic Tuning register.
	///
	/// \param[in] baseTuning Value for the BaseTuning field.
	/// \param[in] adaptiveTuning Value for the AdaptiveTuning field.
	/// \param[in] adaptiveFiltering Value for the AdaptiveFiltering field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeAccelerometerBasicTuning(
		const math::vec3f &baseTuning,
		const math::vec3f &adaptiveTuning,
		const math::vec3f &adaptiveFiltering,
		bool waitForReply = true);

	/// \brief Reads the Magnetometer Calibration Control register.
	///
	/// \return The register's values.
	MagnetometerCalibrationControlRegister readMagnetometerCalibrationControl();

	/// \brief Writes to the Magnetometer Calibration Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCalibrationControl(MagnetometerCalibrationControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Magnetometer Calibration Control register.
	///
	/// \param[in] hsiMode Value for the HSIMode field.
	/// \param[in] hsiOutput Value for the HSIOutput field.
	/// \param[in] convergeRate Value for the ConvergeRate field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCalibrationControl(
		protocol::uart::HsiMode hsiMode,
		protocol::uart::HsiOutput hsiOutput,
		const uint8_t &convergeRate,
		bool waitForReply = true);

	/// \brief Reads the Calculated Magnetometer Calibration register.
	///
	/// \return The register's values.
	CalculatedMagnetometerCalibrationRegister readCalculatedMagnetometerCalibration();

	/// \brief Reads the Velocity Compensation Control register.
	///
	/// \return The register's values.
	VelocityCompensationControlRegister readVelocityCompensationControl();

	/// \brief Writes to the Velocity Compensation Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVelocityCompensationControl(VelocityCompensationControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Velocity Compensation Control register.
	///
	/// \param[in] mode Value for the Mode field.
	/// \param[in] velocityTuning Value for the VelocityTuning field.
	/// \param[in] rateTuning Value for the RateTuning field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVelocityCompensationControl(
		const uint8_t &mode,
		const float &velocityTuning,
		const float &rateTuning,
		bool waitForReply = true);

	/// \brief Reads the Velocity Compensation Status register.
	///
	/// \return The register's values.
	VelocityCompensationStatusRegister readVelocityCompensationStatus();

	/// \brief Reads the Velocity Compensation Measurement register.
	///
	/// \return The register's values.
	math::vec3f readVelocityCompensationMeasurement();

	/// \brief Writes to the Velocity Compensation Measurement register.
	///
	/// \param[in] velocity The register's Velocity field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVelocityCompensationMeasurement(const math::vec3f &velocity, bool waitForReply = true);

	/// \brief Reads the Magnetic and Gravity Reference Vectors register.
	///
	/// \return The register's values.
	MagneticAndGravityReferenceVectorsRegister readMagneticAndGravityReferenceVectors();

	/// \brief Writes to the Magnetic and Gravity Reference Vectors register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagneticAndGravityReferenceVectors(MagneticAndGravityReferenceVectorsRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Magnetic and Gravity Reference Vectors register.
	///
	/// \param[in] magRef Value for the MagRef field.
	/// \param[in] accRef Value for the AccRef field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagneticAndGravityReferenceVectors(
		const math::vec3f &magRef,
		const math::vec3f &accRef,
		bool waitForReply = true);

	/// \brief Reads the Reference Vector Configuration register.
	///
	/// \return The register's values.
	ReferenceVectorConfigurationRegister readReferenceVectorConfiguration();

	/// \brief Writes to the Reference Vector Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeReferenceVectorConfiguration(ReferenceVectorConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Reference Vector Configuration register.
	///
	/// \param[in] useMagModel Value for the UseMagModel field.
	/// \param[in] useGravityModel Value for the UseGravityModel field.
	/// \param[in] recalcThreshold Value for the RecalcThreshold field.
	/// \param[in] year Value for the Year field.
	/// \param[in] position Value for the Position field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeReferenceVectorConfiguration(
		const uint8_t &useMagModel,
		const uint8_t &useGravityModel,
		const uint32_t &recalcThreshold,
		const float &year,
		const math::vec3d &position,
		bool waitForReply = true);

	/// \brief Reads the GPS Solution - LLA register.
	///
	/// \return The register's values.
	GpsSolutionLlaRegister readGpsSolutionLla();

	/// \brief Reads the GPS Solution - ECEF register.
	///
	/// \return The register's values.
	GpsSolutionEcefRegister readGpsSolutionEcef();

	/// \brief Reads the GPS Configuration register.
	///
	/// \return The register's values.
	GpsConfigurationRegister readGpsConfiguration();

	/// \brief Writes to the GPS Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsConfiguration(GpsConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the GPS Configuration register.
	///
	/// \param[in] mode Value for the Mode field.
	/// \param[in] ppsSource Value for the PpsSource field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsConfiguration(
		protocol::uart::GpsMode mode,
		protocol::uart::PpsSource ppsSource,
		bool waitForReply = true);

	/// \brief Reads the GPS Antenna Offset register.
	///
	/// \return The register's values.
	math::vec3f readGpsAntennaOffset();

	/// \brief Writes to the GPS Antenna Offset register.
	///
	/// \param[in] position The register's Position field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsAntennaOffset(const math::vec3f &position, bool waitForReply = true);

	/// \brief Reads the GPS Compass Baseline register.
	///
	/// \return The register's values.
	GpsCompassBaselineRegister readGpsCompassBaseline();

	/// \brief Writes to the GPS Compass Baseline register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsCompassBaseline(GpsCompassBaselineRegister &fields, bool waitForReply = true);

	/// \brief Writes to the GPS Compass Baseline register.
	///
	/// \param[in] position Value for the Position field.
	/// \param[in] uncertainty Value for the Uncertainty field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsCompassBaseline(
		const math::vec3f &position,
		const math::vec3f &uncertainty,
		bool waitForReply = true);

	/// \brief Reads the GPS Compass Estimated Baseline register.
	///
	/// \return The register's values.
	GpsCompassEstimatedBaselineRegister readGpsCompassEstimatedBaseline();

	/// \brief Reads the INS Solution - LLA register.
	///
	/// \return The register's values.
	InsSolutionLlaRegister readInsSolutionLla();

	/// \brief Reads the INS Solution - ECEF register.
	///
	/// \return The register's values.
	InsSolutionEcefRegister readInsSolutionEcef();

	/// \brief Reads the INS State - LLA register.
	///
	/// \return The register's values.
	InsStateLlaRegister readInsStateLla();

	/// \brief Reads the INS State - ECEF register.
	///
	/// \return The register's values.
	InsStateEcefRegister readInsStateEcef();

	/// \brief Reads the INS Basic Configuration register for a VN-200 sensor.
	///
	/// \return The register's values.
	InsBasicConfigurationRegisterVn200 readInsBasicConfigurationVn200();

	/// \brief Writes to the INS Basic Configuration register for a VN-200 sensor.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfiguration(InsBasicConfigurationRegisterVn200 &fields, bool waitForReply = true);

	/// \brief Writes to the INS Basic Configuration register for a VN-200 sensor.
	///
	/// \param[in] scenario Value for the Scenario field.
	/// \param[in] ahrsAiding Value for the AhrsAiding field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfiguration(
		protocol::uart::Scenario scenario,
		const uint8_t &ahrsAiding,
		bool waitForReply = true);

	/// \brief Reads the INS Basic Configuration register for a VN-300 sensor.
	///
	/// \return The register's values.
	InsBasicConfigurationRegisterVn300 readInsBasicConfigurationVn300();

	/// \brief Writes to the INS Basic Configuration register for a VN-300 sensor.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfiguration(InsBasicConfigurationRegisterVn300 &fields, bool waitForReply = true);

	/// \brief Writes to the INS Basic Configuration register for a VN-300 sensor.
	///
	/// \param[in] scenario Value for the Scenario field.
	/// \param[in] ahrsAiding Value for the AhrsAiding field.
	/// \param[in] estBaseline Value for the EstBaseline field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfiguration(
		protocol::uart::Scenario scenario,
		const uint8_t &ahrsAiding,
		const uint8_t &estBaseline,
		bool waitForReply = true);

	/// \brief Reads the Startup Filter Bias Estimate register.
	///
	/// \return The register's values.
	StartupFilterBiasEstimateRegister readStartupFilterBiasEstimate();

	/// \brief Writes to the Startup Filter Bias Estimate register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeStartupFilterBiasEstimate(StartupFilterBiasEstimateRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Startup Filter Bias Estimate register.
	///
	/// \param[in] gyroBias Value for the GyroBias field.
	/// \param[in] accelBias Value for the AccelBias field.
	/// \param[in] pressureBias Value for the PressureBias field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeStartupFilterBiasEstimate(
		const math::vec3f &gyroBias,
		const math::vec3f &accelBias,
		const float &pressureBias,
		bool waitForReply = true);

	/// \}

	#ifdef PYTHON_WRAPPERS

	#endif

private:
	struct Impl;
	Impl *_pi;

};

}
}

#endif
