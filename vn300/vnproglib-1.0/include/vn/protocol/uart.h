// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#ifndef _VNPROTOCOL_UART_H_
#define _VNPROTOCOL_UART_H_

#if defined(_MSC_VER)
	/* We are using Visual Studio. Suppress warnings about unsafe library functions. */
	#define _CRT_SECURE_NO_WARNINGS
	#define _SCL_SECURE_NO_WARNINGS
#endif

#include "vn/int.h"
#include "vn/math/vector.h"
#include "vn/math/matrix.h"
#include "vn/util/nocopy.h"

namespace vn {
namespace protocol {
namespace uart {

enum ErrorDetectionMode
{
	ERRORDETECTIONMODE_NONE,		///< No error detection is used.
	ERRORDETECTIONMODE_CHECKSUM,	///< 8-bit XOR checksum is used.
	ERRORDETECTIONMODE_CRC			///< 16-bit CRC16-CCITT is used.
};

/// \brief Enumeration of the available asynchronous ASCII message types.
enum AsciiAsync
{
	VNOFF	= 0,		///< Asynchronous output is turned off.
	VNYPR	= 1,		///< Asynchronous output type is Yaw, Pitch, Roll.
	VNQTN	= 2,		///< Asynchronous output type is Quaternion.
	VNQMR	= 8,		///< Asynchronous output type is Quaternion, Magnetic, Acceleration and Angular Rates.
	VNMAG	= 10,		///< Asynchronous output type is Magnetic Measurements.
	VNACC	= 11,		///< Asynchronous output type is Acceleration Measurements.
	VNGYR	= 12,		///< Asynchronous output type is Angular Rate Measurements.
	VNMAR	= 13,		///< Asynchronous output type is Magnetic, Acceleration, and Angular Rate Measurements.
	VNYMR	= 14,		///< Asynchronous output type is Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements.
	VNYBA	= 16,		///< Asynchronous output type is Yaw, Pitch, Roll, Body True Acceleration.
	VNYIA	= 17,		///< Asynchronous output type is Yaw, Pitch, Roll, Inertial True Acceleration.
	VNIMU	= 19,		///< Asynchronous output type is Calibrated Inertial Measurements.
	VNGPS	= 20,		///< Asynchronous output type is GPS LLA.
	VNGPE	= 21,		///< Asynchronous output type is GPS ECEF.
	VNINS	= 22,		///< Asynchronous output type is INS LLA solution.
	VNINE	= 23,		///< Asynchronous output type is INS ECEF solution.
	VNISL	= 28,		///< Asynchronous output type is INS LLA 2 solution.
	VNISE	= 29,		///< Asynchronous output type is INS ECEF 2 solution.
	VNDTV	= 30,		///< Asynchronous output type is Delta Theta and Delta Velocity.
};

/// \brief Async modes for the Binary Output registers.
enum AsyncMode
{
	ASYNCMODE_NONE	= 0,	///< None.
	ASYNCMODE_PORT1	= 1,	///< Serial port 1.
	ASYNCMODE_PORT2	= 2,	///< Serial port 2.
	ASYNCMODE_BOTH	= 3		///< Both serial ports.
};

/// \brief Flags for the binary group 1 'Common' in the binary output registers.
enum CommonGroup
{
	COMMONGROUP_NONE				= 0x0000,	///< None.
	COMMONGROUP_TIMESTARTUP			= 0x0001,	///< TimeStartup.
	COMMONGROUP_TIMEGPS				= 0x0002,	///< TimeGps.
	COMMONGROUP_TIMESYNCIN			= 0x0004,	///< TimeSyncIn.
	COMMONGROUP_YAWPITCHROLL		= 0x0008,	///< YawPitchRoll.
	COMMONGROUP_QUATERNION			= 0x0010,	///< Quaternion.
	COMMONGROUP_ANGULARRATE			= 0x0020,	///< AngularRate.
	COMMONGROUP_POSITION			= 0x0040,	///< Position.
	COMMONGROUP_VELOCITY			= 0x0080,	///< Velocity.
	COMMONGROUP_ACCEL				= 0x0100,	///< Accel.
	COMMONGROUP_IMU					= 0x0200,	///< Imu.
	COMMONGROUP_MAGPRES				= 0x0400,	///< MagPres.
	COMMONGROUP_DELTATHETA			= 0x0800,	///< DeltaTheta.
	COMMONGROUP_INSSTATUS			= 0x1000,	///< InsStatus.
	COMMONGROUP_SYNCINCNT			= 0x2000,	///< SyncInCnt.
	COMMONGROUP_TIMEGPSPPS			= 0x4000	///< TimeGpsPps.
};

/// \brief Flags for the binary group 2 'Time' in the binary output registers.
enum TimeGroup
{
	TIMEGROUP_NONE					= 0x0000,	///< None.
	TIMEGROUP_TIMESTARTUP			= 0x0001,	///< TimeStartup.
	TIMEGROUP_TIMEGPS				= 0x0002,	///< TimeGps.
	TIMEGROUP_GPSTOW				= 0x0004,	///< GpsTow.
	TIMEGROUP_GPSWEEK				= 0x0008,	///< GpsWeek.
	TIMEGROUP_TIMESYNCIN			= 0x0010,	///< TimeSyncIn.
	TIMEGROUP_TIMEGPSPPS			= 0x0020,	///< TimeGpsPps.
	TIMEGROUP_TIMEUTC				= 0x0040,	///< TimeUTC.
	TIMEGROUP_SYNCINCNT				= 0x0080	///< SyncInCnt.
};

/// \brief Flags for the binary group 3 'IMU' in the binary output registers.
enum ImuGroup
{
	IMUGROUP_NONE					= 0x0000,	///< None.
	IMUGROUP_IMUSTATUS				= 0x0001,	///< ImuStatus.
	IMUGROUP_UNCOMPMAG				= 0x0002,	///< UncompMag.
	IMUGROUP_UNCOMPACCEL			= 0x0004,	///< UncompAccel.
	IMUGROUP_UNCOMPGYRO				= 0x0008,	///< UncompGyro.
	IMUGROUP_TEMP					= 0x0010,	///< Temp.
	IMUGROUP_PRES					= 0x0020,	///< Pres.
	IMUGROUP_DELTATHETA				= 0x0040,	///< DeltaTheta.
	IMUGROUP_DELTAVEL				= 0x0080,	///< DeltaVel.
	IMUGROUP_MAG					= 0x0100,	///< Mag.
	IMUGROUP_ACCEL					= 0x0200,	///< Accel.
	IMUGROUP_ANGULARRATE			= 0x0400,	///< AngularRate.
	IMUGROUP_SENSSAT				= 0x0800,	///< SensSat.
};

/// \brief Flags for the binary group 4 'GPS' in the binary output registers.
enum GpsGroup
{
	GPSGROUP_NONE					= 0x0000,	///< None.
	GPSGROUP_UTC					= 0x0001,	///< UTC.
	GPSGROUP_TOW					= 0x0002,	///< Tow.
	GPSGROUP_WEEK					= 0x0004,	///< Week.
	GPSGROUP_NUMSATS				= 0x0008,	///< NumSats.
	GPSGROUP_FIX					= 0x0010,	///< Fix.
	GPSGROUP_POSLLA					= 0x0020,	///< PosLla.
	GPSGROUP_POSECEF				= 0x0040,	///< PosEcef.
	GPSGROUP_VELNED					= 0x0080,	///< VelNed.
	GPSGROUP_VELECEF				= 0x0100,	///< VelEcef.
	GPSGROUP_POSU					= 0x0200,	///< PosU.
	GPSGROUP_VELU					= 0x0400,	///< VelU.
	GPSGROUP_TIMEU					= 0x0800,	///< TimeU.
	GPSGROUP_SVSTAT					= 0x1000	///< SvStat.
};

/// \brief Flags for the binary group 5 'Attitude' in the binary output registers.
enum AttitudeGroup
{
	ATTITUDEGROUP_NONE				= 0x0000,	///< None.
	ATTITUDEGROUP_VPESTATUS			= 0x0001,	///< VpeStatus.
	ATTITUDEGROUP_YAWPITCHROLL		= 0x0002,	///< YawPitchRoll.
	ATTITUDEGROUP_QUATERNION		= 0x0004,	///< Quaternion.
	ATTITUDEGROUP_DCM				= 0x0008,	///< DCM.
	ATTITUDEGROUP_MAGNED			= 0x0010,	///< MagNed.
	ATTITUDEGROUP_ACCELNED			= 0x0020,	///< AccelNed.
	ATTITUDEGROUP_LINEARACCELBODY	= 0x0040,	///< LinearAccelBody.
	ATTITUDEGROUP_LINEARACCELNED	= 0x0080,	///< LinearAccelNed.
	ATTITUDEGROUP_YPRU				= 0x0100	///< YprU.
};

/// \brief Flags for the binary group 6 'INS' in the binary output registers.
enum InsGroup
{
	INSGROUP_NONE					= 0x0000,	///< None.
	INSGROUP_INSSTATUS				= 0x0001,	///< InsStatus.
	INSGROUP_POSLLA					= 0x0002,	///< PosLla.
	INSGROUP_POSECEF				= 0x0004,	///< PosEcef.
	INSGROUP_VELBODY				= 0x0008,	///< VelBody.
	INSGROUP_VELNED					= 0x0010,	///< VelNed.
	INSGROUP_VELECEF				= 0x0020,	///< VelEcef.
	INSGROUP_MAGECEF				= 0x0040,	///< MagEcef.
	INSGROUP_ACCELECEF				= 0x0080,	///< AccelEcef.
	INSGROUP_LINEARACCELECEF		= 0x0100,	///< LinearAccelEcef.
	INSGROUP_POSU					= 0x0200,	///< PosU.
	INSGROUP_VELU					= 0x0400	///< VelU.
};

/// \brief Errors that the VectorNav sensor can report.
enum SensorError
{
	ERR_HARD_FAULT = 1,					///< Hard fault.
	ERR_SERIAL_BUFFER_OVERFLOW = 2,		///< Serial buffer overflow.
	ERR_INVALID_CHECKSUM = 3,			///< Invalid checksum.
	ERR_INVALID_COMMAND = 4,			///< Invalid command.
	ERR_NOT_ENOUGH_PARAMETERS = 5,		///< Not enough parameters.
	ERR_TOO_MANY_PARAMETERS = 6,		///< Too many parameters.
	ERR_INVALID_PARAMETER = 7,			///< Invalid parameter.
	ERR_INVALID_REGISTER = 8,			///< Invalid register.
	ERR_UNAUTHORIZED_ACCESS = 9,		///< Unauthorized access.
	ERR_WATCHDOG_RESET = 10,			///< Watchdog reset.
	ERR_OUTPUT_BUFFER_OVERFLOW = 11,	///< Output buffer overflow.
	ERR_INSUFFICIENT_BAUD_RATE = 12,	///< Insufficient baud rate.
	ERR_ERROR_BUFFER_OVERFLOW = 255		///< Error buffer overflow.
};

/// \brief Different modes for the SyncInMode field of the Synchronization Control register.
enum SyncInMode
{
	/// \brief Count number of trigger events on SYNC_IN pin.
	SYNCINMODE_COUNT = 3,
	/// \brief Start IMU sampling on trigger of SYNC_IN pin.
	SYNCINMODE_IMU = 4,
	/// \brief Output asynchronous message on trigger of SYNC_IN pin.
	SYNCINMODE_ASYNC = 5
};

/// \brief Different modes for the SyncInEdge field of the Synchronization Control register.
enum SyncInEdge
{
	/// \brief Trigger on the rising edge on the SYNC_IN pin.
	SYNCINEDGE_RISING = 0,
	/// \brief Trigger on the falling edge on the SYNC_IN pin.
	SYNCINEDGE_FALLING = 1
};

/// \brief Different modes for the SyncOutMode field of the Synchronization Control register.
enum SyncOutMode
{
	/// \brief None.
	SYNCOUTMODE_NONE = 0,
	/// \brief Trigger at start of IMU sampling.
	SYNCOUTMODE_ITEMSTART = 1,
	/// \brief Trigger when IMU measurements are available.
	SYNCOUTMODE_IMUREADY = 2,
	/// \brief Trigger when attitude measurements are available.
	SYNCOUTMODE_INS = 3,
	/// \brief Trigger on GPS PPS event when a 3D fix is valid.
	SYNCOUTMODE_GPSPPS = 6
};

/// \brief Different modes for the SyncOutPolarity field of the Synchronization Control register.
enum SyncOutPolarity
{
	/// \brief Negative pulse.
	SYNCOUTPOLARITY_NEGATIVE = 0,
	/// \brief Positive pulse.
	SYNCOUTPOLARITY_POSITIVE = 1
};

/// \brief Counting modes for the Communication Protocol Control register.
enum CountMode
{
	/// \brief Off.
	COUNTMODE_NONE = 0,
	/// \brief SyncIn counter.
	COUNTMODE_SYNCINCOUNT = 1,
	/// \brief SyncIn time.
	COUNTMODE_SYNCINTIME = 2,
	/// \brief SyncOut counter.
	COUNTMODE_SYNCOUTCOUNTER = 3,
	/// \brief GPS PPS time.
	COUNTMODE_GPSPPS = 4
};

/// \brief Status modes for the Communication Protocol Control register.
enum StatusMode
{
	/// \brief Off.
	STATUSMODE_OFF = 0,
	/// \brief VPE status.
	STATUSMODE_VPESTATUS = 1,
	/// \brief INS status.
	STATUSMODE_INSSTATUS = 2
};

/// \brief Checksum modes for the Communication Protocol Control register.
enum ChecksumMode
{
	/// \brief Off.
	CHECKSUMMODE_OFF = 0,
	/// \brief 8-bit checksum.
	CHECKSUMMODE_CHECKSUM = 1,
	/// \brief 16-bit CRC.
	CHECKSUMMODE_CRC = 2
};

/// \brief Error modes for the Communication Protocol Control register.
enum ErrorMode
{
	/// \brief Ignore error.
	ERRORMODE_IGNORE = 0,
	/// \brief Send error.
	ERRORMODE_SEND = 1,
	/// \brief Send error and set ADOR register to off.
	ERRORMODE_SENDANDOFF = 2
};

/// \brief Filter modes for the IMU Filtering Configuration register.
enum FilterMode
{
	/// \brief No filtering.
	FILTERMODE_NOFILTERING = 0,
	/// \brief Filtering performed only on raw uncompensated IMU measurements.
	FILTERMODE_ONLYRAW = 1,
	/// \brief Filtering performed only on compensated IMU measurements.
	FILTERMODE_ONLYCOMPENSATED = 2,
	/// \brief Filtering performed on both uncompensated and compensated IMU measurements.
	FILTERMODE_BOTH = 3
};

/// \brief Integration frames for the Delta Theta and Delta Velocity Configuration register.
enum IntegrationFrame
{
	/// \brief Body frame.
	INTEGRATIONFRAME_BODY = 0,
	/// \brief NED frame.
	INTEGRATIONFRAME_NED = 1
};

/// \brief Compensation modes for the Delta Theta and Delta Velocity configuration register.
enum CompensationMode
{
	/// \brief None.
	COMPENSATIONMODE_NONE = 0,
	/// \brief Bias.
	COMPENSATIONMODE_BIAS = 1
};

/// \brief GPS fix modes for the GPS Solution - LLA register.
enum GpsFix
{
	/// \brief No fix.
	GPSFIX_NOFIX = 0,
	/// \brief Time only.
	GPSFIX_TIMEONLY = 1,
	/// \brief 2D.
	GPSFIX_2D = 2,
	/// \brief 3D.
	GPSFIX_3D = 3
};

/// \brief GPS modes for the GPS Configuration register.
enum GpsMode
{
	/// \brief Use onboard GPS.
	GPSMODE_ONBOARDGPS = 0,
	/// \brief Use external GPS.
	GPSMODE_EXTERNALGPS = 1,
	/// \brief Use external VN-200 as GPS.
	GPSMODE_EXTERNALVN200GPS = 2
};

/// \brief GPS PPS mode for the GPS Configuration register.
enum PpsSource
{
	/// \brief GPS PPS signal on GPS_PPS pin and triggered on rising edge.
	PPSSOURCE_GPSPPSRISING = 0,
	/// \brief GPS PPS signal on GPS_PPS pin and triggered on falling edge.
	PPSSOURCE_GPSPPSFALLING = 1,
	/// \brief GPS PPS signal on SyncIn pin and triggered on rising edge.
	PPSSOURCE_SYNCINRISING = 2,
	/// \brief GPS PPS signal on SyncIn pin and triggered on falling edge.
	PPSSOURCE_SYNCINFALLING = 3
};

/// \brief VPE Enable mode for the VPE Basic Control register.
enum VpeEnable
{
	/// \brief Disable
	VPEENABLE_DISABLE = 0,
	/// \brief Enable
	VPEENABLE_ENABLE = 1
};

/// \brief VPE Heading modes used by the VPE Basic Control register.
enum HeadingMode
{
	/// \brief Absolute heading.
	HEADINGMODE_ABSOLUTE = 0,
	/// \brief Relative heading.
	HEADINGMODE_RELATIVE = 1,
	/// \brief Indoor heading.
	HEADINGMODE_INDOOR = 2
};

/// \brief VPE modes for the VPE Basic Control register.
enum VpeMode
{
	/// \brief Off.
	VPEMODE_OFF = 0,
	/// \brief Mode 1.
	VPEMODE_MODE1 = 1
};

/// \brief Different scenario modes for the INS Basic Configuration register.
enum Scenario
{
	/// \brief AHRS.
	SCENARIO_AHRS = 0,
	/// \brief General purpose INS with barometric pressure sensor.
	SCENARIO_INSWITHPRESSURE = 1,
	/// \brief General purpose INS without barometric pressure sensor.
	SCENARIO_INSWITHOUTPRESSURE = 2,
	/// \brief GPS moving baseline for dynamic applications.
	SCENARIO_GPSMOVINGBASELINEDYNAMIC = 3,
	/// \brief GPS moving baseline for static applications.
	SCENARIO_GPSMOVINGBASELINESTATIC = 4
};

/// \brief HSI modes used for the Magnetometer Calibration Control register.
enum HsiMode
{
	/// \brief Real-time hard/soft iron calibration algorithm is turned off.
	HSIMODE_OFF = 0,
	/// \brief Runs the real-time hard/soft iron calibration algorithm.
	HSIMODE_RUN = 1,
	/// \brief Resets the real-time hard/soft iron solution.
	HSIMODE_RESET = 2
};

/// \brief HSI output types for the Magnetometer Calibration Control register.
enum HsiOutput
{
	/// \brief Onboard HSI is not applied to the magnetic measurements.
	HSIOUTPUT_NOONBOARD = 1,
	/// \brief Onboard HSI is applied to the magnetic measurements.
	HSIOUTPUT_USEONBOARD = 3
};

/// \brief Structure representing a UART packet received from the VectorNav
/// sensor.
struct Packet
{
	/// \brief Array containing sizes for the binary group fields.
	static const unsigned char BinaryGroupLengths[sizeof(uint8_t)*8][sizeof(uint16_t)*8];

	/// \brief The different types of UART packets.
	enum Type
	{
		TYPE_BINARY,	///< Binary packet.
		TYPE_ASCII	///< ASCII packet.
	};

	/// \brief The available binary output groups.
	enum BinaryGroupType
	{
		Common = 0x01,		///< Common group.
		Time = 0x02,		///< Time group.
		Imu = 0x04,			///< IMU group.
		Gps = 0x08,			///< GPS group.
		Attitude = 0x10,	///< Attitude group.
		Ins = 0x20			///< INS group.
	};

	Packet();

	/// \brief Creates a new packet based on the provided packet data buffer.
	///
	/// \param[in] type The type of packet.
	/// \param[in] fullPacketData Pointer to a buffer containing the packet
	///     data.
	/// \param[in] length The number of bytes in the packet.
	Packet(Type type, char *fullPacketData, size_t length);

	/// \brief Copy constructor.
	///
	/// \param[in] toCopy The Packet to copy.
	Packet(const Packet &toCopy);

	~Packet();

	/// \brief Assignment operator.
	///
	/// \param[in] from The packet to assign from.
	/// \return Reference to the newly copied packet.
	Packet& operator=(const Packet &from);

	/// \brief Returns the encapsulated data as a string.
	///
	/// \return The packet data.
	std::string datastr();

	/// \brief Returns the type of packet.
	///
	/// \return The type of packet.
	Type type();

	/// \brief Performs data integrity check on the data packet.
	///
	/// This will perform an 8-bit XOR checksum, a CRC16-CCITT CRC, or no
	/// checking depending on the provided data integrity in the packet.
	///
	/// \return <c>true</c> if the packet passed the data integrity checks;
	///     otherwise <c>false</c>.
	bool isValid();

	/// \brief Indicates if the packet is an ASCII error message.
	///
	/// \return <c>true</c> if the packet is an error message; otherwise
	/// <c>false</c>.
	bool isError();

	/// \brief Indicates if the packet is a response to a message sent to the
	/// sensor.
	///
	/// \return <c>true</c> if the packet is a response message; otherwise
	/// <c>false</c>.
	bool isResponse();

	/// \brief Indicates if the packet is an ASCII asynchronous message.
	///
	/// \return <c>true</c> if the packet is an ASCII asynchronous message;
	///     otherwise <c>false</c>.
	bool isAsciiAsync();

	/// \brief Determines the type of ASCII asynchronous message this packet
	/// is.
	///
	/// \return The asynchronous data type of the packet.
	AsciiAsync determineAsciiAsyncType();

	/// \brief Determines if the packet is a compatible match for an expected
	/// binary output message type.
	///
	/// \param[in] commonGroup The Common Group configuration.
	/// \param[in] timeGroup The Time Group configuration.
	/// \param[in] imuGroup The IMU Group configuration.
	/// \param[in] gpsGroup The GPS Group configuration.
	/// \param[in] attitudeGroup The Attitude Group configuration.
	/// \param[in] insGroup The INS Group configuration.
	/// \return <c>true</c> if the packet matches the expected group
	///     configuration; otherwise <c>false</c>.
	bool isCompatible(CommonGroup commonGroup, TimeGroup timeGroup, ImuGroup imuGroup, GpsGroup gpsGroup, AttitudeGroup attitudeGroup, InsGroup insGroup);

	/// \brief Computes the expected number of bytes for a possible binary
	/// packet.
	///
	/// This method requires that the group fields present and the complete
	/// collection of individual group description fields are present.
	///
	/// \param[in] startOfPossibleBinaryPacket The start of the possible binary
	///     packet (i.e. the 0xFA character).
	///
	/// \return The number of bytes expected for this binary packet.
	static size_t computeBinaryPacketLength(const char *startOfPossibleBinaryPacket); 

	/// \brief Computes the number of bytes expected for a binary group field.
	///
	/// \param[in] group The group to calculate the total for.
	/// \param[in] groupField The flags for data types present.
	/// \return The number of bytes for this group.
	static size_t computeNumOfBytesForBinaryGroupPayload(BinaryGroupType group, uint16_t groupField);

	/// \brief Parses an error packet to get the error type.
	///
	/// \return The sensor error.
	SensorError parseError();

	/// \defgroup uartPacketBinaryExtractors UART Binary Data Extractors
	/// \brief This group of methods are useful for extracting data from binary
	/// data packets.
	///
	/// \{

	/// \brief Extracts a uint8_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint8_t extractUint8();

	/// \brief Extracts a int8_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	int8_t extractInt8();

	/// \brief Extracts a uint16_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint16_t extractUint16();

	/// \brief Extracts a uint32_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint32_t extractUint32();

	/// \brief Extracts a uint64_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint64_t extractUint64();

	/// \brief Extracts a float fdata type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	float extractFloat();

	/// \brief Extracts a vec3f data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	math::vec3f extractVec3f();

	/// \brief Extracts a vec3d data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	math::vec3d extractVec3d();

	/// \brief Extracts a vec4f data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	math::vec4f extractVec4f();

	/// \brief Extract a mat3f data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	math::mat3f extractMat3f();

	/// \}

	/// \brief Appends astrick (*), checksum, and newlines to command.
	///
	/// \param[in] errorDetectionMode The error detection type to append to the
	///     command.
	/// \param[in] packet The start of the packet. Should point to the '$'
	///     character.
	/// \param[in] length The current running length of the packet.
	/// \return The final size of the command after appending the endings.
	static size_t finalizeCommand(ErrorDetectionMode errorDetectionMode, char *packet, size_t length);

	/// \defgroup regReadWriteMethods Register Read/Write Generator Methods
	/// \brief This group of methods will create commands that can be used to
	/// read/write the register of a VectorNav sensor.
	///
	/// \{

	/// \brief Generates a command to read the Binary Output 1 register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadBinaryOutput1(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Binary Output 2 register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadBinaryOutput2(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Binary Output 13 register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadBinaryOutput3(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Binary Output 1 register on a VectorNav sensor.
	///
	/// The field outputGroup available on the sensor's register is not
	/// necessary as this method will compute the necessary value from the
	/// provided data fields.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] asyncMode The register's asyncMode field.
	/// \param[in] rateDivisor The register's rateDivisor field.
	/// \param[in] commonField The register's Group 1 (Common) field.
	/// \param[in] timeField The register's Group 2 (Time) field.
	/// \param[in] imuField The register's Group 3 (IMU) field.
	/// \param[in] gpsField The register's Group 4 (GPS) field.
	/// \param[in] attitudeField The register's Group 5 (Attitude) field.
	/// \param[in] insField The register's Group 6 (INS) field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteBinaryOutput1(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField);

	/// \brief Generates a command to write to the Binary Output 2 register on a VectorNav sensor.
	///
	/// The field outputGroup available on the sensor's register is not
	/// necessary as this method will compute the necessary value from the
	/// provided data fields.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] asyncMode The register's asyncMode field.
	/// \param[in] rateDivisor The register's rateDivisor field.
	/// \param[in] commonField The register's Group 1 (Common) field.
	/// \param[in] timeField The register's Group 2 (Time) field.
	/// \param[in] imuField The register's Group 3 (IMU) field.
	/// \param[in] gpsField The register's Group 4 (GPS) field.
	/// \param[in] attitudeField The register's Group 5 (Attitude) field.
	/// \param[in] insField The register's Group 6 (INS) field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteBinaryOutput2(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField);

	/// \brief Generates a command to write to the Binary Output 3 register on a VectorNav sensor.
	///
	/// The field outputGroup available on the sensor's register is not
	/// necessary as this method will compute the necessary value from the
	/// provided data fields.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] asyncMode The register's asyncMode field.
	/// \param[in] rateDivisor The register's rateDivisor field.
	/// \param[in] commonField The register's Group 1 (Common) field.
	/// \param[in] timeField The register's Group 2 (Time) field.
	/// \param[in] imuField The register's Group 3 (IMU) field.
	/// \param[in] gpsField The register's Group 4 (GPS) field.
	/// \param[in] attitudeField The register's Group 5 (Attitude) field.
	/// \param[in] insField The register's Group 6 (INS) field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteBinaryOutput3(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField);

	/// \brief Generates a command to write sensor settings to non-volatile memory.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to retore factory settings.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genRestoreFactorySettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to reset the sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to read the User Tag register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadUserTag(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the User Tag register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] tag The register's Tag field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteUserTag(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, std::string tag);

	/// \brief Generates a command to read the Model Number register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadModelNumber(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Hardware Revision register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadHardwareRevision(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Serial Number register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSerialNumber(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Firmware Version register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadFirmwareVersion(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Serial Baud Rate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSerialBaudRate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Serial Baud Rate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] baudrate The register's Baud Rate field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSerialBaudRate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t baudrate);

	/// \brief Generates a command to read the Async Data Output Type register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Async Data Output Type register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] ador The register's ADOR field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t ador);

	/// \brief Generates a command to read the Async Data Output Frequency register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Async Data Output Frequency register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] adof The register's ADOF field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t adof);

	/// \brief Generates a command to read the Synchronization Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSynchronizationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Synchronization Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] syncInMode The register's SyncInMode field.
	/// \param[in] syncInEdge The register's SyncInEdge field.
	/// \param[in] syncInSkipFactor The register's SyncInSkipFactor field.
	/// \param[in] reserved1 The register's RESERVED1 field.
	/// \param[in] syncOutMode The register's SyncOutMode field.
	/// \param[in] syncOutPolarity The register's SyncOutPolarity field.
	/// \param[in] syncOutSkipFactor The register's SyncOutSkipFactor field.
	/// \param[in] syncOutPulseWidth The register's SyncOutPulseWidth field.
	/// \param[in] reserved2 The register's RESERVED2 field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSynchronizationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t syncInMode, uint8_t syncInEdge, uint16_t syncInSkipFactor, uint32_t reserved1, uint8_t syncOutMode, uint8_t syncOutPolarity, uint16_t syncOutSkipFactor, uint32_t syncOutPulseWidth, uint32_t reserved2);

	/// \brief Generates a command to read the Communication Protocol Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Communication Protocol Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] serialCount The register's SerialCount field.
	/// \param[in] serialStatus The register's SerialStatus field.
	/// \param[in] spiCount The register's SPICount field.
	/// \param[in] spiStatus The register's SPIStatus field.
	/// \param[in] serialChecksum The register's SerialChecksum field.
	/// \param[in] spiChecksum The register's SPIChecksum field.
	/// \param[in] errorMode The register's ErrorMode field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t serialCount, uint8_t serialStatus, uint8_t spiCount, uint8_t spiStatus, uint8_t serialChecksum, uint8_t spiChecksum, uint8_t errorMode);

	/// \brief Generates a command to read the Synchronization Status register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Synchronization Status register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] syncInCount The register's SyncInCount field.
	/// \param[in] syncInTime The register's SyncInTime field.
	/// \param[in] syncOutCount The register's SyncOutCount field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t syncInCount, uint32_t syncInTime, uint32_t syncOutCount);

	/// \brief Generates a command to read the IMU Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadImuMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Delta Theta and Delta Velocity register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadDeltaThetaAndDeltaVelocity(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Magnetometer Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Magnetometer Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \param[in] b The register's B field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::mat3f c, math::vec3f b);

	/// \brief Generates a command to read the Acceleration Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Acceleration Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \param[in] b The register's B field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::mat3f c, math::vec3f b);

	/// \brief Generates a command to read the Gyro Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGyroCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Gyro Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \param[in] b The register's B field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGyroCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::mat3f c, math::vec3f b);

	/// \brief Generates a command to read the Reference Frame Rotation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Reference Frame Rotation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::mat3f c);

	/// \brief Generates a command to read the IMU Filtering Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the IMU Filtering Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] magWindowSize The register's MagWindowSize field.
	/// \param[in] accelWindowSize The register's AccelWindowSize field.
	/// \param[in] gyroWindowSize The register's GyroWindowSize field.
	/// \param[in] tempWindowSize The register's TempWindowSize field.
	/// \param[in] presWindowSize The register's PresWindowSize field.
	/// \param[in] magFilterMode The register's MagFilterMode field.
	/// \param[in] accelFilterMode The register's AccelFilterMode field.
	/// \param[in] gyroFilterMode The register's GyroFilterMode field.
	/// \param[in] tempFilterMode The register's TempFilterMode field.
	/// \param[in] presFilterMode The register's PresFilterMode field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t magWindowSize, uint16_t accelWindowSize, uint16_t gyroWindowSize, uint16_t tempWindowSize, uint16_t presWindowSize, uint8_t magFilterMode, uint8_t accelFilterMode, uint8_t gyroFilterMode, uint8_t tempFilterMode, uint8_t presFilterMode);

	/// \brief Generates a command to read the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] integrationFrame The register's IntegrationFrame field.
	/// \param[in] gyroCompensation The register's GyroCompensation field.
	/// \param[in] accelCompensation The register's AccelCompensation field.
	/// \param[in] reserved1 The register's Reserved1 field.
	/// \param[in] reserved2 The register's Reserved2 field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t integrationFrame, uint8_t gyroCompensation, uint8_t accelCompensation, uint8_t reserved1, uint16_t reserved2);

	/// \brief Generates a command to read the Yaw Pitch Roll register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRoll(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Attitude Quaternion register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAttitudeQuaternion(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRollMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Quaternion, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadQuaternionMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Magnetic Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagneticMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Acceleration Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAccelerationMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Angular Rate Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAngularRateMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRollTrueBodyAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRollTrueInertialAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the VPE Basic Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeBasicControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Basic Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] enable The register's Enable field.
	/// \param[in] headingMode The register's HeadingMode field.
	/// \param[in] filteringMode The register's FilteringMode field.
	/// \param[in] tuningMode The register's TuningMode field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeBasicControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t enable, uint8_t headingMode, uint8_t filteringMode, uint8_t tuningMode);

	/// \brief Generates a command to read the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] baseTuning The register's BaseTuning field.
	/// \param[in] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::vec3f baseTuning, math::vec3f adaptiveTuning, math::vec3f adaptiveFiltering);

	/// \brief Generates a command to read the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] baseTuning The register's BaseTuning field.
	/// \param[in] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::vec3f baseTuning, math::vec3f adaptiveTuning, math::vec3f adaptiveFiltering);

	/// \brief Generates a command to read the Magnetometer Calibration Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Magnetometer Calibration Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] hsiMode The register's HSIMode field.
	/// \param[in] hsiOutput The register's HSIOutput field.
	/// \param[in] convergeRate The register's ConvergeRate field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t hsiMode, uint8_t hsiOutput, uint8_t convergeRate);

	/// \brief Generates a command to read the Calculated Magnetometer Calibration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadCalculatedMagnetometerCalibration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Velocity Compensation Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Velocity Compensation Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] mode The register's Mode field.
	/// \param[in] velocityTuning The register's VelocityTuning field.
	/// \param[in] rateTuning The register's RateTuning field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t mode, float velocityTuning, float rateTuning);

	/// \brief Generates a command to read the Velocity Compensation Status register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVelocityCompensationStatus(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Velocity Compensation Measurement register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Velocity Compensation Measurement register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] velocity The register's Velocity field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::vec3f velocity);

	/// \brief Generates a command to read the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] magRef The register's MagRef field.
	/// \param[in] accRef The register's AccRef field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::vec3f magRef, math::vec3f accRef);

	/// \brief Generates a command to read the Reference Vector Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Reference Vector Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] useMagModel The register's UseMagModel field.
	/// \param[in] useGravityModel The register's UseGravityModel field.
	/// \param[in] resv1 The register's Resv1 field.
	/// \param[in] resv2 The register's Resv2 field.
	/// \param[in] recalcThreshold The register's RecalcThreshold field.
	/// \param[in] year The register's Year field.
	/// \param[in] position The register's Position field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t useMagModel, uint8_t useGravityModel, uint8_t resv1, uint8_t resv2, uint32_t recalcThreshold, float year, math::vec3d position);

	/// \brief Generates a command to read the GPS Solution - LLA register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsSolutionLla(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the GPS Solution - ECEF register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsSolutionEcef(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the GPS Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the GPS Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] mode The register's Mode field.
	/// \param[in] ppsSource The register's PpsSource field.
	/// \param[in] reserved1 The register's Reserved1 field.
	/// \param[in] reserved2 The register's Reserved2 field.
	/// \param[in] reserved3 The register's Reserved3 field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGpsConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t mode, uint8_t ppsSource, uint8_t reserved1, uint8_t reserved2, uint8_t reserved3);

	/// \brief Generates a command to read the GPS Antenna Offset register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the GPS Antenna Offset register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] position The register's Position field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::vec3f position);

	/// \brief Generates a command to read the GPS Compass Baseline register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the GPS Compass Baseline register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] position The register's Position field.
	/// \param[in] uncertainty The register's Uncertainty field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::vec3f position, math::vec3f uncertainty);

	/// \brief Generates a command to read the GPS Compass Estimated Baseline register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsCompassEstimatedBaseline(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS Solution - LLA register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsSolutionLla(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS Solution - ECEF register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsSolutionEcef(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS State - LLA register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsStateLla(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS State - ECEF register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsStateEcef(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the INS Basic Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] scenario The register's Scenario field.
	/// \param[in] ahrsAiding The register's AhrsAiding field.
	/// \param[in] estBaseline The register's EstBaseline field.
	/// \param[in] resv2 The register's Resv2 field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t scenario, uint8_t ahrsAiding, uint8_t estBaseline, uint8_t resv2);

	/// \brief Generates a command to read the Startup Filter Bias Estimate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Startup Filter Bias Estimate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] gyroBias The register's GyroBias field.
	/// \param[in] accelBias The register's AccelBias field.
	/// \param[in] pressureBias The register's PressureBias field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, math::vec3f gyroBias, math::vec3f accelBias, float pressureBias);

	/// \}

	/// \defgroup uartPacketAsciiAsyncParsers UART ASCII Asynchronous Packet Parsers
	/// \brief This group of methods allow parsing of ASCII asynchronous data
	/// packets from VectorNav sensors.
	///
	/// The units are not specified for the out parameters since these
	/// methods do a simple conversion operation from the packet string. Please
	/// consult the appropriate sensor user manual for details about
	/// the units returned by the sensor.
	///
	/// \{

	/// \brief Parses a VNYPR asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	void parseVNYPR(math::vec3f *yawPitchRoll);

	/// \brief Parses a VNQTN asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	void parseVNQTN(math::vec4f *quaternion);

	/// \brief Parses a VNQMR asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNQMR(math::vec4f *quaternion, math::vec3f *magnetic, math::vec3f *acceleration, math::vec3f *angularRate);

	/// \brief Parses a VNMAG asynchronous packet.
	///
	/// \param[out] magnetic The magnetic values in the packet.
	void parseVNMAG(math::vec3f *magnetic);

	/// \brief Parses a VNACC asynchronous packet.
	///
	/// \param[out] acceleration The acceleration values in the packet.
	void parseVNACC(math::vec3f *acceleration);

	/// \brief Parses a VNGYR asynchronous packet.
	///
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNGYR(math::vec3f *angularRate);
	
	/// \brief Parses a VNMAR asynchronous packet.
	///
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNMAR(math::vec3f *magnetic, math::vec3f *acceleration, math::vec3f *angularRate);
	
	/// \brief Parses a VNYMR asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNYMR(math::vec3f *yawPitchRoll, math::vec3f *magnetic, math::vec3f *acceleration, math::vec3f *angularRate);

	/// \brief Parses a VNYBA asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] accelerationBody The acceleration body values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNYBA(math::vec3f *yawPitchRoll, math::vec3f *accelerationBody, math::vec3f *angularRate);

	/// \brief Parses a VNYIA asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] accelerationInertial The acceleration inertial values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNYIA(math::vec3f *yawPitchRoll, math::vec3f *accelerationInertial, math::vec3f *angularRate);

	/// \brief Parses a VNIMU asynchronous packet.
	///
	/// \param[out] magneticUncompensated The uncompensated magnetic values in the packet.
	/// \param[out] accelerationUncompensated The uncompensated acceleration values in the packet.
	/// \param[out] angularRateUncompensated The uncompensated angular rate values in the packet.
	/// \param[out] temperature The temperature value in the packet.
	/// \param[out] pressure The pressure value in the packet.
	void parseVNIMU(math::vec3f *magneticUncompensated, math::vec3f *accelerationUncompensated, math::vec3f *angularRateUncompensated, float *temperature, float *pressure);

	/// \brief Parses a VNGPS asynchronous packet.
	///
	/// \param[out] time The time value in the packet.
	/// \param[out] week The week value in the packet.
	/// \param[out] gpsFix The GPS fix value in the packet.
	/// \param[out] numSats The NumSats value in the packet.
	/// \param[out] lla The latitude, longitude and altitude values in the packet.
	/// \param[out] nedVel The NED velocity values in the packet.
	/// \param[out] nedAcc The NED position accuracy values in the packet.
	/// \param[out] speedAcc The SpeedAcc value in the packet.
	/// \param[out] timeAcc The TimeAcc value in the packet.
	void parseVNGPS(double *time, uint16_t *week, uint8_t *gpsFix, uint8_t *numSats, math::vec3d *lla, math::vec3f *nedVel, math::vec3f *nedAcc, float *speedAcc, float *timeAcc);
	
	/// \brief Parses a VNINS asynchronous packet.
	///
	/// \param[out] time The time value in the packet.
	/// \param[out] week The week value in the packet.
	/// \param[out] status The status value in the packet.
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] lla The latitude, longitude, altitude values in the packet.
	/// \param[out] nedVel The NED velocity values in the packet.
	/// \param[out] attUncertainty The attitude uncertainty value in the packet.
	/// \param[out] posUncertainty The position uncertainty value in the packet.
	/// \param[out] velUncertainty The velocity uncertainty value in the packet.
	void parseVNINS(double *time, uint16_t *week, uint16_t *status, math::vec3f *yawPitchRoll, math::vec3d *lla, math::vec3f *nedVel, float *attUncertainty, float *posUncertainty, float *velUncertainty);

	/// \brief Parses a VNGPE asynchronous packet.
	///
	/// \param[out] tow The tow value in the packet.
	/// \param[out] week The week value in the packet.
	/// \param[out] gpsFix The GPS fix value in the packet.
	/// \param[out] numSats The numSats value in the packet.
	/// \param[out] position The ECEF position values in the packet.
	/// \param[out] velocity The ECEF velocity values in the packet.
	/// \param[out] posAcc The PosAcc values in the packet.
	/// \param[out] speedAcc The SpeedAcc value in the packet.
	/// \param[out] timeAcc The TimeAcc value in the packet.
	void parseVNGPE(double *tow, uint16_t *week, uint8_t *gpsFix, uint8_t *numSats, math::vec3d *position, math::vec3f *velocity, math::vec3f *posAcc, float *speedAcc, float *timeAcc);

	/// \brief Parses a VNDTV asynchronous packet.
	///
	/// \param[out] deltaTime The DeltaTime value in the packet.
	/// \param[out] deltaTheta The DeltaTheta values in the packet.
	/// \param[out] deltaVelocity The DeltaVelocity values in the packet.
	void parseVNDTV(float *deltaTime, math::vec3f *deltaTheta, math::vec3f *deltaVelocity);

	/// \}

	/// \defgroup uartAsciiResponseParsers UART ASCII Response Packet Parsers
	/// \brief This group of methods allow parsing of ASCII response data
	/// packets from VectorNav's sensors.
	///
	/// The units are not specified for the out parameters since these
	/// methods do a simple conversion operation from the packet string. Please
	/// consult the appropriate user manual for your sensor for details about
	/// the units returned by the sensor.
	///
	/// \{

	/// \brief Parses a response from reading any of the Binary Output registers.
	///
	/// \param[out] asyncMode The register's AsyncMode field.
	/// \param[out] rateDivisor The register's RateDivisor field.
	/// \param[out] outputGroup The register's OutputGroup field.
	/// \param[out] commonField The set fields of Output Group 1 (Common) if present.
	/// \param[out] timeField The set fields of Output Group 2 (Time) if present.
	/// \param[out] imuField The set fields of Output Group 3 (IMU) if present.
	/// \param[out] gpsField The set fields of Output Group 4 (GPS) if present.
	/// \param[out] attitudeField The set fields of Output Group 5 (Attitude) if present.
	/// \param[out] insField The set fields of Output Group 6 (INS) if present.
	void parseBinaryOutput(
		uint16_t* asyncMode,
		uint16_t* rateDivisor,
		uint16_t* outputGroup,
		uint16_t* commonField,
		uint16_t* timeField,
		uint16_t* imuField,
		uint16_t* gpsField,
		uint16_t* attitudeField,
		uint16_t* insField);

	/// \brief Parses a response from reading the User Tag register.
	///
	/// \param[out] tag The register's Tag field.
	void parseUserTag(char* tag);

	/// \brief Parses a response from reading the Model Number register.
	///
	/// \param[out] productName The register's Product Name field.
	void parseModelNumber(char* productName);

	/// \brief Parses a response from reading the Hardware Revision register.
	///
	/// \param[out] revision The register's Revision field.
	void parseHardwareRevision(uint32_t* revision);

	/// \brief Parses a response from reading the Serial Number register.
	///
	/// \param[out] serialNum The register's SerialNum field.
	void parseSerialNumber(uint32_t* serialNum);

	/// \brief Parses a response from reading the Firmware Version register.
	///
	/// \param[out] firmwareVersion The register's Firmware Version field.
	void parseFirmwareVersion(char* firmwareVersion);

	/// \brief Parses a response from reading the Serial Baud Rate register.
	///
	/// \param[out] baudrate The register's Baud Rate field.
	void parseSerialBaudRate(uint32_t* baudrate);

	/// \brief Parses a response from reading the Async Data Output Type register.
	///
	/// \param[out] ador The register's ADOR field.
	void parseAsyncDataOutputType(uint32_t* ador);

	/// \brief Parses a response from reading the Async Data Output Frequency register.
	///
	/// \param[out] adof The register's ADOF field.
	void parseAsyncDataOutputFrequency(uint32_t* adof);

	/// \brief Parses a response from reading the Synchronization Control register.
	///
	/// \param[out] syncInMode The register's SyncInMode field.
	/// \param[out] syncInEdge The register's SyncInEdge field.
	/// \param[out] syncInSkipFactor The register's SyncInSkipFactor field.
	/// \param[out] syncOutMode The register's SyncOutMode field.
	/// \param[out] syncOutPolarity The register's SyncOutPolarity field.
	/// \param[out] syncOutSkipFactor The register's SyncOutSkipFactor field.
	/// \param[out] syncOutPulseWidth The register's SyncOutPulseWidth field.
	void parseSynchronizationControl(uint8_t* syncInMode, uint8_t* syncInEdge, uint16_t* syncInSkipFactor, uint8_t* syncOutMode, uint8_t* syncOutPolarity, uint16_t* syncOutSkipFactor, uint32_t* syncOutPulseWidth);

	/// \brief Parses a response from reading the Communication Protocol Control register.
	///
	/// \param[out] serialCount The register's SerialCount field.
	/// \param[out] serialStatus The register's SerialStatus field.
	/// \param[out] spiCount The register's SPICount field.
	/// \param[out] spiStatus The register's SPIStatus field.
	/// \param[out] serialChecksum The register's SerialChecksum field.
	/// \param[out] spiChecksum The register's SPIChecksum field.
	/// \param[out] errorMode The register's ErrorMode field.
	void parseCommunicationProtocolControl(uint8_t* serialCount, uint8_t* serialStatus, uint8_t* spiCount, uint8_t* spiStatus, uint8_t* serialChecksum, uint8_t* spiChecksum, uint8_t* errorMode);

	/// \brief Parses a response from reading the Synchronization Status register.
	///
	/// \param[out] syncInCount The register's SyncInCount field.
	/// \param[out] syncInTime The register's SyncInTime field.
	/// \param[out] syncOutCount The register's SyncOutCount field.
	void parseSynchronizationStatus(uint32_t* syncInCount, uint32_t* syncInTime, uint32_t* syncOutCount);

	/// \brief Parses a response from reading the IMU Measurements register.
	///
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	/// \param[out] temp The register's Temp field.
	/// \param[out] pressure The register's Pressure field.
	void parseImuMeasurements(math::vec3f* mag, math::vec3f* accel, math::vec3f* gyro, float* temp, float* pressure);

	/// \brief Parses a response from reading the Delta Theta and Delta Velocity register.
	///
	/// \param[out] deltaTime The register's DeltaTime field.
	/// \param[out] deltaTheta The register's DeltaTheta field.
	/// \param[out] deltaVelocity The register's DeltaVelocity field.
	void parseDeltaThetaAndDeltaVelocity(float* deltaTime, math::vec3f* deltaTheta, math::vec3f* deltaVelocity);

	/// \brief Parses a response from reading the Magnetometer Compensation register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseMagnetometerCompensation(math::mat3f* c, math::vec3f* b);

	/// \brief Parses a response from reading the Acceleration Compensation register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseAccelerationCompensation(math::mat3f* c, math::vec3f* b);

	/// \brief Parses a response from reading the Gyro Compensation register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseGyroCompensation(math::mat3f* c, math::vec3f* b);

	/// \brief Parses a response from reading the Reference Frame Rotation register.
	///
	/// \param[out] c The register's C field.
	void parseReferenceFrameRotation(math::mat3f* c);

	/// \brief Parses a response from reading the IMU Filtering Configuration register.
	///
	/// \param[out] magWindowSize The register's MagWindowSize field.
	/// \param[out] accelWindowSize The register's AccelWindowSize field.
	/// \param[out] gyroWindowSize The register's GyroWindowSize field.
	/// \param[out] tempWindowSize The register's TempWindowSize field.
	/// \param[out] presWindowSize The register's PresWindowSize field.
	/// \param[out] magFilterMode The register's MagFilterMode field.
	/// \param[out] accelFilterMode The register's AccelFilterMode field.
	/// \param[out] gyroFilterMode The register's GyroFilterMode field.
	/// \param[out] tempFilterMode The register's TempFilterMode field.
	/// \param[out] presFilterMode The register's PresFilterMode field.
	void parseImuFilteringConfiguration(uint16_t* magWindowSize, uint16_t* accelWindowSize, uint16_t* gyroWindowSize, uint16_t* tempWindowSize, uint16_t* presWindowSize, uint8_t* magFilterMode, uint8_t* accelFilterMode, uint8_t* gyroFilterMode, uint8_t* tempFilterMode, uint8_t* presFilterMode);

	/// \brief Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
	///
	/// \param[out] integrationFrame The register's IntegrationFrame field.
	/// \param[out] gyroCompensation The register's GyroCompensation field.
	/// \param[out] accelCompensation The register's AccelCompensation field.
	void parseDeltaThetaAndDeltaVelocityConfiguration(uint8_t* integrationFrame, uint8_t* gyroCompensation, uint8_t* accelCompensation);

	/// \brief Parses a response from reading the Yaw Pitch Roll register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	void parseYawPitchRoll(math::vec3f* yawPitchRoll);

	/// \brief Parses a response from reading the Attitude Quaternion register.
	///
	/// \param[out] quat The register's Quat field.
	void parseAttitudeQuaternion(math::vec4f* quat);

	/// \brief Parses a response from reading the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	void parseYawPitchRollMagneticAccelerationAndAngularRates(math::vec3f* yawPitchRoll, math::vec3f* mag, math::vec3f* accel, math::vec3f* gyro);

	/// \brief Parses a response from reading the Quaternion, Magnetic, Acceleration and Angular Rates register.
	///
	/// \param[out] quat The register's Quat field.
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	void parseQuaternionMagneticAccelerationAndAngularRates(math::vec4f* quat, math::vec3f* mag, math::vec3f* accel, math::vec3f* gyro);

	/// \brief Parses a response from reading the Magnetic Measurements register.
	///
	/// \param[out] mag The register's Mag field.
	void parseMagneticMeasurements(math::vec3f* mag);

	/// \brief Parses a response from reading the Acceleration Measurements register.
	///
	/// \param[out] accel The register's Accel field.
	void parseAccelerationMeasurements(math::vec3f* accel);

	/// \brief Parses a response from reading the Angular Rate Measurements register.
	///
	/// \param[out] gyro The register's Gyro field.
	void parseAngularRateMeasurements(math::vec3f* gyro);

	/// \brief Parses a response from reading the Magnetic, Acceleration and Angular Rates register.
	///
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	void parseMagneticAccelerationAndAngularRates(math::vec3f* mag, math::vec3f* accel, math::vec3f* gyro);

	/// \brief Parses a response from reading the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] bodyAccel The register's BodyAccel field.
	/// \param[out] gyro The register's Gyro field.
	void parseYawPitchRollTrueBodyAccelerationAndAngularRates(math::vec3f* yawPitchRoll, math::vec3f* bodyAccel, math::vec3f* gyro);

	/// \brief Parses a response from reading the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] inertialAccel The register's InertialAccel field.
	/// \param[out] gyro The register's Gyro field.
	void parseYawPitchRollTrueInertialAccelerationAndAngularRates(math::vec3f* yawPitchRoll, math::vec3f* inertialAccel, math::vec3f* gyro);

	/// \brief Parses a response from reading the VPE Basic Control register.
	///
	/// \param[out] enable The register's Enable field.
	/// \param[out] headingMode The register's HeadingMode field.
	/// \param[out] filteringMode The register's FilteringMode field.
	/// \param[out] tuningMode The register's TuningMode field.
	void parseVpeBasicControl(uint8_t* enable, uint8_t* headingMode, uint8_t* filteringMode, uint8_t* tuningMode);

	/// \brief Parses a response from reading the VPE Magnetometer Basic Tuning register.
	///
	/// \param[out] baseTuning The register's BaseTuning field.
	/// \param[out] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
	void parseVpeMagnetometerBasicTuning(math::vec3f* baseTuning, math::vec3f* adaptiveTuning, math::vec3f* adaptiveFiltering);

	/// \brief Parses a response from reading the VPE Accelerometer Basic Tuning register.
	///
	/// \param[out] baseTuning The register's BaseTuning field.
	/// \param[out] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
	void parseVpeAccelerometerBasicTuning(math::vec3f* baseTuning, math::vec3f* adaptiveTuning, math::vec3f* adaptiveFiltering);

	/// \brief Parses a response from reading the Magnetometer Calibration Control register.
	///
	/// \param[out] hsiMode The register's HSIMode field.
	/// \param[out] hsiOutput The register's HSIOutput field.
	/// \param[out] convergeRate The register's ConvergeRate field.
	void parseMagnetometerCalibrationControl(uint8_t* hsiMode, uint8_t* hsiOutput, uint8_t* convergeRate);

	/// \brief Parses a response from reading the Calculated Magnetometer Calibration register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseCalculatedMagnetometerCalibration(math::mat3f* c, math::vec3f* b);

	/// \brief Parses a response from reading the Velocity Compensation Control register.
	///
	/// \param[out] mode The register's Mode field.
	/// \param[out] velocityTuning The register's VelocityTuning field.
	/// \param[out] rateTuning The register's RateTuning field.
	void parseVelocityCompensationControl(uint8_t* mode, float* velocityTuning, float* rateTuning);

	/// \brief Parses a response from reading the Velocity Compensation Status register.
	///
	/// \param[out] x The register's x field.
	/// \param[out] xDot The register's xDot field.
	/// \param[out] accelOffset The register's accelOffset field.
	/// \param[out] omega The register's omega field.
	void parseVelocityCompensationStatus(float* x, float* xDot, math::vec3f* accelOffset, math::vec3f* omega);

	/// \brief Parses a response from reading the Velocity Compensation Measurement register.
	///
	/// \param[out] velocity The register's Velocity field.
	void parseVelocityCompensationMeasurement(math::vec3f* velocity);

	/// \brief Parses a response from reading the Magnetic and Gravity Reference Vectors register.
	///
	/// \param[out] magRef The register's MagRef field.
	/// \param[out] accRef The register's AccRef field.
	void parseMagneticAndGravityReferenceVectors(math::vec3f* magRef, math::vec3f* accRef);

	/// \brief Parses a response from reading the Reference Vector Configuration register.
	///
	/// \param[out] useMagModel The register's UseMagModel field.
	/// \param[out] useGravityModel The register's UseGravityModel field.
	/// \param[out] recalcThreshold The register's RecalcThreshold field.
	/// \param[out] year The register's Year field.
	/// \param[out] position The register's Position field.
	void parseReferenceVectorConfiguration(uint8_t* useMagModel, uint8_t* useGravityModel, uint32_t* recalcThreshold, float* year, math::vec3d* position);

	/// \brief Parses a response from reading the GPS Solution - LLA register.
	///
	/// \param[out] time The register's Time field.
	/// \param[out] week The register's Week field.
	/// \param[out] gpsFix The register's GpsFix field.
	/// \param[out] numSats The register's NumSats field.
	/// \param[out] lla The register's Lla field.
	/// \param[out] nedVel The register's NedVel field.
	/// \param[out] nedAcc The register's NedAcc field.
	/// \param[out] speedAcc The register's SpeedAcc field.
	/// \param[out] timeAcc The register's TimeAcc field.
	void parseGpsSolutionLla(double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, math::vec3d* lla, math::vec3f* nedVel, math::vec3f* nedAcc, float* speedAcc, float* timeAcc);

	/// \brief Parses a response from reading the GPS Solution - ECEF register.
	///
	/// \param[out] tow The register's Tow field.
	/// \param[out] week The register's Week field.
	/// \param[out] gpsFix The register's GpsFix field.
	/// \param[out] numSats The register's NumSats field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] posAcc The register's PosAcc field.
	/// \param[out] speedAcc The register's SpeedAcc field.
	/// \param[out] timeAcc The register's TimeAcc field.
	void parseGpsSolutionEcef(double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, math::vec3d* position, math::vec3f* velocity, math::vec3f* posAcc, float* speedAcc, float* timeAcc);

	/// \brief Parses a response from reading the GPS Configuration register.
	///
	/// \param[out] mode The register's Mode field.
	/// \param[out] ppsSource The register's PpsSource field.
	void parseGpsConfiguration(uint8_t* mode, uint8_t* ppsSource);

	/// \brief Parses a response from reading the GPS Antenna Offset register.
	///
	/// \param[out] position The register's Position field.
	void parseGpsAntennaOffset(math::vec3f* position);

	/// \brief Parses a response from reading the GPS Compass Baseline register.
	///
	/// \param[out] position The register's Position field.
	/// \param[out] uncertainty The register's Uncertainty field.
	void parseGpsCompassBaseline(math::vec3f* position, math::vec3f* uncertainty);

	/// \brief Parses a response from reading the GPS Compass Estimated Baseline register.
	///
	/// \param[out] estBaselineUsed The register's EstBaselineUsed field.
	/// \param[out] numMeas The register's NumMeas field.
	/// \param[out] position The register's Position field.
	/// \param[out] uncertainty The register's Uncertainty field.
	void parseGpsCompassEstimatedBaseline(uint8_t* estBaselineUsed, uint16_t* numMeas, math::vec3f* position, math::vec3f* uncertainty);

	/// \brief Parses a response from reading the INS Solution - LLA register.
	///
	/// \param[out] time The register's Time field.
	/// \param[out] week The register's Week field.
	/// \param[out] status The register's Status field.
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] nedVel The register's NedVel field.
	/// \param[out] attUncertainty The register's AttUncertainty field.
	/// \param[out] posUncertainty The register's PosUncertainty field.
	/// \param[out] velUncertainty The register's VelUncertainty field.
	void parseInsSolutionLla(double* time, uint16_t* week, uint16_t* status, math::vec3f* yawPitchRoll, math::vec3d* position, math::vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty);

	/// \brief Parses a response from reading the INS Solution - ECEF register.
	///
	/// \param[out] time The register's Time field.
	/// \param[out] week The register's Week field.
	/// \param[out] status The register's Status field.
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] attUncertainty The register's AttUncertainty field.
	/// \param[out] posUncertainty The register's PosUncertainty field.
	/// \param[out] velUncertainty The register's VelUncertainty field.
	void parseInsSolutionEcef(double* time, uint16_t* week, uint16_t* status, math::vec3f* yawPitchRoll, math::vec3d* position, math::vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty);

	/// \brief Parses a response from reading the INS State - LLA register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] angularRate The register's AngularRate field.
	void parseInsStateLla(math::vec3f* yawPitchRoll, math::vec3d* position, math::vec3f* velocity, math::vec3f* accel, math::vec3f* angularRate);

	/// \brief Parses a response from reading the INS State - ECEF register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] angularRate The register's AngularRate field.
	void parseInsStateEcef(math::vec3f* yawPitchRoll, math::vec3d* position, math::vec3f* velocity, math::vec3f* accel, math::vec3f* angularRate);

	/// \brief Parses a response from reading the INS Basic Configuration register.
	///
	/// \param[out] scenario The register's Scenario field.
	/// \param[out] ahrsAiding The register's AhrsAiding field.
	void parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding);

	/// \brief Parses a response from reading the INS Basic Configuration register.
	///
	/// \param[out] scenario The register's Scenario field.
	/// \param[out] ahrsAiding The register's AhrsAiding field.
	/// \param[out] estBaseline The register's EstBaseline field.
	void parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding, uint8_t* estBaseline);

	/// \brief Parses a response from reading the Startup Filter Bias Estimate register.
	///
	/// \param[out] gyroBias The register's GyroBias field.
	/// \param[out] accelBias The register's AccelBias field.
	/// \param[out] pressureBias The register's PressureBias field.
	void parseStartupFilterBiasEstimate(math::vec3f* gyroBias, math::vec3f* accelBias, float* pressureBias);

	/// \}

private:

	void ensureCanExtract(size_t numOfBytes);

	Type _type;
	bool _isPacketDataMine;
	size_t _length;
	char *_data;
	size_t _curExtractLoc;
};

/// \brief Helps with management of communication with a sensor using the UART
/// protocol.
///
/// Internally, the PacketFinder keeps track of a running data index which
/// keeps a running count of the bytes that are processed by the class. This is
/// useful for users who wish to keep track of where packets where found in the
/// incoming raw data stream. When the PacketFinder receives its first byte
/// from the user, this is given the index of 0 for the running index and
/// incremented for each byte received.
class PacketFinder : private util::NoCopy
{

public:

	/// \brief Defines the signature for a method that can receive
	/// notifications of new valid packets found.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerPossiblePacketFoundHandler.
	/// \param[in] possiblePacket The possible packet that was found.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	typedef void (*ValidPacketFoundHandler)(void* userData, Packet& packet, size_t runningIndexOfPacketStart);

	/// \brief Creates a new /ref PacketFinder with internal buffers to store
	/// incoming bytes and alert when valid packets are received.
	PacketFinder();

	/// \brief Creates a new /ref PacketFinder with an internal buffer the size
	/// specified.
	///
	/// \param[in] internalReceiveBufferSize The number of bytes to make the
	///     internal buffer.
	explicit PacketFinder(size_t internalReceiveBufferSize);

	~PacketFinder();

	/// \brief Adds new data to the internal buffers and processes the received
	/// data to determine if any new received packets are available.
	///
	/// \param[in] data The data buffer containing the received data.
	/// \param[in] length The number of bytes of data in the buffer.
	void processReceivedData(char data[], size_t length);

	/// \brief Registers a callback method for notification when a new possible
	/// packet is found.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterPossiblePacketFoundHandler();

private:
	struct Impl;
	Impl *_pi;
};

// Utility functions.

/// \brief Converts an AsciiAsync enum into a string.
///
/// \param[in] val The AsciiAsync enum value to convert to string.
/// \return The converted value.
std::string str(AsciiAsync val);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// AsciiAsync enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, AsciiAsync e);

/// \brief Converts a SensorError enum into a string.
///
/// \param[in] val The SensorError enum value to convert to string.
/// \return The converted value.
std::string str(SensorError val);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SensorError enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SensorError e);

/// \brief Allows combining flags of the CommonGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
CommonGroup operator|(CommonGroup lhs, CommonGroup rhs);

/// \brief Allows combining flags of the TimeGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
TimeGroup operator|(TimeGroup lhs, TimeGroup rhs);

/// \brief Allows combining flags of the ImuGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
ImuGroup operator|(ImuGroup lhs, ImuGroup rhs);

/// \brief Allows combining flags of the GpsGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
GpsGroup operator|(GpsGroup lhs, GpsGroup rhs);

/// \brief Allows combining flags of the AttitudeGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
AttitudeGroup operator|(AttitudeGroup lhs, AttitudeGroup rhs);

/// \brief Allows combining flags of the InsGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
InsGroup operator|(InsGroup lhs, InsGroup rhs);

/// \brief Converts a SyncInMode enum into a string.
///
/// \param[in] val The SyncInMode enum value to convert to string.
/// \return The converted value.
std::string str(SyncInMode val);

/// \brief Converts a SyncInEdge enum into a string.
///
/// \param[in] val The SyncInEdge enum value to convert to string.
/// \return The converted value.
std::string str(SyncInEdge val);

/// \brief Converts a SyncOutMode enum into a string.
///
/// \param[in] val The SyncOutMode enum value to convert to string.
/// \return The converted value.
std::string str(SyncOutMode val);

/// \brief Converts a SyncOutPolarity enum into a string.
///
/// \param[in] val The SyncOutPolarity enum value to convert to string.
/// \return The converted value.
std::string str(SyncOutPolarity val);

/// \brief Converts a CountMode enum into a string.
///
/// \param[in] val The CountMode enum value to convert to string.
/// \return The converted value.
std::string str(CountMode val);

/// \brief Converts a StatusMode enum into a string.
///
/// \param[in] val The StatusMode enum value to convert to string.
/// \return The converted value.
std::string str(StatusMode val);

/// \brief Converts a ChecksumMode enum into a string.
///
/// \param[in] val The ChecksumMode enum value to convert to string.
/// \return The converted value.
std::string str(ChecksumMode val);

/// \brief Converts a ErrorMode enum into a string.
///
/// \param[in] val The ErrorMode enum value to convert to string.
/// \return The converted value.
std::string str(ErrorMode val);

/// \brief Converts a FilterMode enum into a string.
///
/// \param[in] val The FilterMode enum value to convert to string.
/// \return The converted value.
std::string str(FilterMode val);

/// \brief Converts a IntegrationFrame enum into a string.
///
/// \param[in] val The IntegrationFrame enum value to convert to string.
/// \return The converted value.
std::string str(IntegrationFrame val);

/// \brief Converts a CompensationMode enum into a string.
///
/// \param[in] val The CompensationMode enum value to convert to string.
/// \return The converted value.
std::string str(CompensationMode val);

/// \brief Converts a GpsFix enum into a string.
///
/// \param[in] val The GpsFix enum value to convert to string.
/// \return The converted value.
std::string str(GpsFix val);

/// \brief Converts a GpsMode enum into a string.
///
/// \param[in] val The GpsMode enum value to convert to string.
/// \return The converted value.
std::string str(GpsMode val);

/// \brief Converts a PpsSource enum into a string.
///
/// \param[in] val The PpsSource enum value to convert to string.
/// \return The converted value.
std::string str(PpsSource val);

/// \brief Converts a VpeEnable enum into a string.
///
/// \param[in] val The VpeEnable enum value to convert to string.
/// \return The converted value.
std::string str(VpeEnable val);

/// \brief Converts a HeadingMode enum into a string.
///
/// \param[in] val The HeadingMode enum value to convert to string.
/// \return The converted value.
std::string str(HeadingMode val);

/// \brief Converts a VpeMode enum into a string.
///
/// \param[in] val The VpeMode enum value to convert to string.
/// \return The converted value.
std::string str(VpeMode val);

/// \brief Converts a Scenario enum into a string.
///
/// \param[in] val The Scenario enum value to convert to string.
/// \return The converted value.
std::string str(Scenario val);

/// \brief Converts a HsiMode enum into a string.
///
/// \param[in] val The HsiMode enum value to convert to string.
/// \return The converted value.
std::string str(HsiMode val);

/// \brief Converts a HsiOutput enum into a string.
///
/// \param[in] val The HsiOutput enum value to convert to string.
/// \return The converted value.
std::string str(HsiOutput val);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncInMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncInMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncInEdge enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncInEdge e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncOutMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncOutMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncOutPolarity enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncOutPolarity e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// CountMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, CountMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// StatusMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, StatusMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// ChecksumMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, ChecksumMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// ErrorMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, ErrorMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// FilterMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, FilterMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// IntegrationFrame enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, IntegrationFrame e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// CompensationMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, CompensationMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// GpsFix enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, GpsFix e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// GpsMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, GpsMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// PpsSource enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, PpsSource e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// VpeEnable enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, VpeEnable e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// HeadingMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, HeadingMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// VpeMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, VpeMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// Scenario enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, Scenario e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// HsiMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, HsiMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// HsiOutput enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, HsiOutput e);

}
}
}

#endif
