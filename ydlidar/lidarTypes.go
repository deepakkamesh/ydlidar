package ydlidar

import "go.bug.st/serial"

// YDLidar is the lidar object.
type YDLidar struct {
	SerialPort serial.Port
	Packets    chan Packet
	Stop       chan struct{}
}

// Models Each model has a different set of commands
type Models struct {
	X2 X2 `json:"X2"`
	X4 X4 `json:"X4"`
	G1 G1 `json:"G1"`
	G2 G2 `json:"G2"`
	G4 G4 `json:"G4"`
	G6 G6 `json:"G6"`
}

type X2 struct {
}

type X4 struct {
}

type G1 struct {
}

type G2 struct {
}

type G4 struct {
}

type G6 struct {
}

const (
	// preCommand is the command to send before sending any other command.
	preCommand = 0xA5

	// healthStatus is the command to get the health status.
	healthStatus = 0x92

	// deviceInfo is the command to get the device information.
	deviceInfo = 0x90

	// resetDevice is the command to reset the device.
	restartDevice = 0x40

	// stopScanning is the command to stop scanning.
	stopScanning = 0x65

	// startScanning is the command to start scanning.
	startScanning = 0x60

	// HealthTypeCode is the device response Health HealthInfo type code.
	HealthTypeCode = 0x06

	// InfoTypeCode is the device response Device Information type code.
	InfoTypeCode = 0x04

	// ScanTypeCode is the device response Scan Command type code.
	ScanTypeCode = 0x81

	SingleResponse     = 0x0
	ContinuousResponse = 0x1
)

// PointCloudData represents a single lidar reading.
type PointCloudData struct {
	Intensity int
	Dist      uint16
	Angle     float32
}

// Packet represents struct of a single sample set of readings as translated by this application
type Packet struct {
	FirstAngle         float32  // First/Minimum angle corresponds to first distance sample.
	LastAngle          float32  // Last/Max angle corresponds to last distance sample.
	DeltaAngle         float32  // Delta between Min and Max Angles.
	NumDistanceSamples int      // Number of distance samples.
	Distances          []uint16 // Slice containing distance data.
	Intensities        []int    // Slice containing intensity data.
	PacketType         uint8    // Indicates the current packet type. 0x00: Point cloud packet 0x01: Zero packet.
	Error              error
}

// DeviceInfo Works with G2
// DeviceInfo contains the device model, firmware, hardware, and serial number.
type DeviceInfo struct {
	Model    byte     // Model number.
	Firmware [2]byte  // Firmware version.
	Hardware byte     // Hardware version.
	Serial   [16]byte // Serial number.
}

// DeviceInfoString Works with G2
// DeviceInfoString contains the device model, firmware, hardware, and serial number.
type DeviceInfoString struct {
	Model    string // Model number.
	Firmware string // Firmware version.
	Hardware string // Hardware version.
	Serial   string // Serial number.
}

// pointCloudHeader is the preamble for the point cloud data from the lidar
type pointCloudHeader struct {
	// PacketHeader 2B in length, fixed at 0x55AA, low in front, high in back
	// PH(2B)
	PacketHeader uint16

	// PackageType F(bit7:1): represents the scanning frequency of the lidar at the current moment,
	// the value is valid in the initial data packet, and the value is 0 by default in the
	// point cloud data packet; C(bit0): represents the type of the current data packet;
	// 0x00: Point cloud data package 0x01: Start data package
	// F&C (1B) [0 0 0 0 0 0 0 0]
	PackageType uint8

	// SampleQuantity Indicates the number of sampling points contained in the current packet. There is only one zero point of data in the zero packet. The value is 1.
	// LSN(1B)
	SampleQuantity uint8

	// StartAngle The angle data corresponding to the first sample point in the sampled data
	// FSA(2B)
	StartAngle uint16

	// EndAngle The angle data corresponding to the last sample point in the sampled data
	// LSA(2B)
	EndAngle uint16

	// CheckCode The check code of the current data packet uses a two-byte exclusive OR to check the current data packet.
	// CS(2B)
	CheckCode uint16
}
