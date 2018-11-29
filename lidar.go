/*
* Package ydlidar provides go api for YDLidar X4.
* https://www.robotshop.com/media/files/content/y/ydl/pdf/ydlidar_x4_development_manual.pdf
 */
package ydlidar

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"math"

	"github.com/deepakkamesh/go-serial/serial"
	"github.com/golang/glog"
)

const (
	// YDLIDAR Commands.
	START      = 0xA5
	CSTATUS    = 0x91
	CINFO      = 0x90
	CRESTART   = 0x40
	CSTOPSCAN  = 0x65
	CSTARTSCAN = 0x60

	//YDLIDAR Type Codes.
	STATUS_TYPE_CODE = 0x06
	INFO_TYPE_CODE   = 0x04
	SCAN_TYPE_CODE   = 0x81
)

// YDLidar is the lidar object.
type YDLidar struct {
	ser  serial.SerialPort
	D    chan DataPoint
	Dp   chan DistPacket
	stop chan struct{}
}

// DataPoint represents a single lidar reading.
type DataPoint struct {
	Angle  float64
	Dist   float64
	ZeroPt bool
	Error  error
}

//DistPacket represents struct of a single sample set of readings.
type DistPacket struct {
	MinAngle  float64   // Minimum angle corresponds to first distance sample.
	MaxAngle  float64   // Max angle corresponds to last distance sample.
	Num       int       // Number of distance samples.
	Distances []float64 // Slice containing distance data.
}

// DeviceInfo struct contains the version information.
type DeviceInfo struct {
	model    byte     // Model number.
	firmware [2]byte  // firmware version.
	hardware byte     // hardware version.
	serial   [16]byte // serial number.
}

// pointCloudHeader is the preample for the point cloud data.
type pointCloudHeader struct {
	Header int16  // The length is 2B, fixed at 0x55AA, low in front, high in back.
	Pkt    int8   // Indicates the current packet type. 0x00: Point cloud packet 0x01: Zero packet.
	Num    int8   // Indicates the number of sample points contained in the current packet.
	Fsa    uint16 // The angle data corresponding to the first sample point in the sampled data.
	Lsa    uint16 // The angle data corresponding to the last sample point in the sampled data.
	Chksum uint16 //  Two-byte exclusive OR checksum value.

}

// NewLidar returns a YDLidar object.
func NewLidar() *YDLidar {
	return &YDLidar{
		D:    make(chan DataPoint),
		Dp:   make(chan DistPacket),
		stop: make(chan struct{}),
	}
}

// SetMockSerial used to set Mock serial interface for testing.
// Do not call Init if using the mock serial.
func (l *YDLidar) SetMockSerial(s serial.SerialPort) {
	l.ser = s
}

// Init initializes the Lidar.
func (l *YDLidar) Init(ttyPort string) error {

	c := serial.OpenOptions{
		PortName:              ttyPort,
		BaudRate:              128000,
		DataBits:              8,
		StopBits:              1,
		InterCharacterTimeout: 200, // Waits at least 200ms for next character.
		MinimumReadSize:       2,   // Wait for at least 4 characters.
		ParityMode:            serial.PARITY_NONE,
	}
	ser, err := serial.Open(c)
	if err != nil {
		return fmt.Errorf("error opening %s", err)
	}

	l.ser = ser
	return nil
}

// StartScan starts up the scanning and data acquisition.
// see startScan for more details.
func (l *YDLidar) StartScan(pointCloud bool) {
	go l.startScan(pointCloud)
}

// StopScan stops the lidar scans.
func (l *YDLidar) StopScan() error {
	if err := l.ser.SetDTR(false); err != nil {
		return err
	}

	if _, err := l.ser.Write([]byte{START, CSTOPSCAN}); err != nil {
		return err
	}
	l.stop <- struct{}{}
	return nil

}

// sendErr sends error on channel.
func (l *YDLidar) sendErr(e error) {
	l.D <- DataPoint{
		Error: e,
	}
}

// startScan runs the data acquistion from the lidar. pointCloud controls id
// the result is sent as DataPoint (point cloud) or DistPacket through channel.

func (l *YDLidar) startScan(pointCloud bool) {

	if err := l.ser.SetDTR(true); err != nil {
		l.sendErr(fmt.Errorf("failed to set DTR :%v", err))
		return
	}

	if _, err := l.ser.Write([]byte{START, CSTARTSCAN}); err != nil {
		l.sendErr(fmt.Errorf("failed to start scan:%v", err))
		return
	}

	// Read and validate header.
	e, _, typ, mode := readHeader(l.ser)
	var err error
	switch {
	case e != nil:
		err = fmt.Errorf("read header failed:%v", err)

	case typ != SCAN_TYPE_CODE:
		err = fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", SCAN_TYPE_CODE, typ, mode)

	case mode != 1:
		err = fmt.Errorf("expected continuous mode")
	}
	if err != nil {
		l.sendErr(err)
		return
	}

	// Start loop to read distance samples.
	for {
		select {
		case <-l.stop:
			return
		default:
			// Read point cloud preamble header.
			data := make([]byte, 10)
			n, err := l.ser.Read(data)
			if byte(n) != 10 {
				l.sendErr(fmt.Errorf("not enough bytes. Expected %v got %v", 10, n))
				return
			}
			if err != nil {
				l.sendErr(fmt.Errorf("failed to read serial %v", 10, n))
				return
			}
			ptHdr := pointCloudHeader{}
			buf := bytes.NewBuffer(data)
			if err = binary.Read(buf, binary.LittleEndian, &ptHdr); err != nil {
				l.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				return
			}

			// Read distance data.
			data = make([]byte, ptHdr.Num*2)
			n, err = l.ser.Read(data)
			if err != nil {
				l.sendErr(fmt.Errorf("failed to read serial %v", 10, n))
				return
			}
			if n != int(ptHdr.Num*2) {
				l.sendErr(fmt.Errorf("not enough bytes. Expected %v got %v", ptHdr.Num*2, n))
				return
			}
			readings := make([]int16, ptHdr.Num)
			buf = bytes.NewBuffer(data)
			if err = binary.Read(buf, binary.LittleEndian, &readings); err != nil {
				l.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				return
			}
			// Convert readings to millimeters (divide by 4).
			distances := make([]float64, ptHdr.Num)
			for i := int8(0); i < ptHdr.Num; i++ {
				distances[i] = float64(readings[i]) / 4
			}

			// Calculate angles.
			angleCorFSA := angleCorrection(distances[0])
			angleFSA := float64(ptHdr.Fsa>>1)/64 + angleCorFSA

			angleCorLSA := angleCorrection(distances[ptHdr.Num-1])
			angleLSA := float64(ptHdr.Lsa>>1)/64 + angleCorLSA

			// Construct DistPacket and send if the option is set.
			if !pointCloud {
				l.Dp <- DistPacket{
					MinAngle:  angleLSA,
					MaxAngle:  angleLSA,
					Num:       int(ptHdr.Num),
					Distances: distances,
				}
				continue
			}

			var angleDelta float64

			switch {
			case angleLSA > angleFSA:
				angleDelta = angleLSA - angleFSA
			case angleLSA < angleFSA:
				angleDelta = 360 + angleLSA - angleFSA
			case angleLSA == angleFSA:
				angleDelta = 0
			}

			if ptHdr.Pkt == 1 {
				l.D <- DataPoint{
					Angle:  angleLSA,
					Dist:   distances[0],
					ZeroPt: true,
				}
				continue
			}

			for i := int8(0); i < ptHdr.Num; i++ {
				dist := distances[i]
				angleCor := angleCorrection(dist)
				angle := angleDelta/float64(ptHdr.Num-1)*float64(i) + angleFSA + angleCor
				l.D <- DataPoint{
					Angle: angle,
					Dist:  dist,
				}
			}
		}
	}

	return
}

// angleCorrection calculates the corrected angle for Lidar.
func angleCorrection(dist float64) float64 {
	if dist == 0 {
		return 0
	}
	return 180 / math.Pi * math.Atan(21.8*(155.3-dist)/(155.3*dist))
}

// Close will shutdown the connection.
func (l *YDLidar) Close() error {
	return l.ser.Close()
}

// Reboot soft reboots the lidar.
func (l *YDLidar) Reboot() error {
	if _, err := l.ser.Write([]byte{START, CRESTART}); err != nil {
		return err
	}
	return nil
}

// DeviceInfo returns the version information.
func (l *YDLidar) DeviceInfo() (error, *DeviceInfo) {
	if _, err := l.ser.Write([]byte{START, CINFO}); err != nil {
		return err, nil
	}

	err, sz, typ, mode := readHeader(l.ser)
	if err != nil {
		return err, nil
	}

	if typ != INFO_TYPE_CODE {
		return fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", STATUS_TYPE_CODE, typ, mode), nil
	}

	data := make([]byte, sz)
	n, err := l.ser.Read(data)

	if byte(n) != sz {
		return fmt.Errorf("not enough bytes. Expected %v got %v", sz, n), nil
	}
	if err != nil {
		return fmt.Errorf("failed to read serial:%v", err), nil
	}

	devInfo := &DeviceInfo{}

	devInfo.model = data[0]
	copy(devInfo.firmware[:], data[1:3])
	devInfo.hardware = data[3:4][0]
	copy(devInfo.serial[:], data[4:20])

	return nil, devInfo

}

// Status returns the lidar status. Returns nil if the lidar is operating optimally.
func (l *YDLidar) Status() error {

	if _, err := l.ser.Write([]byte{START, CSTATUS}); err != nil {
		return err
	}

	err, sz, typ, mode := readHeader(l.ser)
	if err != nil {
		return err
	}

	if typ != STATUS_TYPE_CODE {
		return fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", STATUS_TYPE_CODE, typ, mode)
	}

	data := make([]byte, sz)
	n, err := l.ser.Read(data)

	if byte(n) != sz {
		return fmt.Errorf("not enough bytes. Expected %v got %v", sz, n)
	}
	if err != nil {
		return fmt.Errorf("failed to read serial:%v", err)
	}
	if data[0] == 0x01 {
		return fmt.Errorf("device problem. Error Code:%x %x", data[1], data[2])
	}

	return nil
}

// readHeader reads the header portion of the response.
func readHeader(ser serial.SerialPort) (err error, sz byte, typ byte, mode byte) {
	header := make([]byte, 7)
	n, err := ser.Read(header)

	glog.V(2).Infof("Response header %v", header)

	if err != nil {
		return
	}

	if n != 7 {
		err = fmt.Errorf("not enough bytes reading header. Expected 7 bytes got %v", n)
		return
	}

	preamble := int(header[1])<<8 | int(header[0])

	if preamble != 0x5AA5 {
		err = fmt.Errorf("invalid header. Expected preamble 0x5AA5 got %x", preamble)
		return
	}

	sz = header[2]
	typ = header[6]
	mode = header[5] & 0xC0 >> 6

	glog.V(2).Infof("Got size:%v,type:%x,mode:%x", sz, typ, mode)

	return
}
