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
	cSTATUS    = 0x91
	cINFO      = 0x90
	cRESTART   = 0x40
	cSTOPSCAN  = 0x65
	cSTARTSCAN = 0x60

	//YDLIDAR Type Codes.
	STATUS_TYPE_CODE = 0x06
	INFO_TYPE_CODE   = 0x04
	SCAN_TYPE_CODE   = 0x81
)

// YDLidar is the lidar object.
type YDLidar struct {
	ser serial.SerialPort
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
	Header uint16 // The length is 2B, fixed at 0x55AA, low in front, high in back.
	Pkt    uint8  // Indicates the current packet type. 0x00: Point cloud packet 0x01: Zero packet.
	Num    uint8  // Indicates the number of sample points contained in the current packet.
	Fsa    uint16 // The angle data corresponding to the first sample point in the sampled data.
	Lsa    uint16 // The angle data corresponding to the last sample point in the sampled data.
	Chksum int16  //  Two-byte exclusive OR checksum value.

}

// NewLidar returns a YDLidar object.
func NewLidar() *YDLidar {
	return &YDLidar{}
}

// Init initializes the Lidar.
func (l *YDLidar) Init(ttyPort string) error {

	c := serial.OpenOptions{
		PortName:              ttyPort,
		BaudRate:              128000,
		DataBits:              8,
		StopBits:              1,
		InterCharacterTimeout: 0,
		MinimumReadSize:       1,
		ParityMode:            serial.PARITY_NONE,
	}
	ser, err := serial.Open(c)
	if err != nil {
		return fmt.Errorf("error opening %s", err)
	}

	l.ser = ser
	return nil
}

func (l *YDLidar) StartScan(c chan struct{}) error {

	if err := l.ser.SetDTR(true); err != nil {
		return err
	}

	if _, err := l.ser.Write([]byte{START, cSTARTSCAN}); err != nil {
		return err
	}

	err, _, typ, mode := readHeader(l.ser)
	if err != nil {
		return err
	}
	if typ != SCAN_TYPE_CODE {
		return fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", STATUS_TYPE_CODE, typ, mode)
	}
	if mode != 0x1 {
		return fmt.Errorf("expected continuous mode")
	}

	for {
		select {
		case <-c:
			return nil
		default:
			// Read point cloud preamble.
			data := make([]byte, 10)
			n, err := l.ser.Read(data)

			if byte(n) != 10 {
				return fmt.Errorf("not enough bytes. Expected %v got %v", 10, n)
			}
			if err != nil {
				return fmt.Errorf("failed to read serial:%v", err)
			}

			glog.V(2).Infof("Point cloud header: %v", data)

			ptHdr := pointCloudHeader{}
			buf := bytes.NewBuffer(data)
			if err = binary.Read(buf, binary.LittleEndian, &ptHdr); err != nil {
				return err
			}

			glog.V(2).Infof("%+v", ptHdr)

			// Read distance sample data.
			distSamples := make([]int16, ptHdr.Num)
			data = make([]byte, ptHdr.Num*2)
			n, err = l.ser.Read(data)

			fmt.Printf("%3.2f %3.2f \n", float64(ptHdr.Fsa>>1)/64, float64(ptHdr.Lsa>>1)/64)
			continue // TODO

			buf = bytes.NewBuffer(data)
			if err = binary.Read(buf, binary.LittleEndian, &distSamples); err != nil {
				return err
			}

			angleCorFSA := angleCorr(float64(distSamples[0]) / 4)
			angleFSA := float64(ptHdr.Fsa>>1)/64 + angleCorFSA

			angleCorLSA := angleCorr(float64(distSamples[ptHdr.Num-1]) / 4)
			angleLSA := float64(ptHdr.Lsa>>1)/64 + angleCorLSA

			angleDelta := angleLSA - angleFSA

			for i := uint8(0); i < ptHdr.Num; i++ {
				dist := float64(distSamples[i]) / 4
				angleCor := angleCorr(dist)
				angle := angleDelta/float64(ptHdr.Num-1)*float64(i) + angleFSA + angleCor
				//fmt.Printf("{D:%3.2f A:%3.2f} ", dist, angle)
				//fmt.Printf("%3.2f ", angle)
				_ = angle
			}
			fmt.Println()
		}
	}

	return nil
}

func angleCorr(dist float64) float64 {
	if dist == 0 {
		return 0
	}
	return 180 / math.Pi * math.Atan(21.8*(155.3-dist)/(155.3*dist))
}

// Close will shutdown the connection.
func (l *YDLidar) Close() {
	l.Close()
}

// Reboot soft reboots the lidar.
func (l *YDLidar) StopScan() error {

	if err := l.ser.SetDTR(false); err != nil {
		return err
	}

	if _, err := l.ser.Write([]byte{START, cSTOPSCAN}); err != nil {
		return err
	}
	return nil
}

// Reboot soft reboots the lidar.
func (l *YDLidar) Reboot() error {
	if _, err := l.ser.Write([]byte{START, cRESTART}); err != nil {
		return err
	}
	return nil
}

// DeviceInfo returns the version information.
func (l *YDLidar) DeviceInfo() (error, *DeviceInfo) {
	if _, err := l.ser.Write([]byte{START, cINFO}); err != nil {
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

	if _, err := l.ser.Write([]byte{START, cSTATUS}); err != nil {
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
		err = fmt.Errorf("invalid header. Expected preamble %x got %x %x", preamble, header[0], header[1])
		return
	}

	sz = header[2]
	typ = header[6]
	mode = header[5] & 0xC0 >> 6

	glog.V(2).Infof("Got size:%v,type:%x,mode:%x", sz, typ, mode)

	return
}
