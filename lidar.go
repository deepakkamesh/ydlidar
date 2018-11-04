/*
* Package ydlidar provides go api for YDLidar X4.
*
 */
package ydlidar

import (
	"fmt"

	"github.com/deepakkamesh/go-serial/serial"
	"github.com/golang/glog"
)

const (
	// YDLIDAR Commands.
	START   = 0xA5
	cSTATUS = 0x91
	cINFO   = 0x90

	//YDLIDAR Type Codes.
	STATUS_TYPE_CODE = 0x06
	INFO_TYPE_CODE   = 0x04
)

type DeviceInfo struct {
	model    byte     // Model number.
	firmware [2]byte  // firmware version.
	hardware byte     // hardware version.
	serial   [16]byte // serial number.
}

type YDLidar struct {
	ser serial.SerialPort
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

// Close will shutdown the connection.
func (l *YDLidar) Close() {
	l.Close()
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

	if header[0] != 0xA5 || header[1] != 0x5A {
		err = fmt.Errorf("invalid header. Expected 0xA5 0x5A got %x %x", header[0], header[1])
		return
	}

	sz = header[2]
	typ = header[6]
	mode = header[5] & 0x3

	glog.V(2).Infof("Got size:%v,type:%x,mode:%x", sz, typ, mode)

	return
}
