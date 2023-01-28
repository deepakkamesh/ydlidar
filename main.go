/*
* Package ydlidar provides go api for YDLidar G2.
* https://www.robotshop.com/media/files/content/y/ydl/pdf/ydlidar_x4_development_manual.pdf
 */
package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"github.com/deepakkamesh/ydlidar"
	"log"
	"math"
	"time"

	"go.bug.st/serial"
)

const (
	// YDLIDAR Commands.
	START          = 0xA5
	CSTATUS    int = 0x92
	CINFO          = 0x90
	CRESTART       = 0x40
	CSTOPSCAN      = 0x65
	CSTARTSCAN     = 0x60

	//YDLIDAR Type Codes.
	STATUS_TYPE_CODE = 0x06
	INFO_TYPE_CODE   = 0x04
	SCAN_TYPE_CODE   = 0x81
)

// YDLidar is the lidar object.
type YDLidar struct {
	ser  serial.Port
	D    chan Packet
	stop chan struct{}
}

// PointCloud represents a single lidar reading.
type PointCloud struct {
	Angle float32
	Dist  float32
}

// Packet represents struct of a single sample set of readings.
type Packet struct {
	MinAngle   float32   // Minimum angle corresponds to first distance sample.
	MaxAngle   float32   // Max angle corresponds to last distance sample.
	DeltaAngle float32   // Delta between Min and Max Angles.
	Num        int       // Number of distance samples.
	Distances  []float32 // Slice containing distance data.
	PktType    int       // Indicates the current packet type. 0x00: Point cloud packet 0x01: Zero packet.
	Error      error
}

// DeviceInfo struct contains the version information.
type DeviceInfo struct {
	Model       byte     // Model number.
	FirmwareVer [2]byte  // firmware version.
	HardwareVer byte     // hardware version.
	SerialNo    [16]byte // serial number.
}

// pointCloudHeader is the preamble for the point cloud data.
type pointCloudHeader struct {
	Header uint16 // The length is 2B, fixed at 0x55AA, low in front, high in back.
	Pkt    uint8  // Indicates the current packet type. 0x00: Point cloud packet 0x01: Zero packet.
	Num    uint8  // Indicates the number of sample points contained in the current packet.
	Fsa    uint16 // The angle data corresponding to the first sample point in the sampled data.
	Lsa    uint16 // The angle data corresponding to the last sample point in the sampled data.
	Chksum uint16 //  Two-byte exclusive OR checksum value.

}

// NewLidar returns a YDLidar object.
func NewLidar() *YDLidar {
	return &YDLidar{
		D:    make(chan Packet),
		stop: make(chan struct{}),
	}
}

// GetSerialPort returns a real serial port connection.
func GetSerialPort(ttyPort *string) (serial.Port, error) {

	ports, err := serial.GetPortsList()
	if err != nil {
		log.Fatal(err)
	}

	if len(ports) == 0 {
		log.Fatal("No serial ports found!")
	}

	for _, port := range ports {
		log.Printf("Found port: %v\n", port)
		mode := &serial.Mode{
			BaudRate: 230400,
			DataBits: 8,
			Parity:   serial.NoParity,
			StopBits: 0,
		}

		currentPort, err := serial.Open(port, mode)
		if err != nil {
			log.Printf("Failed to connect to port %v. Error: %v", port, err)
			continue
		}
		currentPort.SetDTR(true)
		log.Print("Connected to port: ", port)

		return currentPort, nil
	}

	log.Panic("Could not create a connection with the found serial ports!")

	return nil, nil
}

// SetSerial used to set serial.
func (l *YDLidar) SetSerial(s serial.Port) {
	l.ser = s
}

// StartScan starts up the scanning and data acquisition.
// see startScan for more details.
func (l *YDLidar) StartScan() {
	go l.startScan()
}

// TODO: Need to flush the serial buffer when done.
// StopScan stops the lidar scans.
func (l *YDLidar) StopScan() error {

	if _, err := l.ser.Write([]byte{START, CSTOPSCAN}); err != nil {
		return err
	}
	l.stop <- struct{}{}
	return nil

}

// sendErr sends error on channel.
func (l *YDLidar) sendErr(e error) {
	l.D <- Packet{
		Error: e,
	}
}

// SetDTR enables the DTR control for serial which controls the motor enable function.
func (l *YDLidar) SetDTR(s bool) error {
	return l.ser.SetDTR(s)
}

// startScan runs the data acquistion from the lidar.
func (l *YDLidar) startScan() {

	if _, err := l.ser.Write([]byte{START, CSTARTSCAN}); err != nil {
		l.sendErr(fmt.Errorf("failed to start scan:%v", err))
		return
	}

	// Read and validate header.
	e, _, typ, mode := readHeader(l.ser)
	var err error
	switch {
	case e != nil:
		err = fmt.Errorf("read header failed: %v", e)

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
			header := make([]byte, 10)
			n, err := l.ser.Read(header)
			if byte(n) != 10 {
				l.sendErr(fmt.Errorf("not enough bytes. Expected %v got %v", 10, n))
				continue
			}
			if err != nil {
				l.sendErr(fmt.Errorf("failed to read serial %v", err))
				continue
			}
			ptHdr := pointCloudHeader{}
			buf := bytes.NewBuffer(header)
			if err = binary.Read(buf, binary.LittleEndian, &ptHdr); err != nil {
				l.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				continue
			}

			// Read distance data.
			data := make([]byte, int(ptHdr.Num)*2)
			n, err = l.ser.Read(data)
			if err != nil {
				l.sendErr(fmt.Errorf("failed to read serial %v", err))
				continue
			}
			if n != int(ptHdr.Num*2) {
				l.sendErr(fmt.Errorf("not enough bytes. Expected %v got %v", ptHdr.Num*2, n))
				continue
			}
			readings := make([]int16, ptHdr.Num)
			buf = bytes.NewBuffer(data)
			if err = binary.Read(buf, binary.LittleEndian, &readings); err != nil {
				l.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				continue
			}

			// Check CRC of the packet.
			err = checkCRC(header, data, ptHdr.Chksum)
			if err != nil {
				log.Printf(err.Error())
				continue
			}
			// Check for sane number of packets.
			if ptHdr.Num <= 0 {
				continue
			}

			// Convert readings to millimeters (divide by 4).
			distances := make([]float32, ptHdr.Num)
			for i := uint8(0); i < ptHdr.Num; i++ {
				distances[i] = float32(readings[i]) / 4
			}

			// Calculate angles.
			angleCorFSA := angleCorrection(distances[0])
			angleFSA := float32(ptHdr.Fsa>>1)/64 + angleCorFSA

			angleCorLSA := angleCorrection(distances[ptHdr.Num-1])
			angleLSA := float32(ptHdr.Lsa>>1)/64 + angleCorLSA

			var angleDelta float32

			switch {
			case angleLSA > angleFSA:
				angleDelta = angleLSA - angleFSA
			case angleLSA < angleFSA:
				angleDelta = 360 + angleLSA - angleFSA
			case angleLSA == angleFSA:
				angleDelta = 0
			}

			l.D <- Packet{
				MinAngle:   angleFSA,
				MaxAngle:   angleLSA,
				Num:        int(ptHdr.Num),
				Distances:  distances,
				DeltaAngle: angleDelta,
				PktType:    int(ptHdr.Pkt),
			}
		}
	}
}

// GetPointCloud returns pointcloud (angle, dist) from the data packet.
func GetPointCloud(d Packet) (pt []PointCloud) {
	// Zero Point packet.
	if d.PktType == 1 {
		pt = append(pt,
			PointCloud{
				Angle: d.MinAngle,
				Dist:  d.Distances[0],
			})
		return
	}

	for i := 0; i < d.Num; i++ {
		dist := d.Distances[i]
		angle := d.DeltaAngle/float32(d.Num-1)*float32(i) + d.MinAngle + angleCorrection(dist)
		pt = append(pt,
			PointCloud{
				Angle: angle,
				Dist:  dist,
			})
	}
	return
}

// checkCRC validates the CRC of the packet.
// https://www.robotshop.com/media/files/content/y/ydl/pdf/ydlidar_x4_development_manual.pdf pg 6.
func checkCRC(header []byte, data []byte, crc uint16) error {

	// Make a 16bit slice big enough to hold header (minus CRC) and data.
	dataPkt := make([]uint16, 4+len(data)/2)

	buf := bytes.NewBuffer(append(header[:8], data...))
	if err := binary.Read(buf, binary.LittleEndian, &dataPkt); err != nil {
		return fmt.Errorf("failed to pack struct: %v", err)
	}

	// Calculate Xor of all bits.
	x := uint16(0)
	for i := 0; i < len(dataPkt); i++ {
		x ^= dataPkt[i]
	}
	if x != crc {
		return fmt.Errorf("CRC failed. Ignoring data packet")
	}
	return nil
}

// angleCorrection calculates the corrected angle for Lidar.
func angleCorrection(dist float32) float32 {
	if dist == 0 {
		return 0
	}
	return float32(180 / math.Pi * math.Atan(21.8*(155.3-float64(dist))/(155.3*float64(dist))))
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

	devInfo.Model = data[0]
	copy(devInfo.FirmwareVer[:], data[1:3])
	devInfo.HardwareVer = data[3:4][0]
	copy(devInfo.SerialNo[:], data[4:20])

	return nil, devInfo

}

// Status returns the lidar status. Returns nil if the lidar is operating optimally.
func (l *YDLidar) Status() error {

	if _, err := l.ser.Write([]byte{START}); err != nil {
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
func readHeader(ser serial.Port) (err error, sz byte, typ byte, mode byte) {
	header := make([]byte, 7)
	n, err := ser.Read(header)

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

	return
}

func main() {
	//log.Print("**************************")

	devicePort, err := GetSerialPort(nil)
	if err != nil {
		log.Fatal(err)
	}

	lidar := ydlidar.NewLidar()

	devicePort.SetReadTimeout(1000 * time.Millisecond)

	lidar.SetSerial(devicePort)
	if err := lidar.SetDTR(true); err != nil {
		panic(fmt.Sprintf("failed to set DTR:%v", err))
	}
	lidar.StartScan()

	log.Print("Using port: ", devicePort)
	//buff := make([]byte, 100)
	//for {
	//	n, err := devicePort.Read(buff)
	//	if err != nil {
	//		log.Fatal(err)
	//		break
	//	}
	//	if n == 0 {
	//		fmt.Println("\nEOF")
	//		break
	//	}
	//	fmt.Printf("%v", string(buff[:n]))
	//}
}
