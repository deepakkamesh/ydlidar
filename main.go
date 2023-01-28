/*
* Package ydlidar provides go api for YDLidar G2.
* https://www.robotshop.com/media/files/content/y/ydl/pdf/ydlidar_x4_development_manual.pdf
 */
package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"time"

	"go.bug.st/serial"
)

const (
	// YDLIDAR Commands.
	PRECOMMAND    = 0xA5 // verified
	HEALTHSTATUS  = 0x92 // verified
	DEVICEINFO    = 0x90 // verified
	RESTARTDEVICE = 0x40 // verified
	STOPSCANNING  = 0x65 // verified
	STARTSCANNING = 0x60 // verified

	//YDLIDAR Type Codes.
	STATUS_TYPE_CODE = 0x06
	INFO_TYPE_CODE   = 0x04
	SCAN_TYPE_CODE   = 0x81
)

// YDLidar is the lidar object.
type YDLidar struct {
	SerialPort serial.Port
	D          chan Packet
	Stop       chan struct{}
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

// DeviceInfo Works with G2
// DeviceInfo contains the device model, firmware, hardware, and serial number.
type DeviceInfo struct {
	Model    byte     // Model number.
	Firmware [2]byte  // Firmware version.
	Hardware byte     // Hardware version.
	Serial   [16]byte // Serial number.
}

// pointCloudHeader is the preamble for the point cloud data.
type pointCloudHeader struct {
	// PacketHeader 2B in length, fixed at 0x55AA, low in front, high in back
	PacketHeader uint16 // PH(2B)

	// FrequencyAndPackageType F(bit7:1): represents the scanning frequency of the lidar at the current moment,
	// the value is valid in the initial data packet, and the value is 0 by default in the
	// point cloud data packet; C(bit0): represents the type of the current data packet;
	// 0x00: Point cloud data package 0x01: Start data package
	FrequencyAndPackageType uint8 // F&C (1B)

	// SamplingQuality Indicates the number of sampling points contained in the current packet. There is only one zero point of data in the zero packet. The value is 1.
	SamplingQuantity uint8 // LSN(1B)

	// StartAngle The angle data corresponding to the first sample point in the sampled data
	StartAngle uint16 // FSA(2B)

	// EndAngle The angle data corresponding to the last sample point in the sampled data
	EndAngle uint16 // LSA(2B)

	// CheckCode The check code of the current data packet uses a two-byte exclusive OR to check the current data packet.
	CheckCode uint16 // CS(2B

	// Don't need to parse this data in the header.
	//// SampleData of the system test is the distance data of the sampling point,
	//// and the interference flag is also integrated in the LSB of the Si node
	//SampleData [3]byte // Si(3B)

}

// NewLidar returns a YDLidar object.
func NewLidar() *YDLidar {
	return &YDLidar{
		D:    make(chan Packet),
		Stop: make(chan struct{}),
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
		mode := &serial.Mode{
			BaudRate: 230400,
			DataBits: 8,
			Parity:   serial.NoParity,
			StopBits: 0,
		}

		currentPort, err := serial.Open(port, mode)
		if err != nil {
			continue
		}

		err = currentPort.SetDTR(true)
		if err != nil {
			return nil, err
		}

		log.Print("Connected to port: ", port)

		return currentPort, nil
	}

	log.Panic("Could not create a connection with the found serial ports!")

	return nil, nil
}

// SetSerial used to set serial.
func (lidar *YDLidar) SetSerial(s serial.Port) {
	lidar.SerialPort = s
}

// StartScan starts up the scanning and data acquisition.
// see startScan for more details.
func (lidar *YDLidar) StartScan() {
	go lidar.startScan()
}

// StopScan TODO: Need to flush the serial buffer when done.
// StopScan stops the lidar scans.
func (lidar *YDLidar) StopScan() error {

	if _, err := lidar.SerialPort.Write([]byte{PRECOMMAND, STOPSCANNING}); err != nil {
		return err
	}
	lidar.Stop <- struct{}{}
	return nil

}

// sendErr sends error on channel.
func (lidar *YDLidar) sendErr(e error) {
	lidar.D <- Packet{
		Error: e,
	}
}

// SetDTR enables the DTR control for serial which controls the motor enable function.
func (lidar *YDLidar) SetDTR(s bool) error {
	return lidar.SerialPort.SetDTR(s)
}

// startScan runs the data acquistion from the lidar.
func (lidar *YDLidar) startScan() {

	if _, err := lidar.SerialPort.Write([]byte{PRECOMMAND, STARTSCANNING}); err != nil {
		lidar.sendErr(fmt.Errorf("failed to start scan:%v", err))
		return
	}

	// Read and validate header.
	e, _, typ, mode := readHeader(lidar.SerialPort)
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
		lidar.sendErr(err)
		return
	}

	// Start loop to read distance samples.
	for {
		select {
		case <-lidar.Stop:
			return
		default:
			// Read point cloud preamble header.
			header := make([]byte, 10)
			n, err := lidar.SerialPort.Read(header)
			log.Print(n, err)
			if byte(n) != 0 {
				err := lidar.StopScan()
				if err != nil {
					log.Printf("failed to Stop scan: %v", err)
				}

				lidar.sendErr(fmt.Errorf("not enough bytes. Expected %v got %v", 10, n))
				continue
			}
			if err != nil {
				lidar.sendErr(fmt.Errorf("failed to read serial %v", err))
				continue
			}
			pointCloudHeader := pointCloudHeader{}
			buf := bytes.NewBuffer(header)
			if err = binary.Read(buf, binary.LittleEndian, &pointCloudHeader); err != nil {
				lidar.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				continue
			}

			// Read distance data.
			data := make([]byte, int(pointCloudHeader.SamplingQuantity)*2)
			n, err = lidar.SerialPort.Read(data)
			if err != nil {
				lidar.sendErr(fmt.Errorf("failed to read serial %v", err))
				continue
			}
			if n != int(pointCloudHeader.SamplingQuantity*2) {
				lidar.sendErr(fmt.Errorf("not enough bytes. Expected %v got %v", pointCloudHeader.SamplingQuantity*2, n))
				continue
			}
			readings := make([]int16, pointCloudHeader.SamplingQuantity)
			buf = bytes.NewBuffer(data)
			if err = binary.Read(buf, binary.LittleEndian, &readings); err != nil {
				lidar.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				continue
			}

			// Check CRC of the packet.
			err = checkCRC(header, data, pointCloudHeader.CheckCode)
			if err != nil {
				log.Printf(err.Error())
				continue
			}
			// Check for sane number of packets.
			if pointCloudHeader.SamplingQuantity <= 0 {
				continue
			}

			// Convert readings to millimeters (divide by 4).
			distances := make([]float32, pointCloudHeader.SamplingQuantity)
			for i := uint8(0); i < pointCloudHeader.SamplingQuantity; i++ {
				distances[i] = float32(readings[i]) / 4
			}

			// Calculate angles.
			angleCorFSA := angleCorrection(distances[0])
			angleFSA := float32(pointCloudHeader.StartAngle>>1)/64 + angleCorFSA

			angleCorLSA := angleCorrection(distances[pointCloudHeader.SamplingQuantity-1])
			angleLSA := float32(pointCloudHeader.EndAngle>>1)/64 + angleCorLSA

			var angleDelta float32

			switch {
			case angleLSA > angleFSA:
				angleDelta = angleLSA - angleFSA
			case angleLSA < angleFSA:
				angleDelta = 360 + angleLSA - angleFSA
			case angleLSA == angleFSA:
				angleDelta = 0
			}

			lidar.D <- Packet{
				MinAngle:   angleFSA,
				MaxAngle:   angleLSA,
				Num:        int(pointCloudHeader.SamplingQuantity),
				Distances:  distances,
				DeltaAngle: angleDelta,
				PktType:    int(pointCloudHeader.PacketHeader),
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
func (lidar *YDLidar) Close() error {
	return lidar.SerialPort.Close()
}

// Reboot soft reboots the lidar.
func (lidar *YDLidar) Reboot() error {
	if _, err := lidar.SerialPort.Write([]byte{PRECOMMAND, RESTARTDEVICE}); err != nil {
		return err
	}
	return nil
}

// DeviceInfo returns the version information.
func (lidar *YDLidar) DeviceInfo() (*DeviceInfo, error) {
	if _, err := lidar.SerialPort.Write([]byte{PRECOMMAND, DEVICEINFO}); err != nil {
		return nil, err
	}

	err, sz, typ, mode := readHeader(lidar.SerialPort)
	if err != nil {
		return nil, err
	}

	if typ != INFO_TYPE_CODE {
		return nil, fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", STATUS_TYPE_CODE, typ, mode)
	}

	data := make([]byte, sz)
	n, err := lidar.SerialPort.Read(data)

	stringData := bytes.NewBuffer(data).String()
	log.Printf("Device Info: %v", stringData)

	if byte(n) != sz {
		return nil, fmt.Errorf("not enough bytes. Expected %v got %v", sz, n)
	}
	if err != nil {
		return nil, fmt.Errorf("failed to read serial:%v", err)
	}

	devInfo := &DeviceInfo{}

	devInfo.Model = data[0]
	copy(devInfo.Firmware[:], data[1:3])
	devInfo.Hardware = data[3:4][0]
	copy(devInfo.Serial[:], data[4:20])

	return devInfo, nil

}

// Status returns the lidar status. Returns nil if the lidar is operating optimally.
func (lidar *YDLidar) Status() error {

	if _, err := lidar.SerialPort.Write([]byte{PRECOMMAND, HEALTHSTATUS}); err != nil {
		return err
	}

	err, sz, typ, mode := readHeader(lidar.SerialPort)
	if err != nil {
		return err
	}

	if typ != STATUS_TYPE_CODE {
		return fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", STATUS_TYPE_CODE, typ, mode)
	}

	data := make([]byte, sz)
	n, err := lidar.SerialPort.Read(data)

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

	lidar := NewLidar()

	devicePort.SetReadTimeout(1000 * time.Millisecond)

	lidar.SetSerial(devicePort)
	if err := lidar.SetDTR(true); err != nil {
		panic(fmt.Sprintf("failed to set DTR:%v", err))
	}

	deviceInfo, err := lidar.DeviceInfo()
	if err != nil {
		log.Fatal(err)
	}
	log.Print(deviceInfo.Model, "\n", deviceInfo.Firmware, "\n", deviceInfo.Hardware, "\n", deviceInfo.Serial)

	lidar.StartScan()
}
