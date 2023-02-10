// Package ydlidar this lidar outputs bytes in little endian format
package ydlidar

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"go.bug.st/serial"
	"log"
	"math"
	"os"
	"os/signal"
	"syscall"
	"time"
)

// NewLidar returns a YDLidar object.
func NewLidar(devicePort serial.Port) *YDLidar {
	return &YDLidar{
		SerialPort: devicePort,
		Packets:    make(chan Packet),
		Stop:       make(chan struct{}),
	}
}

func InitAndConnectToDevice(port *string) (*YDLidar, error) {
	var devicePort serial.Port
	var err error

	devicePort, err = GetSerialPort(port)
	if err != nil {
		return nil, err
	}

	err = devicePort.SetReadTimeout(1000 * time.Millisecond)
	if err != nil {
		return nil, err
	}

	lidar := NewLidar(devicePort)
	lidar.SetupCloseHandler()

	time.Sleep(time.Millisecond * 100)

	deviceInfo, err := lidar.DeviceInfo()
	if err != nil {
		return nil, err
	}
	log.Printf(*deviceInfo)

	healthStatus, err := lidar.HealthInfo()
	if err != nil {
		return nil, err
	}
	log.Printf(*healthStatus)

	return lidar, nil
}

// GetSerialPort returns a real serial port connection.
func GetSerialPort(ttyPort *string) (serial.Port, error) {

	// use ttyPort if not nil
	if ttyPort != nil {

		mode := &serial.Mode{
			BaudRate: 230400,          // 230400 baud
			DataBits: 8,               // 8 data bits
			Parity:   serial.NoParity, // No parity
			StopBits: 0,               // 0 == 1 stop bit
		}

		currentPort, err := serial.Open(*ttyPort, mode)
		if err != nil {
			return nil, err
		}

		//err = currentPort.SetDTR(true)
		//if err != nil {
		//	return nil, err
		//}

		log.Printf("Connected to port: %v", ttyPort)

		return currentPort, nil
	}
	// else iterate over ports to get the correct one
	ports, err := serial.GetPortsList()
	if err != nil {
		log.Panic(err)
	}

	if len(ports) == 0 {
		log.Panic(fmt.Errorf("no serial ports found"))
	}

	mode := &serial.Mode{
		BaudRate: 230400,          // 230400 baud
		DataBits: 8,               // 8 data bits
		Parity:   serial.NoParity, // No parity
		StopBits: 0,               // 0 == 1 stop bit
	}

	log.Print(ports)

	// use last port
	port := ports[len(ports)-1]
	log.Printf("Using port: %s", port)

	currentPort, err := serial.Open(port, mode)

	//for _, port := range ports {
	//	currentPort, err := serial.Open(port, mode)
	//	if err != nil {
	//		log.Print("Ignoring error: ", err)
	//	}
	//

	err = currentPort.SetDTR(true)
	if err != nil {
		return nil, err
	}

	log.Print("Connected to port: ", currentPort)

	return currentPort, nil

	log.Panic("Could not create a connection with the found serial ports!")

	return nil, nil
}

// SetupCloseHandler creates a 'listener' on a new goroutine which will notify the
// program if it receives an interrupt from the OS. We then handle this by calling
// our clean-up procedure and exiting the program.
func (lidar *YDLidar) SetupCloseHandler() {
	c := make(chan os.Signal)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-c
		log.Println("Ctrl+C pressed in Terminal")
		err := lidar.StopScan()
		if err != nil {
			return
		}

		err = lidar.Close()
		if err != nil {
			return
		}

		lidar.Reboot()

		os.Exit(0)

	}()
}

// SetDTR enables the DTR control for serial which controls the motor enable function.
func (lidar *YDLidar) SetDTR(s bool) {
	lidar.SerialPort.SetDTR(s)
}

// DeviceInfo returns the version information.
func (lidar *YDLidar) DeviceInfo() (*string, error) {
	if _, err := lidar.SerialPort.Write([]byte{preCommand, deviceInfo}); err != nil {
		return nil, err
	}

	sizeOfMessage, typeCode, mode, err := lidar.readInfoHeader()
	if err != nil {
		return nil, err
	}

	if typeCode != InfoTypeCode {
		return nil, fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", HealthTypeCode, typeCode, mode)
	}

	data := make([]byte, sizeOfMessage)
	n, err := lidar.SerialPort.Read(data)

	if byte(n) != sizeOfMessage {
		return nil, fmt.Errorf("device Info: not enough bytes. Expected %v got %v", sizeOfMessage, n)
	}
	if err != nil {
		return nil, fmt.Errorf("failed to read serial:%v", err)
	}

	deviceInfo := &DeviceInfo{}

	deviceInfo.Model = data[0]
	copy(deviceInfo.Firmware[:], data[1:3])
	deviceInfo.Hardware = data[3:4][0]
	copy(deviceInfo.Serial[:], data[4:20])

	stringDeviceInfo := &DeviceInfoString{}

	if deviceInfo.Model == 15 {
		stringDeviceInfo.Model = "G2"
		stringDeviceInfo.Firmware = fmt.Sprintf("%v.%v", deviceInfo.Firmware[0], deviceInfo.Firmware[1])
		stringDeviceInfo.Hardware = fmt.Sprintf("%v", deviceInfo.Hardware)
		stringDeviceInfo.Serial = string(deviceInfo.Serial[:])
		info := fmt.Sprintf("Device Info: Model: %v Hardware Version: %v Firmware Version: %v Serial Number: %v\n", stringDeviceInfo.Model, stringDeviceInfo.Hardware, stringDeviceInfo.Firmware, stringDeviceInfo.Serial)
		return &info, nil
	} else {
		return nil, fmt.Errorf("unknown model: %v", deviceInfo.Model)
	}

}

// HealthInfo returns the lidar status. Returns nil if the lidar is operating optimally.
func (lidar *YDLidar) HealthInfo() (*string, error) {

	if _, err := lidar.SerialPort.Write([]byte{preCommand, healthStatus}); err != nil {
		return nil, err
	}

	sizeOfMessage, typeCode, mode, err := lidar.readInfoHeader()
	if err != nil {
		return nil, err
	}

	if typeCode != HealthTypeCode {
		return nil, fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", HealthTypeCode, typeCode, mode)
	}

	data := make([]byte, sizeOfMessage)
	n, err := lidar.SerialPort.Read(data)

	if byte(n) != sizeOfMessage {
		return nil, fmt.Errorf("health Info: not enough bytes. Expected %v got %v", sizeOfMessage, n)
	}
	if err != nil {
		return nil, fmt.Errorf("failed to read serial:%v", err)
	}
	if data[0] == 0x01 {
		return nil, fmt.Errorf("device problem. Error Code:%x %x", data[1], data[2])
	}
	if data[0] == 0 {
		healthInfo := "Health Info: Device is operating optimally"
		return &healthInfo, nil
	}

	return nil, nil
}

// readInfoHeader reads and validate header response.
func (lidar *YDLidar) readInfoHeader() (sizeOfMessage byte, typeCode byte, mode byte, err error) {
	header := make([]byte, 7)
	numBytesInHeader, err := lidar.SerialPort.Read(header)

	if err != nil {
		return 0, 0, 0, err
	}

	if numBytesInHeader != 7 {

		err = fmt.Errorf("read Header: not enough bytes reading header. Expected 7 bytes got %v", numBytesInHeader)
		return 0, 0, 0, err
	}

	startSign := int(header[1])<<8 | int(header[0])
	if startSign != 0x5AA5 {
		return 0, 0, 0, fmt.Errorf("invalid header. Expected startSign 0x5AA5 got %x", startSign)
	}

	// sizeOfMessage is the lower 6 bits of the 6th byte
	sizeOfMessage = header[2]
	//log.Printf("size of message: %v", sizeOfMessage)

	typeCode = header[6]
	//log.Printf("type code: %v", typeCode)

	lower := header
	print("LOWER IS LOWER IS LOWER: ", lower, "\n")

	// the last number is the position of the byte eg. "...& 0xC0 >> 6", the 6 means the 6th byte
	mode = header[5] & 0xC0 >> 6
	//log.Printf("mode: %v", mode)

	return sizeOfMessage, typeCode, mode, nil
}

// StartScan starts up the scanning and data acquisition.
// see startScan for more details.
func (lidar *YDLidar) StartScan() {

	// Send start scanning command to device.
	if _, err := lidar.SerialPort.Write([]byte{preCommand, startScanning}); err != nil {
		lidar.sendErr(fmt.Errorf("failed to start scan: %v", err))
		return
	}

	//Read and validate initial scan header.
	_, typeCode, responseMode, err := lidar.readInfoHeader()
	switch {
	case err != nil:
		err = fmt.Errorf("read header failed: %v", err)

	case typeCode != ScanTypeCode: // 0x81
		err = fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", ScanTypeCode, typeCode, responseMode)

	case responseMode != ContinuousResponse: // 0x1
		err = fmt.Errorf("expected continuous response mode, got %v", responseMode)
	}
	if err != nil {
		lidar.sendErr(err)
		return
	}

	log.Print("Scan Command Response: GOOD")

	// Start loop to read distance samples.
	for {

		select {
		default:

			/////////////////////HEADER/////////////////////////////////////////////
			var numBytesScanPointCloud int
			scanPacketSize := 10

			// Read the scan packet initial header.
			scanPacketHeaderSlice := make([]byte, scanPacketSize)
			numBytesScanPointCloud, err := lidar.SerialPort.Read(scanPacketHeaderSlice)
			if byte(numBytesScanPointCloud) != 10 {
				log.Printf("not enough bytes in header. Expected %v got %v", scanPacketSize, numBytesScanPointCloud)
			}
			if err != nil {
				lidar.sendErr(fmt.Errorf("failed to read serial %v", err))
			}

			pointCloud := pointCloudHeader{}
			// Unpack the scan packet header into the pointCloudHeader struct.
			readingsBuffer := bytes.NewBuffer(scanPacketHeaderSlice)
			if err = binary.Read(readingsBuffer, binary.LittleEndian, &pointCloud); err != nil {
				lidar.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				continue
			}

			// Returns scan data in a human readable format.
			scanningFrequency, dataPacketType, sampleQuantity, startAngle, endAngle := lidar.extractScanPacket(pointCloud)

			switch dataPacketType {
			case 0x0:
				//There is only one zero point of data in the zero packet. The sampleQuantity is 1
				log.Printf("START DATA PACKET")
				if sampleQuantity != 1 {
					log.Printf("START DATA PACKET: sample quantity should be 1, got %v", sampleQuantity)
				}
				log.Printf("Scanning Frequency: %vHz", scanningFrequency)

			case 0x1:
				log.Printf("POINT CLOUD DATA PACKET")
				if sampleQuantity <= 0 {
					log.Printf("POINT CLOUD DATA PACKET: sample quantity should be greater than 0, got %v", sampleQuantity)
				}
				log.Print("Scanning Frequency is invalid in this packet")
			}

			/////////////////////////LUMINOSITY, DISTANCE, AND ANGLES/////////////////////////////////////
			sampleLength := int(sampleQuantity) * 3 // 3 bytes per sample
			rawContent := make([]byte, sampleLength)
			numBytesScanPointCloud, err = lidar.SerialPort.Read(rawContent)
			if err != nil {
				log.Print(fmt.Errorf("failed to read serial %v", err))
				continue
			}

			if numBytesScanPointCloud != sampleLength {
				log.Print(fmt.Errorf("Start Scan Sampling Quality: not enough bytes. Expected %v got %v", sampleLength, numBytesScanPointCloud))
				continue
			}

			// Unpack the rawContent into the readings slice.
			readings := make([]byte, sampleLength)
			readingsBuffer = bytes.NewBuffer(rawContent)
			if err = binary.Read(readingsBuffer, binary.LittleEndian, &readings); err != nil {
				log.Panic(fmt.Errorf("failed to pack struct: %v", err))
				continue
			}

			// Check Scan Packet Type.
			err = checkScanPacket(scanPacketHeaderSlice, rawContent)
			if err != nil {
				log.Printf(err.Error())
				continue
			}

			// TODO Create intensity conversion function.
			// TODO Create distance conversion function.
			// TODO Hoist conversions to separate function.
			//////////////////////////////////Intensity Calculations//////////////////////////////////
			//intensities := make([]float32, sampleLength)
			for i := uint8(0); i < pointCloud.SampleQuantity; i++ {
				log.Printf("readings[%v]: %v", i, readings[i])
			}

			//////////////////////////////Distance Calculations//////////////////////////////////
			// Convert readings to millimeters (divide by 4) and store in distances. // not right for G2
			distances := make([]float32, sampleLength)
			for i := uint8(0); i < pointCloud.SampleQuantity; i++ {
				log.Printf("readings[%v]: %v", i, readings[i])
			}
			/////////////////////////////////////////////////////////////////////////////////////////////////////////

			angleFSA, angleLSA, angleDelta := lidar.CalculateAngles(distances, startAngle, endAngle, sampleQuantity)

			lidar.Packets <- Packet{
				FirstAngle:         angleFSA,
				LastAngle:          angleLSA,
				DeltaAngle:         angleDelta,
				NumDistanceSamples: int(pointCloud.SampleQuantity),
				Distances:          distances,
				PacketType:         int(pointCloud.PackageType),
				Error:              err,
			}

		case <-lidar.Stop:
			log.Printf("STOPPING SCAN")
			return
		}

	}
}

func (lidar *YDLidar) extractScanPacket(pointCloud pointCloudHeader) (uint8, uint8, uint8, uint16, uint16) {
	// separate the least significant bit from the rest of the bits in pointCloudHeader.PackageType
	//packetHeader := pointCloud.PacketHeader

	scanningFrequency := ((pointCloud.PackageType >> 1) & 0x7F) / 10

	dataPacketType := pointCloud.PackageType & 0x01

	sampleQuantity := pointCloud.SampleQuantity

	startAngle := pointCloud.StartAngle
	endAngle := pointCloud.EndAngle

	//checkCode := pointCloud.CheckCode
	//if packetHeader != 0x55AA {
	//	log.Printf("Start Scan Packet Header: not enough bytes. Expected %v got %v", 0x55AA, packetHeader)
	//}

	return scanningFrequency, dataPacketType, sampleQuantity, startAngle, endAngle
}

// checkScanPacket validates the type of the packet.
// https://www.robotshop.com/media/files/content/y/ydl/pdf/ydlidar_x4_development_manual.pdf pg 6.
func checkScanPacket(scanPacketHeaderSlice []byte, scanPointCloudSlice []byte) error {

	// Make a slice big enough to hold scanPacketHeaderSlice (minus CRC) and scanPointCloudSlice.
	dataPacket := make([]uint16, 4+len(scanPointCloudSlice)/3)

	bufferedData := bytes.NewBuffer(append(scanPacketHeaderSlice[:8], scanPointCloudSlice...))

	// Read the scanPointCloudSlice packet into the bufferedData.
	err := binary.Read(bufferedData, binary.LittleEndian, &dataPacket)
	if err != nil {
		return fmt.Errorf("failed to pack struct: %v", err)
	}

	// Calculate Xor of all bits.
	var checkSum uint16
	for _, x := range dataPacket {

		checkSum ^= x
		// print out the XOR of all the bits and result
		log.Printf("XOR of all bits: %X", checkSum)
	}
	//
	//if byte(x) != byte(checkCode) {
	//	return fmt.Errorf("CRC failed. Ignoring scanPointCloudSlice packet\n\n\n")
	//}
	return nil
}

// GetPointCloud returns point-cloud (angle, dist) from the data packet.
func GetPointCloud(packet Packet) (pointClouds []PointCloudData) {
	// Zero Point packet.
	if packet.PacketType == 1 {
		pointClouds = append(pointClouds,
			PointCloudData{
				Angle: packet.FirstAngle,
				Dist:  packet.Distances[0],
			})
		return
	}

	for i, _ := range packet.Distances {
		dist := packet.Distances[i]
		angle := packet.DeltaAngle/float32(packet.NumDistanceSamples-1)*float32(i) + packet.FirstAngle + angleCorrection(dist)
		pointClouds = append(pointClouds,
			PointCloudData{
				Angle: angle,
				Dist:  dist,
			})
	}
	return
}

// angleCorrection calculates the corrected angle for Lidar.
func angleCorrection(dist float32) float32 {
	if dist == 0 {
		return 0
	}
	return float32(180 / math.Pi * math.Atan(21.8*(155.3-float64(dist))/(155.3*float64(dist))))
}

// StopScan stops the lidar scans, flushes the buffers and closes the serial port.
func (lidar *YDLidar) StopScan() error {
	log.Printf("Stopping scan")
	if _, err := lidar.SerialPort.Write([]byte{preCommand, stopScanning}); err != nil {
		return err
	}
	lidar.Stop <- struct{}{}
	lidar.SerialPort.ResetOutputBuffer()
	return nil

}

// sendErr sends error on channel with the packet.
func (lidar *YDLidar) sendErr(err error) {
	lidar.Packets <- Packet{
		Error: err,
	}
}

// Reboot soft reboots the lidar.
func (lidar *YDLidar) Reboot() error {
	if _, err := lidar.SerialPort.Write([]byte{preCommand, restartDevice}); err != nil {
		log.Print("Error sending reboot command: ", err)
		return err
	}
	return nil
}

// Close will shut down the connection.
func (lidar *YDLidar) Close() error {
	return lidar.SerialPort.Close()
}

// CalculateAngles will shut down the connection.
func (lidar *YDLidar) CalculateAngles(distances []float32, endAngle uint16, startAngle uint16, sampleQuantity uint8) (float32, float32, float32) {

	// Calculate angles.
	angleCorFSA := angleCorrection(distances[0])
	angleFSA := float32(startAngle>>1)/64 + angleCorFSA

	angleCorLSA := angleCorrection(distances[sampleQuantity-1])
	angleLSA := float32(endAngle>>1)/64 + angleCorLSA

	// Calculate angle delta.
	var angleDelta float32
	switch {
	case angleLSA > angleFSA:
		angleDelta = angleLSA - angleFSA
	case angleLSA < angleFSA:
		angleDelta = 360 + angleLSA - angleFSA
	case angleLSA == angleFSA:
		angleDelta = 0
	}

	return angleFSA, angleLSA, angleDelta

}
