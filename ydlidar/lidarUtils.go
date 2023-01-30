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

		err = currentPort.SetDTR(true)
		if err != nil {
			return nil, err
		}

		log.Print("Connected to port: ", *ttyPort)

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

	sizeOfMessage, typeCode, mode, err := readInfoHeader(lidar.SerialPort)
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

	sizeOfMessage, typeCode, mode, err := readInfoHeader(lidar.SerialPort)
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
func readInfoHeader(devicePort serial.Port) (sizeOfMessage byte, typeCode byte, mode byte, err error) {
	header := make([]byte, 7)
	numBytesInHeader, err := devicePort.Read(header)

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

	sizeOfMessage = header[2]
	//log.Printf("size of message: %v", sizeOfMessage)

	typeCode = header[6]
	//log.Printf("type code: %v", typeCode)

	mode = header[5] & 0xC0 >> 6
	//log.Printf("mode: %v", mode)

	return sizeOfMessage, typeCode, mode, nil
}

// StartScan starts up the scanning and data acquisition.
// see startScan for more details.
func (lidar *YDLidar) StartScan() {
	continuousResponse := byte(1)

	// Send start scanning command to device.
	log.Printf("starting scan")
	if _, err := lidar.SerialPort.Write([]byte{preCommand, startScanning}); err != nil {
		lidar.sendErr(fmt.Errorf("failed to start scan: %v", err))
		return
	}

	// Read and validate initial scan header.
	log.Printf("reading scan header")
	sizeOfMessage, typeCode, responseMode, err := readInfoHeader(lidar.SerialPort)
	switch {
	case err != nil:
		err = fmt.Errorf("read header failed: %v", err)

	case typeCode != ScanTypeCode:
		err = fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", ScanTypeCode, typeCode, responseMode)

	case responseMode != continuousResponse:
		err = fmt.Errorf("expected continuous response mode, got %v", responseMode)

	case sizeOfMessage != 5:
		err = fmt.Errorf("invalid size of header message. Expected %v, got %v", 5, sizeOfMessage)
	}
	if err != nil {
		lidar.sendErr(err)
		return
	}

	log.Printf("Inital Header Message Size: %v", sizeOfMessage)

	// Start loop to read distance samples.
	for {
		var numBytesScanPacketHeader int

		// Read the scan packet initial header.
		log.Printf("reading scan packet header")
		scanPacketHeaderSlice := make([]byte, 10)
		numBytesScanPacketHeader, err = lidar.SerialPort.Read(scanPacketHeaderSlice)
		if byte(numBytesScanPacketHeader) != 10 {
			lidar.sendErr(fmt.Errorf("start Scan: not enough bytes in header. Expected %v got %v", 10, numBytesScanPacketHeader))
			continue
		}
		if err != nil {
			lidar.sendErr(fmt.Errorf("failed to read serial %v", err))
			continue
		}

		pointCloudHeader := pointCloudHeader{}

		scanPacketHeaderBuffer := bytes.NewBuffer(scanPacketHeaderSlice)
		if err = binary.Read(scanPacketHeaderBuffer, binary.LittleEndian, &pointCloudHeader); err != nil {
			lidar.sendErr(fmt.Errorf("failed to pack struct: %v", err))
			continue
		}

		testingBytes := int(pointCloudHeader.FrequencyAndPackageType)

		if testingBytes == 5 {
			log.Printf("Should see this every scan")
			log.Printf("POINT CLOUD FREQUENCY AND PACKAGE TYPE: %v", pointCloudHeader.FrequencyAndPackageType)
		}

		// Read distance data.
		data := make([]byte, int(pointCloudHeader.NumberOfPoints)*2)
		numBytesScanPacketHeader, err = lidar.SerialPort.Read(data)
		if err != nil {
			lidar.sendErr(fmt.Errorf("failed to read serial %v", err))
			continue
		}

		if numBytesScanPacketHeader != int(pointCloudHeader.NumberOfPoints*2) {
			lidar.sendErr(fmt.Errorf("start Scan Sampling Quality: not enough bytes. Expected %v got %v", pointCloudHeader.NumberOfPoints*2, numBytesScanPacketHeader))
			continue
		}

		// Make a slice to hold the distanceReadings.
		distanceReadings := make([]int16, pointCloudHeader.NumberOfPoints)

		scanPacketHeaderSlice = bytes.NewBuffer(data)
		if err = binary.Read(scanPacketHeaderSlice, binary.LittleEndian, &distanceReadings); err != nil {
			lidar.sendErr(fmt.Errorf("failed to pack struct: %v", err))
			continue
		}

		// Check Scan Packet Type.
		err = checkScanPacket(scanPacketHeaderBuffer, data, pointCloudHeader.CheckCode)
		if err != nil {
			log.Printf(err.Error())
			continue
		}

		// TODO Hoist conversions to separate function.
		// Check for sane number of packets.
		if pointCloudHeader.NumberOfPoints <= 0 {
			continue
		}

		// Convert distanceReadings to millimeters (divide by 4).
		distances := make([]float32, pointCloudHeader.NumberOfPoints) // NUMBER POINTS IS INCORRECT ASSIGNMENT
		for i := uint8(0); i < pointCloudHeader.NumberOfPoints; i++ {
			distances[i] = float32(distanceReadings[i]) / 4
		}

		// Calculate angles.
		angleCorFSA := angleCorrection(distances[0])
		angleFSA := float32(pointCloudHeader.StartAngle>>1)/64 + angleCorFSA

		angleCorLSA := angleCorrection(distances[pointCloudHeader.NumberOfPoints-1])
		angleLSA := float32(pointCloudHeader.EndAngle>>1)/64 + angleCorLSA

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

		log.Printf("READINGS: %x", distanceReadings)

		log.Printf("BUFFER: %v", scanPacketHeaderSlice)

		startCode := scanPacketHeaderBuffer[0:2]
		// convert startCode to readable text
		log.Printf("Hex Start Code: %X", binary.LittleEndian.Uint16(startCode))

		ScanOrFreq := scanPacketHeaderBuffer[2:3]
		// convert ScanOrFreq to readable text
		log.Printf("Hex Scan or Freq: %08b", ScanOrFreq)

		FreqOrScan := scanPacketHeaderBuffer[3:4]
		// convert FreqOrScan to readable text
		log.Printf("Hex Freq or Scan: %X", FreqOrScan)

		startAngle := scanPacketHeaderBuffer[4:6]
		// convert startAngle to readable text

		log.Printf("Hex Start Angle: %X", startAngle)

		endAngle := scanPacketHeaderBuffer[6:8]
		// convert endAngle to readable text
		log.Printf("Hex End Angle: %X", endAngle)

		CheckCode := scanPacketHeaderBuffer[8:10]
		// convert CheckCode to readable text
		log.Printf("Hex Check Code: %X", CheckCode)

		log.Printf("Number of Samples: %v\n", int(pointCloudHeader.NumberOfPoints))
		log.Printf("Packet Header: %x\n", pointCloudHeader.PacketHeader)
		log.Printf("Distances: %v\n", distances)
		log.Printf("Angle Delta: %v\n", angleDelta)

		lidar.Packets <- Packet{
			MinAngle:           angleFSA,
			MaxAngle:           angleLSA,
			DeltaAngle:         angleDelta,
			NumDistanceSamples: int(pointCloudHeader.NumberOfPoints),
			Distances:          distances,
			PacketType:         uint16(int(pointCloudHeader.FrequencyAndPackageType)),
			Error:              err,
		}

	}
}

// checkScanPacket validates the type of the packet.
// https://www.robotshop.com/media/files/content/y/ydl/pdf/ydlidar_x4_development_manual.pdf pg 6.
func checkScanPacket(header []byte, data []byte, checkCode uint16) error {
	log.Printf("CHECKING SCAN PACKET with CHECKCODE: %x\n\n\n", checkCode)

	// Make a slice big enough to hold header (minus CRC) and data.
	dataPacket := make([]uint16, 4+len(data)/2)

	bufferedData := bytes.NewBuffer(append(header[:8], data...))

	// Read the data packet into the bufferedData.
	err := binary.Read(bufferedData, binary.LittleEndian, &dataPacket)
	if err != nil {
		return fmt.Errorf("failed to pack struct: %v", err)
	}
	mode := header[5] & 0xC0 >> 6

	log.Printf("MODE: %x", mode)
	log.Printf("HEADER: %x", header[:8]) // Header is the first 8 bytes.
	log.Printf("SAMPLE QUANTITY: %x", header[3:4])
	log.Printf("DATA PACKET: %x\n\n\n", dataPacket)

	// Calculate Xor of all bits.
	x := uint16(0)
	for i := 0; i < len(dataPacket); i++ {
		x ^= dataPacket[i]
	}

	if byte(x) != byte(checkCode) {
		return fmt.Errorf("CRC failed. Ignoring data packet")
	}
	return nil
}

// GetPointCloud returns point-cloud (angle, dist) from the data packet.
func GetPointCloud(packet Packet) (pointClouds []PointCloud) {
	// Zero Point packet.
	if packet.PacketType == 1 {
		pointClouds = append(pointClouds,
			PointCloud{
				Angle: packet.MinAngle,
				Dist:  packet.Distances[0],
			})
		return
	}

	for i, _ := range packet.Distances {
		dist := packet.Distances[i]
		angle := packet.DeltaAngle/float32(packet.NumDistanceSamples-1)*float32(i) + packet.MinAngle + angleCorrection(dist)
		pointClouds = append(pointClouds,
			PointCloud{
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

// StopScan TODO: Need to flush the serial buffer when done.
// StopScan stops the lidar scans.
func (lidar *YDLidar) StopScan() error {
	log.Printf("Stopping scan")
	if _, err := lidar.SerialPort.Write([]byte{preCommand, stopScanning}); err != nil {
		return err
	}
	lidar.Stop <- struct{}{}
	//lidar.SerialPort.ResetOutputBuffer()
	//lidar.SerialPort.ResetInputBuffer()
	err := lidar.SerialPort.Close()
	if err != nil {
		return err
	}
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
		return err
	}
	return nil
}

// Close will shut down the connection.
func (lidar *YDLidar) Close() error {
	return lidar.SerialPort.Close()
}
