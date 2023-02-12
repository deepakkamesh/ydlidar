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

var scanPacketHeaderSize = 10

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

		lidar.SerialPort.ResetOutputBuffer()

		err = lidar.Close()
		if err != nil {
			return
		}

		os.Exit(0)

	}()
}

// SetDTR enables the DTR control for serial which controls the motor enable function.
func (lidar *YDLidar) SetDTR(s bool) {
	err := lidar.SerialPort.SetDTR(s)
	if err != nil {
		return
	}
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
		info := fmt.Sprintf("Device Info: Model: %v Hardware Version: %v Firmware Version: %v Serial Number: %v\n", stringDeviceInfo.Model, stringDeviceInfo.Hardware, stringDeviceInfo.Firmware, deviceInfo.Serial)
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
	sizeOfMessage = header[2] & 0x3F
	log.Printf("SIZE OF MESSAGE: %v", sizeOfMessage)

	typeCode = header[6]

	log.Printf("HEADER: %X", header)

	// the last number is the position of the byte eg. "...& 0xC0 >> 6", the 6 means the 6th byte
	responseMode := header[5] & 0xC0 >> 6
	//log.Printf("mode: %v", mode)

	return sizeOfMessage, typeCode, responseMode, nil
}

// StartScan starts up the scanning and data acquisition.
// see startScan for more details.
func (lidar *YDLidar) StartScan() {

	// Send start scanning command to device.
	if _, err := lidar.SerialPort.Write([]byte{preCommand, startScanning}); err != nil {
		lidar.sendErr(fmt.Errorf("failed to start scan: %v", err))
		return
	}

	/////////////////////////////////////////HEADER/////////////////////////////////////////////
	// size of message should be infinite, so we don't use the value here
	_, typeCode, responseMode, err := lidar.readInfoHeader()
	switch {
	case err != nil:
		err = fmt.Errorf("read header failed: %v", err)

	case typeCode != ScanTypeCode: // 0x81
		err = fmt.Errorf("invalid type code. Expected %x, got %X. Mode: %X", ScanTypeCode, typeCode, responseMode)

	case responseMode != ContinuousResponse: // 0x1
		err = fmt.Errorf("expected continuous response mode, got %X", responseMode)
	}

	if typeCode == ScanTypeCode && responseMode == ContinuousResponse {
		log.Print("Scan Command Response: GOOD")
		cycles := 0
		validFrames := 0
		// Start loop to read distance samples.
		for {
			select {
			default:
				cycles++
				log.Printf("revs: %v", cycles)

				/////////////////////HEADER/////////////////////////////////////////////
				var numHeaderBytesReceived int
				var numSampleBytesReceived int

				// The initial scan packet header is 10 theBytes.
				rawHeaderData := make([]byte, scanPacketHeaderSize)
				numHeaderBytesReceived, err := lidar.SerialPort.Read(rawHeaderData)
				if err != nil {
					lidar.sendErr(fmt.Errorf("failed to read serial %v", err))
				}

				// if numSampleBytesReceived != 10, log the actual value
				if numHeaderBytesReceived != scanPacketHeaderSize {
					log.Printf("The lidar gave us %v in the header packet. Expected 10.", numHeaderBytesReceived)
					log.Printf("The header packet is: %X ", rawHeaderData)
					continue
				}

				pointCloud := pointCloudHeader{}
				// Unpack the scan packet header into the pointCloudHeader struct.
				if err = binary.Read(bytes.NewBuffer(rawHeaderData), binary.LittleEndian, &pointCloud); err != nil {
					lidar.sendErr(fmt.Errorf("failed to pack struct: %v", err))
					continue
				}

				// extract the pointCloud into a slice of bytes

				// Returns scan data in a human readable format.
				packetHeader, scanningFrequency, dataPacketType, sampleQuantityPackets := lidar.extractScanPacketHeader(pointCloud)

				log.Printf("Chars: %X", packetHeader)

				if packetHeader == 0 && scanningFrequency == 0 && dataPacketType == 0 && sampleQuantityPackets == 0 {
					log.Printf("OTH PACKET, SKIPPING")
					continue
				}

				switch dataPacketType {
				case 0x1:

					//There is only one zero point of data in the zero start data packet. The sampleQuantityPackets is 1. We skip this packet.
					log.Printf("ZERO START DATA PACKET")
					if numSampleBytesReceived != scanPacketHeaderSize {
						log.Printf("not enough theBytes in header. Expected %v got %v", scanPacketHeaderSize, numSampleBytesReceived)
					}
					if sampleQuantityPackets != 1 {
						log.Printf("sample quantity should be 1, got %v", sampleQuantityPackets)
					}

					log.Print("Scanning Frequency is invalid in this packet")

				case 0x0:
					// LOOP OVER THE POINT CLOUD SAMPLES
					//The point cloud data packet contains the distance, angle, and luminosity data.
					validFrames++
					log.Printf("POINT CLOUD DATA PACKET FRAME #%v", validFrames)
					if sampleQuantityPackets <= 0 {
						log.Printf("sample quantity is less than 1 with a continuous response, got %v", sampleQuantityPackets)
						continue
					}
					log.Printf("Scanning Frequency: %vHz", scanningFrequency)

					/////////////////////////LUMINOSITY, DISTANCE, AND ANGLES/////////////////////////////////////
					// 3 bytes per sample, ex. If sampleQuantityPackets is 5, then lengthOfSampleData is 15 because there are 5 samples and each sample is 3 bytes.
					lengthOfSampleData := int(sampleQuantityPackets) * 3

					// Make a slice to hold the raw contents, 3 bytes per sample.
					rawSampleData := make([]byte, lengthOfSampleData)
					numSampleBytesReceived, err = lidar.SerialPort.Read(rawSampleData)
					if err != nil {
						log.Print(fmt.Errorf("failed to read serial %v", err))
					}

					// if the lidar didn't provide the data we expected, let us know
					if numSampleBytesReceived != lengthOfSampleData {
						log.Print(fmt.Errorf("incorrect number of bytes received. Expected %v got %v", lengthOfSampleData, numSampleBytesReceived))
					}

					// Unpack the rawSampleData into the individualSampleBytes slice.
					// the outer slice is the number of samples, the inner slice is the number of theBytes per sample
					individualSampleBytes := make([]byte, lengthOfSampleData) //
					if err = binary.Read(bytes.NewBuffer(rawSampleData), binary.LittleEndian, &individualSampleBytes); err != nil {
						log.Panic(fmt.Errorf("failed to pack struct: %v", err))
					}

					// Check Scan Packet Type.
					err = checkScanPacket(rawHeaderData, individualSampleBytes)
					if err != nil {
						log.Printf(err.Error())
						continue
					}

					// TODO Hoist conversions to separate function.

					// TODO Create intensity conversion function.
					//////////////////////////////////Intensity Calculations//////////////////////////////////

					// Si represents the number of samples.
					// Split the individualSampleBytes slice into a slice of slices.
					// Each slice is 3 theBytes long.
					// The outer slice is the number of samples.
					// The inner slice is the number of theBytes per sample.
					n := 3
					samples := make([][]byte, len(individualSampleBytes)/n)

					intensities := make([]uint16, len(individualSampleBytes)/n)
					distances := make([]uint16, len(individualSampleBytes)/n)
					for Si := range samples {
						samples[Si] = individualSampleBytes[Si*n : (Si+1)*n]
						// uint16(samples[Si][0]) means we take the whole first byte of this grouping
						// uint16(samples[Si][1]&0x3) means...&0x3 leaves us with the low two bits of the 2nd byte.
						intensity := (uint16(samples[Si][0]) + uint16(samples[Si][1]&3<<8)) * 256
						intensities[Si] = intensity
						log.Printf("intensity: %v", intensity)

						// Distance𝑖 = Lshiftbit(Si(3), 6) + Rshiftbit(Si(2), 2)
						// This variable represents the distance in millimeters.
						// uint16(samples[Si][2]) << 6 means we take the whole third byte of this grouping and shift it 6 bits to the left.
						// uint16(samples[Si][1]) >> 2 means we take the whole second byte of this grouping and shift it 2 bits to the right.
						distance := (uint16(samples[Si][2]) << 6) + (uint16(samples[Si][1]) >> 2)
						distances[Si] = distance

						log.Printf("distance: %vmm", distance)
					}

					// Contains distance and intensity data
					testSamplePacket := []byte{0x64, 0xE5, 0x6F}

					testReadableDistance := (uint16(testSamplePacket[2]) << 6) + (uint16(testSamplePacket[1]) >> 2)
					if testReadableDistance == 7161 {
						log.Printf("TEST READABLE DISTANCE: %v mm", testReadableDistance)
					}

					intensityByte1 := testSamplePacket[0]
					intensityByte2 := testSamplePacket[1] & 0x3

					if intensityByte1 == 100 {
						log.Printf("TEST INTENSITY BYTE 1: %v", intensityByte1)
					}

					if intensityByte2 == 1 {
						log.Printf("TEST INTENSITY BYTE 2: %v", intensityByte2)
					}

					testReadableIntensity := uint16(testSamplePacket[0]) + uint16(testSamplePacket[1]&0x3)*256
					if testReadableIntensity == 356 {
						log.Printf("TEST READABLE INTENSITY: %v", testReadableIntensity)
					}

					// Contains angle data (LSA and FSA) and the first byte is the sample length
					testAngle := []byte{0x28, 0xE5, 0x6F, 0xBD, 0x79}

					testReadableLSN := testAngle[0] // correct
					if testReadableLSN == 40 {
						log.Printf("TEST READABLE LSN: %v", testReadableLSN)
					}

					testRawFSA := (uint16(testAngle[2]) << 8) | uint16(testAngle[1])
					testReadableFSA := float32(testRawFSA>>1) / 64

					if testReadableFSA == 223.78125 {
						log.Printf("TEST READABLE FSA: %v", testReadableFSA)
					}

					testRawLSA := (uint16(testAngle[4]) << 8) | uint16(testAngle[3])
					testReadableLSA := float32(testRawLSA>>1) / 64

					if testReadableLSA == 243.46875 {
						log.Printf("TEST READABLE LSA: %v", testReadableLSA)
					}

					testReadableAngleDiff := testReadableLSA - testReadableFSA
					if testReadableAngleDiff == 19.6875 {
						log.Printf("TEST READABLE ANGLE DIFF: %v", testReadableAngleDiff)
					}

					//////////////////////////////Angle Calculations//////////////////////////////////
					angleFSA, angleLSA, angleDelta := lidar.CalculateAngles(distances, pointCloud.StartAngle, pointCloud.EndAngle, sampleQuantityPackets)
					/////////////////////////////////////////////////////////////////////////////////

					// Send the packet to the channel.
					lidar.Packets <- Packet{
						FirstAngle:         angleFSA,
						LastAngle:          angleLSA,
						DeltaAngle:         angleDelta,
						NumDistanceSamples: int(pointCloud.SampleQuantity),
						Distances:          distances,
						PacketType:         pointCloud.PackageType,
						Error:              err,
					}
				}

			case <-lidar.Stop:
				return
			}

		}
	}

}

func (lidar *YDLidar) extractScanPacketHeader(pointCloud pointCloudHeader) (uint16, uint8, uint8, uint8) {
	packetHeader := pointCloud.PacketHeader

	scanningFrequency := ((pointCloud.PackageType >> 1) & 0x7F) / 10

	dataPacketType := pointCloud.PackageType & 0x01

	sampleQuantity := pointCloud.SampleQuantity

	return packetHeader, scanningFrequency, dataPacketType, sampleQuantity
}

// checkScanPacket validates the type of the packet.
func checkScanPacket(headerData []byte, sampleData []byte) error {
	checkCode := byte(0)

	// Make a slice big enough to hold headerData (minus the check code position) and sampleData.

	// The check code uses a two-byte exclusive OR to verify the
	// current data packet. The check code itself does not participate in
	// XOR operations, and the XOR order is not strictly in byte order.
	C1 := make([]uint16, 1)
	bufferedC1 := bytes.NewBuffer(headerData[0:2])
	err := binary.Read(bufferedC1, binary.LittleEndian, &C1)
	if err != nil {
		return fmt.Errorf("failed to pack header struct: %v", err)
	}
	log.Printf("Length of header packet to XOR: %v", len(C1))

	C2 := make([]uint16, 1)
	bufferedC2 := bytes.NewBuffer(headerData[4:6])
	err = binary.Read(bufferedC2, binary.LittleEndian, &C2)
	if err != nil {
		return fmt.Errorf("failed to pack header struct: %v", err)
	}
	log.Printf("Length of header packet to XOR: %v", len(C2))

	nextToLastC := make([]uint16, 1)
	bufferedNextToLastC := bytes.NewBuffer(headerData[2:4])
	err = binary.Read(bufferedNextToLastC, binary.LittleEndian, &nextToLastC)
	if err != nil {
		return fmt.Errorf("failed to pack header struct: %v", err)
	}
	log.Printf("Length of header packet to XOR: %v", len(nextToLastC))

	lastC := make([]uint16, 1)
	bufferedLastC := bytes.NewBuffer(headerData[6:8])
	err = binary.Read(bufferedLastC, binary.LittleEndian, &lastC)
	if err != nil {
		return fmt.Errorf("failed to pack header struct: %v", err)
	}
	log.Printf("Length of header packet to XOR: %v", len(lastC))

	// Calculate Xor of all bits.
	for i, B := range C1 { // for each byte in the packet
		// XOR the current byte with the previous XOR
		checkCode ^= byte(B)

		switch i {

		case 0:
			// Check the first byte.
			if B != 0x55AA && B != 0xA55A {
				return fmt.Errorf("error: first byte of packet is not 0x55AA but %x", B)
			} else {
				log.Printf("First byte of header packet XOR is %x! Nice.", B)
			}
		}
	}

	for _, B := range C2 { // for each byte in the packet
		// XOR the current byte with the previous XOR
		checkCode ^= byte(B)
	}

	for _, B := range nextToLastC { // for each byte in the packet
		// XOR the current byte with the previous XOR
		checkCode ^= byte(B)
	}

	for _, B := range lastC { // for each byte in the packet
		// XOR the current byte with the previous XOR
		checkCode ^= byte(B)
	}

	samplePacket := make([]uint16, len(sampleData)/3)
	bufferedSamples := bytes.NewBuffer(sampleData)
	err = binary.Read(bufferedSamples, binary.LittleEndian, &samplePacket)
	if err != nil {
		return fmt.Errorf("failed to pack sample struct: %v", err)
	}
	log.Printf("Length of sample packet to XOR: %v", len(samplePacket))

	// for each byte in the packet
	for i, B := range samplePacket {
		// check if the byte is divisible by 3
		if i%3 == 0 {
			//zero fill the first 8 bits of this byte
			B = B << 8
		}

		// XOR the current byte with the previous XOR
		checkCode ^= byte(B)

		switch i {
		case 0:
			// Check the first byte.
		}
	}

	//if dataPacket[0]>>8 != 0 {
	//	return fmt.Errorf("error: upper 8 bits of first byte are not zero-filled")
	//}
	//
	//// XOR the zero-filled upper 8 bits of the first byte
	//checkCode ^= byte(dataPacket[0] >> 8)
	//
	//// Check the check code.
	//if checkCode != headerData[8] {
	//	return fmt.Errorf("error: check code does not match")
	//}

	return nil
}

// GetPointCloud returns point-cloud (intensity, dist, angle) from the data packet.
func GetPointCloud(packet Packet) (pointClouds []PointCloudData) {
	// Zero Point packet.
	if packet.PacketType == 1 {
		pointClouds = append(pointClouds,
			PointCloudData{
				Intensity: packet.Intensities[0],
				Angle:     packet.FirstAngle,
				Dist:      packet.Distances[0],
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
func (lidar *YDLidar) CalculateAngles(distances []uint16, endAngle uint16, startAngle uint16, sampleQuantity uint8) (float32, float32, float32) {
	if sampleQuantity == 1 {
		return 0, 0, 0
	}
	// setup the data

	// loop over the data and calculate the angles

	realReadableStartAngle := uint16(startAngle>>1) / 64
	realReadableEndAngle := uint16(endAngle>>1) / 64
	log.Printf("realReadableStartAngle: %v", realReadableStartAngle)
	log.Printf("realReadableEndAngle: %v", realReadableEndAngle)

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

// angleCorrection calculates the corrected angle for Lidar.
func angleCorrection(dist uint16) float32 {
	if dist == 0 {
		return -0
	}
	return float32(180 / math.Pi * math.Atan(21.8*(155.3-float64(dist))/(155.3*float64(dist))))
}
