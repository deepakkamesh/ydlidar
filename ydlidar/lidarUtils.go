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
func NewLidar() *YDLidar {
	return &YDLidar{
		Packets: make(chan Packet),
		Stop:    make(chan struct{}),
	}
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
	// else iterate over ports to get the correct oen
	ports, err := serial.GetPortsList()
	if err != nil {
		log.Panic(err)
	}

	if len(ports) == 0 {
		log.Panic("No serial ports found!")
	}

	for _, port := range ports {
		mode := &serial.Mode{
			BaudRate: 230400,          // 230400 baud
			DataBits: 8,               // 8 data bits
			Parity:   serial.NoParity, // No parity
			StopBits: 0,               // 1 stop bit
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
	log.Printf("Stopping scan")
	if _, err := lidar.SerialPort.Write([]byte{preCommand, stopScanning}); err != nil {
		return err
	}
	lidar.Stop <- struct{}{}
	err := lidar.SerialPort.Close()
	if err != nil {
		return err
	}
	return nil

}

// sendErr sends error on channel.
func (lidar *YDLidar) sendErr(e error) {
	lidar.Packets <- Packet{
		Error: e,
	}
}

// SetDTR enables the DTR control for serial which controls the motor enable function.
func (lidar *YDLidar) SetDTR(s bool) {
	lidar.SerialPort.SetDTR(s)
}

// startScan runs the data acquisition from the lidar.
func (lidar *YDLidar) startScan() {
	continuousResponse := byte(1)

	if _, err := lidar.SerialPort.Write([]byte{preCommand, startScanning}); err != nil {
		lidar.sendErr(fmt.Errorf("failed to start scan: %v", err))
		return
	}

	// Read and validate header.
	_, typeCode, responseMode, err := readHeader(lidar.SerialPort)
	switch {
	case err != nil:
		err = fmt.Errorf("read header failed: %v", err)

	case typeCode != ScanTypeCode:
		err = fmt.Errorf("invalid type code. Expected %x, got %v. Mode: %x", ScanTypeCode, typeCode, responseMode)

	case responseMode != continuousResponse:
		err = fmt.Errorf("expected continuous response mode, got %v", responseMode)
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
			newHeaderBuffer := make([]byte, 10)
			n, err := lidar.SerialPort.Read(newHeaderBuffer)
			if byte(n) != 10 {
				lidar.sendErr(fmt.Errorf("start Scan: not enough bytes. Expected %v got %v", 10, n))
				continue
			}
			if err != nil {
				lidar.sendErr(fmt.Errorf("failed to read serial %v", err))
				continue
			}

			pointCloudHeader := pointCloudHeader{}

			buf := bytes.NewBuffer(newHeaderBuffer)
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
				lidar.sendErr(fmt.Errorf("start Scan Sampling Quality: not enough bytes. Expected %v got %v", pointCloudHeader.SamplingQuantity*2, n))
				continue
			}

			readings := make([]int16, pointCloudHeader.SamplingQuantity)
			buf = bytes.NewBuffer(data)
			if err = binary.Read(buf, binary.LittleEndian, &readings); err != nil {
				lidar.sendErr(fmt.Errorf("failed to pack struct: %v", err))
				continue
			}
			log.Printf("Start Scan Reading: %v\n", readings)

			//--------------------------- Everything above this works for the G2 ---------------------------

			// Check CRC of the packet.
			err = checkCRC(newHeaderBuffer, data, pointCloudHeader.CheckCode)
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

			log.Printf("Number of Samples: %v\n", int(pointCloudHeader.SamplingQuantity))
			log.Printf("Packet Header: %v\n", int(pointCloudHeader.PacketHeader))
			log.Printf("Distance: %v\n", distances)
			log.Printf("Angle Delta: %v\n", angleDelta)

			lidar.Packets <- Packet{
				MinAngle:           angleFSA,
				MaxAngle:           angleLSA,
				NumDistanceSamples: int(pointCloudHeader.SamplingQuantity),
				Distances:          distances,
				DeltaAngle:         angleDelta,
				PacketType:         int(pointCloudHeader.PacketHeader),
			}

		}
	}
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

// checkCRC validates the CRC of the packet.
// https://www.robotshop.com/media/files/content/y/ydl/pdf/ydlidar_x4_development_manual.pdf pg 6.
func checkCRC(header []byte, data []byte, checkCode uint16) error {

	// Make a 16bit slice big enough to hold header (minus CRC) and data.
	dataPacket := make([]uint16, 4+len(data)/2)

	log.Printf("Datapacket: %v\n", len(dataPacket))

	buffer := bytes.NewBuffer(append(header[:8], data...))
	if err := binary.Read(buffer, binary.LittleEndian, &dataPacket); err != nil {
		return fmt.Errorf("failed to pack struct: %v", err)
	}

	// Calculate Xor of all bits.
	x := uint16(0)
	for i := 0; i < len(dataPacket); i++ {
		x ^= dataPacket[i]
	}

	log.Printf("XOR: %v\n", x)

	if x != checkCode {
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

// Close will shut down the connection.
func (lidar *YDLidar) Close() error {
	return lidar.SerialPort.Close()
}

// Reboot soft reboots the lidar.
func (lidar *YDLidar) Reboot() error {
	if _, err := lidar.SerialPort.Write([]byte{preCommand, restartDevice}); err != nil {
		return err
	}
	return nil
}

// DeviceInfo returns the version information.
func (lidar *YDLidar) DeviceInfo() (*string, error) {
	if _, err := lidar.SerialPort.Write([]byte{preCommand, deviceInfo}); err != nil {
		return nil, err
	}

	sizeOfMessage, typeCode, mode, err := readHeader(lidar.SerialPort)
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

	sizeOfMessage, typeCode, mode, err := readHeader(lidar.SerialPort)
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

// readHeader reads the header portion of the response.
func readHeader(serialPort serial.Port) (sizeOfMessage byte, typeCode byte, mode byte, err error) {
	header := make([]byte, 7)
	n, err := serialPort.Read(header)

	if err != nil {
		return 0, 0, 0, err
	}

	if n != 7 {
		err = fmt.Errorf("read Header: not enough bytes reading header. Expected 7 bytes got %v", n)
		return 0, 0, 0, err
	}

	preamble := int(header[1])<<8 | int(header[0])

	if preamble != 0x5AA5 {
		err = fmt.Errorf("invalid header. Expected preamble 0x5AA5 got %x", preamble)
		return 0, 0, 0, err
	}

	sizeOfMessage = header[2]
	typeCode = header[6]
	mode = header[5] & 0xC0 >> 6

	return sizeOfMessage, typeCode, mode, nil
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

func InitAndConnectToDevice(ttyPort *string) (*YDLidar, error) {
	var devicePort serial.Port
	var err error

	if ttyPort != nil { // use ttyPort if provided
		devicePort, err = GetSerialPort(ttyPort)
		if err != nil {
			return nil, err
		}

	} else { // try to connect to the first available device
		devicePort, err = GetSerialPort(nil)
		if err != nil {
			return nil, err
		}
	}

	err = devicePort.SetReadTimeout(1000 * time.Millisecond)
	if err != nil {
		return nil, err
	}

	lidar := NewLidar()

	lidar.SetupCloseHandler()
	lidar.SetSerial(devicePort)
	lidar.SetDTR(true)

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
