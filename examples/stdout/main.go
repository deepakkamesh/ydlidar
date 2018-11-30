// Uses mock serial interface to simulate lidar.
package main

import (
	"fmt"
	"time"

	"github.com/deepakkamesh/ydlidar"
)

func main() {

	// GetMockSerial provides a simulated serial port and generates
	// lidar data and start the mock data generations with pre-recorded data file.

	ser := ydlidar.GetMockSerial()
	go ydlidar.MockDataGen(ser, "../../scan.data")
	time.Sleep(10 * time.Millisecond)

	// Or uncomment to get real serial port.
	/*
		ser, err := ydlidar.GetSerialPort("/dev/tty.SLAB_USBtoUART")
		if err != nil {
			panic(fmt.Sprintf("Failed to init Lidar:%v", err))
		}
	*/

	// Setup and initialize the lidar.
	l := ydlidar.NewLidar()
	l.SetSerial(ser)
	l.StartScan()

	// Loop to read data from channel
	for {
		d := <-l.D
		for _, v := range ydlidar.GetPointCloud(d) {
			fmt.Printf(" %+v\n", v)
		}
	}
}