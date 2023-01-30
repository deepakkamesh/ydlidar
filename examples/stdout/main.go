// Uses mock serial interface to simulate lidar.
package main

import (
	"fmt"
	"log"
	. "ydlidar/ydlidar"
)

func main() {

	// TODO read in from config file with option to remain nil
	var devicePort *string

	lidar, err := InitAndConnectToDevice(devicePort)
	if err != nil {
		log.Panic(err)
	}
	lidar.StartScan()

	// Loop to read data from channel
	for {
		packet := <-lidar.Packets
		for _, v := range GetPointCloud(packet) {
			fmt.Printf(" %+v\n", v)
		}
	}
}
