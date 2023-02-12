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


	}
}
