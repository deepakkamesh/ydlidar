package main

import (
	"flag"
	"fmt"
	"log"

	"github.com/deepakkamesh/ydlidar"
)

func main() {

	flag.Parse()
	lidar := ydlidar.NewLidar()

	if err := lidar.Init("/dev/tty.SLAB_USBtoUART"); err != nil {
		log.Printf("Error opening tty:%s", err)
	}
	log.Printf("dd")
	if err := lidar.Status(); err != nil {
		log.Printf("Error with lidar:%v", err)
	}
	err, info := lidar.DeviceInfo()
	if err != nil {
		log.Printf("Error with lidar:%v", err)
	}
	fmt.Printf("Info %v", info)

}
