package main

import (
	"flag"
	"log"
	"time"

	"github.com/deepakkamesh/ydlidar"
)

func main() {

	flag.Parse()
	lidar := ydlidar.NewLidar()

	if err := lidar.Init("/dev/tty.SLAB_USBtoUART"); err != nil {
		log.Printf("Error opening tty:%s", err)
	}
	/*
		if err := lidar.Status(); err != nil {
			log.Printf("Error with lidar:%v", err)
		}
		err, info := lidar.DeviceInfo()
		if err != nil {
			log.Printf("Error with lidar:%v", err)
		}
		log.Printf("Info %+v", info)
	*/
	st := make(chan struct{})
	go func() {
		if err := lidar.StartScan(st); err != nil {
			lidar.StopScan()
			log.Printf("err:%v", err)
		}
	}()
	time.Sleep(3 * time.Second)
	if err := lidar.StopScan(); err != nil {
		log.Printf("err%v", err)
	}
	st <- struct{}{}
	lidar.Close()

	for {
	}
}
