// Capture distance samples and output to stdout.
package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"

	"github.com/deepakkamesh/ydlidar"
)

func main() {

	flag.Parse()

	ydlidar := ydlidar.NewLidar()

	if err := ydlidar.Init("/dev/tty.SLAB_USBtoUART"); err != nil {
		log.Fatalf("Failed to init Lidar:%v", err)
	}

	// Check device health.
	if err := ydlidar.Status(); err != nil {
		log.Fatalf("Failed to query status:%v", err)
	}

	// Get device Info.
	err, info := ydlidar.DeviceInfo()
	if err != nil {
		log.Fatalf("Failed to read device health:%v", err)
	}
	fmt.Printf("%+v\n", info)

	ydlidar.StartScan()

	// Catch interrupts to exit clean.
	c := make(chan os.Signal)
	signal.Notify(c, os.Interrupt)
	go func() {
		select {
		case sig := <-c:
			log.Printf("Got %s signal. Aborting...\n", sig)

			if err := ydlidar.StopScan(); err != nil {
				log.Printf("Failed to stop scan: %v", err)
			}
			if err := ydlidar.Close(); err != nil {
				log.Printf("Failed to close: %v", err)
			}
			os.Exit(1)
		}
	}()

	// Read and output distance data.
	for {
		d := <-ydlidar.D
		if d.Error != nil {
			panic(d.Error)
		}
		fmt.Println(d)
	}

}
