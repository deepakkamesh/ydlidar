// Capture point cloud data and output to http server as jpg.
package main

import (
	"bytes"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"strconv"

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

	// Get Device Info.
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

	// Start a HTTP service to serve up point cloud as a jpg image.
	img := image.NewRGBA(image.Rect(0, 0, 2048, 2048))
	buff := new(bytes.Buffer)

	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		str := []byte(`
			<!DOCTYPE html>
			<html>
			<body>
			<h1>Lidar Scan</h1>
			<p>refreshed every 1 secs </p>
				<img width=100% src="/map" id="reloader" onload="setTimeout('document.getElementById(\'reloader\').src=\'/map?\'+new Date().getMilliseconds()', 1000)" />
			</body>
			</html>
			`)
		if _, err := w.Write(str); err != nil {
			log.Fatalf("Unable to write image: %v", err)
		}
	})

	http.HandleFunc("/map", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "image/jpeg")
		w.Header().Set("Content-Length", strconv.Itoa(len(buff.Bytes())))
		fmt.Println("Output image of sz:", len(buff.Bytes()))
		if _, err := w.Write(buff.Bytes()); err != nil {
			fmt.Errorf("Unable to write image: %v", err)
		}
	})
	go func() {
		log.Fatal(http.ListenAndServe(":8080", nil))
	}()

	DEG2RAD := math.Pi / 180
	mapScale := 8.0
	revs := 0
	// Loop to read data from channel and construct image.
	for {
		d := <-ydlidar.D
		if d.Error != nil {
			panic(d.Error)
		}
		X := math.Cos(d.Angle*DEG2RAD) * d.Dist
		Y := math.Sin(d.Angle*DEG2RAD) * d.Dist
		Xocc := int(math.Ceil(X/mapScale)) + 1000
		Yocc := int(math.Ceil(Y/mapScale)) + 1000

		img.Set(Xocc, Yocc, color.RGBA{200, 100, 200, 200})

		// ZeroPt indicates one revolution of lidar. Update image
		// every 100 revolutions.
		if d.ZeroPt {
			revs++
			if revs == 10 {
				revs = 0
				buff.Reset()
				if err := jpeg.Encode(buff, img, &jpeg.Options{70}); err != nil {
					fmt.Printf("%v", err)
				}
				img = image.NewRGBA(image.Rect(0, 0, 2048, 2048))
			}
		}
	}
}
