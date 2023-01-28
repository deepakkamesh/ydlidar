// Capture point cloud data and output to http server as jpg.
package main

import (
	"bytes"
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"log"
	"math"
	"net/http"
	"strconv"

	"github.com/deepakkamesh/ydlidar"
)

func main() {
	// Get the serial port.
	ser, err := ydlidar.GetSerialPort("/dev/ttyUSB0")
	if err != nil {
		panic(fmt.Sprintf("Failed to init Lidar:%v", err))
	}
	// Setup and initialize the lidar.
	lidar := ydlidar.NewLidar()
	lidar.SetSerial(ser)
	if err := lidar.SetDTR(true); err != nil {
		panic(fmt.Sprintf("failed to set DTR:%v", err))
	}
	lidar.StartScan()

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
		if _, err := w.Write(buff.Bytes()); err != nil {
			_ = fmt.Errorf("unable to write image: %v", err)
		}
	})
	go func() {
		log.Print(http.ListenAndServe(":1337", nil))
	}()

	DEG2RAD := math.Pi / 180
	mapScale := 8.0
	revs := 0
	// Loop to read data from channel and construct image.
	for {
		d := <-lidar.D
		if d.Error != nil {
			panic(d.Error)
		}

		// ZeroPt indicates one revolution of lidar. Update image
		// every 10 revolutions.
		if d.PktType == 1 {
			revs++
			if revs == 10 {
				revs = 0
				buff.Reset()
				if err := jpeg.Encode(buff, img, &jpeg.Options{Quality: 70}); err != nil {
					fmt.Printf("%v", err)
				}
				img = image.NewRGBA(image.Rect(0, 0, 2048, 2048))
			}
		}

		for _, v := range ydlidar.GetPointCloud(d) {

			X := math.Cos(float64(v.Angle)*DEG2RAD) * float64(v.Dist)
			Y := math.Sin(float64(v.Angle)*DEG2RAD) * float64(v.Dist)
			Xocc := int(math.Ceil(X/mapScale)) + 1000
			Yocc := int(math.Ceil(Y/mapScale)) + 1000

			img.Set(Xocc, Yocc, color.RGBA{R: 200, G: 100, B: 200, A: 200})
		}
	}
}
