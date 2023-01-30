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
	. "ydlidar/ydlidar"
)

func main() {

	// TODO read in from config file with option to remain nil
	var devicePort *string

	lidar, err := InitAndConnectToDevice(devicePort)
	if err != nil {
		log.Panic(err)
	}

	go lidar.StartScan()

	// Start an HTTP service to serve up point cloud as a jpg image.
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
			log.Panicf("Unable to write image: %v", err)
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
		log.Printf("Starting HTTP server on port 1337")
		log.Print(http.ListenAndServe(":1337", nil))
	}()

	DEG2RAD := math.Pi / 180
	mapScale := 8.0
	revs := 0
	// Loop to read data from channel and construct image.
	for {
		packet := <-lidar.Packets
		log.Printf("Packet type: %d", packet)

		// ZeroPt indicates one revolution of lidar. Update image
		// every 10 revolutions.
		if packet.PacketType == 1 {
			revs++
			if revs == 10 {
				revs = 0
				buff.Reset()
				if err := jpeg.Encode(buff, img, &jpeg.Options{Quality: 70}); err != nil {
					log.Panicf("%v", err)
				}
				img = image.NewRGBA(image.Rect(0, 0, 2048, 2048))
			}
		}

		for _, v := range GetPointCloud(packet) {

			X := math.Cos(float64(v.Angle)*DEG2RAD) * float64(v.Dist)
			Y := math.Sin(float64(v.Angle)*DEG2RAD) * float64(v.Dist)
			Xocc := int(math.Ceil(X/mapScale)) + 1000
			Yocc := int(math.Ceil(Y/mapScale)) + 1000

			img.Set(Xocc, Yocc, color.RGBA{R: 200, G: 100, B: 200, A: 200})
		}
	}

}
