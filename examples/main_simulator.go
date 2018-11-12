// Uses mock serial interface to simulate lidar.
package main

import (
	"bytes"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"io/ioutil"
	"log"
	"math"
	"net/http"
	"strconv"
	"testing"
	"time"

	"github.com/deepakkamesh/ydlidar"
	"github.com/deepakkamesh/ydlidar/mocks"
	"github.com/golang/mock/gomock"
)

func main() {

	flag.Parse()

	ydlidar := ydlidar.NewLidar()

	// mockSerial simulates serial port and generates
	// lidar data for testing and simulation.
	mockSerial := getMockSerial()
	ydlidar.SetMockSerial(mockSerial)
	go mockDataGen(mockSerial, "../scan.data")
	time.Sleep(10 * time.Millisecond)

	ydlidar.StartScan()

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
			log.Fatalf("Unable to write image: %v", err)
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
		X := math.Cos(d.Angle*DEG2RAD) * d.Dist
		Y := math.Sin(d.Angle*DEG2RAD) * d.Dist
		Xocc := int(math.Ceil(X/mapScale)) + 1000
		Yocc := int(math.Ceil(Y/mapScale)) + 1000

		img.Set(Xocc, Yocc, color.RGBA{200, 100, 200, 200})

		// ZeroPt indicates one revolution of lidar. Update image
		// every 100 revolutions.
		if d.ZeroPt {
			revs++
			if revs == 100 {
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

//<img width=100% src="/gridDisp" id="reloader" onload="setTimeout('document.getElementById(\'reloader\').src=\'/gridDisp?\'+new Date().getMilliseconds()', 1500)" />

func getMockSerial() *mocks.MockSerialPort {
	// mockSerial simulates a serial interface.
	var t testing.T
	mockCtrl := gomock.NewController(&t)
	return mocks.NewMockSerialPort(mockCtrl)
}

func mockDataGen(mockSerial *mocks.MockSerialPort, fname string) {
	// SetDTR.
	mockSerial.EXPECT().SetDTR(true).Return(nil).Times(1)

	// YDLidar scan command.
	mockSerial.EXPECT().Write([]byte{ydlidar.START, ydlidar.CSTARTSCAN}).Return(0, nil).Times(1)

	// Header read request.
	header := []byte{0xA5, 0x5A, 0x5, 0, 0, 0x40, 0x81}
	mockSerial.EXPECT().Read(make([]byte, 7)).Return(7, nil).SetArg(0, header).Times(1)

	// Open scan data file. Hex capture of sending scan command to lidar.
	c, e := ioutil.ReadFile(fname)
	if e != nil {
		fmt.Printf("Failed to open: %v", e)
	}
	c = c[7:] // Ignore the header.
	st := 0
	dlen := 10
	for {
		// If end of data hit, reset and continue streaming.
		if st >= len(c) {
			st = 0
			dlen = 10
			//break
		}

		// data preamble.
		data := c[st : st+dlen]
		mockSerial.EXPECT().Read(make([]byte, 10)).Return(10, nil).SetArg(0, data).Times(1)

		st += int(dlen)
		dlen := int(data[3] * 2)
		// distance data.
		data = c[st:(st + dlen)]
		mockSerial.EXPECT().Read(make([]byte, dlen)).Return(dlen, nil).SetArg(0, data).Times(1)

		st += int(dlen)
		dlen = 10
	}
}
