package ydlidar

import (
	"bytes"
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"io/ioutil"
	"log"
	"math"
	"net/http"
	"testing"

	"github.com/deepakkamesh/ydlidar/mocks"
	"github.com/golang/mock/gomock"
	"github.com/saljam/mjpeg"
	"github.com/stretchr/testify/assert"
)

func TestStatus(t *testing.T) {
	mockCtrl := gomock.NewController(t)
	defer mockCtrl.Finish()

	mockSerial := mocks.NewMockSerialPort(mockCtrl)
	ydlidar := YDLidar{ser: mockSerial}

	// YDLidar command.
	mockSerial.EXPECT().Write([]byte{START, CSTATUS}).Return(0, nil).Times(1)

	// Read header.
	header := []byte{0xA5, 0x5A, 0x3, 0, 0, 0, 0x6}
	mockSerial.EXPECT().Read(make([]byte, 7)).Return(7, nil).SetArg(0, header).Times(1)

	// Read data.
	data := []byte{0, 0, 0}
	mockSerial.EXPECT().Read(make([]byte, 3)).Return(3, nil).SetArg(0, data).Times(1)

	if err := ydlidar.Status(); err != nil {
		t.Errorf("expected no error. Got %v", err)
	}
}

func TestAngleCorr(t *testing.T) {

	r := angleCorrection(1000)
	assert.Equal(t, -6.762186021592019, r)
	r = angleCorrection(8000)
	assert.Equal(t, -7.837425240967011, r)
}

func TestScan(t *testing.T) {
	mockCtrl := gomock.NewController(t)
	defer mockCtrl.Finish()

	mockSerial := mocks.NewMockSerialPort(mockCtrl)
	ydlidar := NewLidar()
	ydlidar.SetMockSerial(mockSerial)

	// SetDTR.
	mockSerial.EXPECT().SetDTR(true).Return(nil).Times(1)

	// YDLidar scan command.
	mockSerial.EXPECT().Write([]byte{START, CSTARTSCAN}).Return(0, nil).Times(1)

	// Header read request.
	header := []byte{0xA5, 0x5A, 0x5, 0, 0, 0x40, 0x81}
	mockSerial.EXPECT().Read(make([]byte, 7)).Return(7, nil).SetArg(0, header).Times(1)

	// Open scan data file. Hex capture of sending scan command to lidar.
	c, e := ioutil.ReadFile("scan.data")
	if e != nil {
		t.Errorf("Failed to open: %v", e)
	}
	c = c[7:] // Ignore the header.
	st := 0
	dlen := 10
	for {
		if st >= len(c) {
			break
		}
		data := c[st : st+dlen]
		mockSerial.EXPECT().Read(make([]byte, 10)).Return(10, nil).SetArg(0, data).Times(1)

		st += int(dlen)
		dlen := int(data[3] * 2)
		data = c[st:(st + dlen)]
		mockSerial.EXPECT().Read(make([]byte, dlen)).Return(dlen, nil).SetArg(0, data).Times(1)

		st += int(dlen)
		dlen = 10

	}

	DEG2RAD := math.Pi / 180
	mapScale := 2.0
	Xd := []int{}
	Yd := []int{}

	ydlidar.StartScan()

	for i := 0; i < 2500; i++ {
		d := <-ydlidar.D
		fmt.Println("DD", d.Angle, d.Dist)

		X := math.Cos(d.Angle*DEG2RAD) * d.Dist
		Y := math.Sin(d.Angle*DEG2RAD) * d.Dist
		Xocc := int(math.Ceil(X/mapScale)) + 1000
		Yocc := int(math.Ceil(Y/mapScale)) + 1000
		Xd = append(Xd, Xocc)
		Yd = append(Yd, Yocc)
	}

	stream := mjpeg.NewStream()
	http.HandleFunc("/hello", func(w http.ResponseWriter, r *http.Request) {

	})
	img := image.NewRGBA(image.Rect(0, 0, 2048, 2048))

	j := 0
	go func() {
		for {

			for i := j + 0; i < j+400; i++ {
				// Write image to png.
				img.Set(Xd[i], Yd[i], color.RGBA{200, 100, 200, 200})
			}
			fmt.Println(j)
			j += 200
			if j > 2000 {
				j = 0
			}

			buff := new(bytes.Buffer)
			if err := jpeg.Encode(buff, img, &jpeg.Options{70}); err != nil {
				t.Errorf("%v", err)
			}
			stream.UpdateJPEG(buff.Bytes())
		}
	}()

	log.Fatal(http.ListenAndServe(":8080", nil))
}
