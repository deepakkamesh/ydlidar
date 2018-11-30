package ydlidar

import (
	"fmt"
	"io/ioutil"
	"testing"

	"github.com/deepakkamesh/ydlidar/mocks"
	"github.com/golang/mock/gomock"
)

func GetMockSerial() *mocks.MockSerialPort {
	// mockSerial simulates a serial interface.
	var t testing.T
	mockCtrl := gomock.NewController(&t)
	return mocks.NewMockSerialPort(mockCtrl)
}

func MockDataGen(mockSerial *mocks.MockSerialPort, fname string) {
	// SetDTR.
	mockSerial.EXPECT().SetDTR(true).Return(nil).Times(1)

	// YDLidar scan command.
	mockSerial.EXPECT().Write([]byte{START, CSTARTSCAN}).Return(0, nil).Times(1)

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
