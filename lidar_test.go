package ydlidar

import (
	"testing"

	"github.com/deepakkamesh/ydlidar/mocks"
	"github.com/golang/mock/gomock"
)

func TestStatus(t *testing.T) {
	mockCtrl := gomock.NewController(t)
	defer mockCtrl.Finish()

	mockSerial := mocks.NewMockSerialPort(mockCtrl)
	ydlidar := YDLidar{ser: mockSerial}

	mockSerial.EXPECT().Write([]byte{START, cSTATUS}).Return(0, nil).Times(1)

	header := []byte{0xA5, 0x5A, 0x3, 0, 0, 0, 0x6}
	mockSerial.EXPECT().Read(make([]byte, 7)).Return(7, nil).SetArg(0, header).Times(1)

	data := []byte{0, 0, 0}
	mockSerial.EXPECT().Read(make([]byte, 3)).Return(3, nil).SetArg(0, data).Times(1)

	if err := ydlidar.Status(); err != nil {
		t.Errorf("expected no error. Got %v", err)
	}

}
