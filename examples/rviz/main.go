// Uses mock serial interface to simulate lidar.
package main

// source rviz_env.sh prior to building and running.
// run "go generate" to generate the msg files.
//go:generate gengo msg sensor_msgs/LaserScan
//go:generate gengo msg std_msgs/Header
//go:generate gengo msg std_msgs/Time
// Note: Due to bug in rosgo the md5sum for laserscan msg is incorrect.
// It needs to be updated in the LaserScan.go file based on
// rosmsg md5 sensor_msgs/LaserScan.

import (
	"fmt"
	"math"
	"os"
	"sensor_msgs"
	"std_msgs"
	"time"

	"github.com/akio/rosgo/ros"
	"github.com/deepakkamesh/ydlidar"
)

const DEG2RAD float32 = math.Pi / 180

func main() {

	// GetMockSerial provides a simulated serial port and generates
	// lidar data and start the mock data generations with pre-recorded data file.
	/*
		ser := ydlidar.GetMockSerial()
		go ydlidar.MockDataGen(ser, "../../scan.data")
		time.Sleep(10 * time.Millisecond)
	*/
	// Or uncomment to get real serial port.

	ser, err := ydlidar.GetSerialPort("/dev/tty.SLAB_USBtoUART")
	if err != nil {
		panic(fmt.Sprintf("Failed to init Lidar:%v", err))
	}

	// Setup and initialize the lidar.
	l := ydlidar.NewLidar()
	l.SetSerial(ser)
	l.StartScan()

	// Start up ros node.
	node, err := ros.NewNode("/ydlidar", os.Args)
	if err != nil {
		fmt.Println(err)
		os.Exit(-1)
	}

	defer node.Shutdown()

	node.Logger().SetSeverity(ros.LogLevelInfo)
	pub := node.NewPublisher("/scan", sensor_msgs.MsgLaserScan)
	seq := uint32(0)
	freq := float32(12)
	for node.OK() {
		node.SpinOnce()
		d := <-l.D

		for i := 0; i < d.Num; i++ {
			d.Distances[i] = d.Distances[i] / 1000
		}
		hdr := std_msgs.Header{
			Seq:     seq,
			Stamp:   ros.NewTime(uint32(time.Now().Unix()), uint32(time.Now().UnixNano())),
			FrameId: "laser_frame",
		}
		_ = hdr
		pkt := sensor_msgs.LaserScan{
			Header:         hdr,
			AngleMin:       DEG2RAD * d.MinAngle,
			AngleMax:       DEG2RAD * d.MaxAngle,
			AngleIncrement: DEG2RAD * d.DeltaAngle / float32(d.Num),
			TimeIncrement:  (1 / freq) / float32(d.Num),
			ScanTime:       float32(1) / freq,
			RangeMin:       0.08,
			RangeMax:       10,
			Ranges:         d.Distances,
		}
		pub.Publish(&pkt)
		time.Sleep(time.Millisecond * 100)
		seq++
	}

}
