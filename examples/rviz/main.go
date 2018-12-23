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
	// raw lidar data and start the mock data generations with pre-recorded data file.

	ser := ydlidar.GetMockSerial()
	go ydlidar.MockDataGen(ser, "../../scan3.data")
	time.Sleep(10 * time.Millisecond)

	// Or uncomment to get real serial port.
	/*
		ser, err := ydlidar.GetSerialPort("/dev/tty.SLAB_USBtoUART")
		if err != nil {
			panic(fmt.Sprintf("Failed to init Lidar:%v", err))
		}
	*/
	// Setup and initialize the lidar.
	l := ydlidar.NewLidar()
	l.SetSerial(ser)
	if err := l.SetDTR(true); err != nil {
		panic(fmt.Sprintf("failed to set DTR:%v", err))
	}
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

	minAngle := float32(360)
	maxAngle := float32(0)
	dist := []float32{}

	for {
		d := <-l.D

		if d.PktType == 1 { // Zero packet.

			// Convert from mm to m.
			for i := 0; i < len(dist); i++ {
				dist[i] = dist[i] / 1000
			}

			sendTopic(node, pub, seq, "laser_frame", minAngle, maxAngle, dist)
			seq++

			dist = []float32{}
			minAngle = 360
			maxAngle = 0
			continue
		}

		// Ignore transitions from 360 -> 0. eg. min: 340 max: 4.
		if d.MinAngle >= d.MaxAngle {
			continue
		}
		if minAngle > d.MinAngle {
			minAngle = d.MinAngle
		}
		if maxAngle < d.MaxAngle {
			maxAngle = d.MaxAngle
		}
		dist = append(dist, d.Distances...)
	}
}

func sendTopic(node ros.Node, pub ros.Publisher, seq uint32, laserFrame string, minAngle float32, maxAngle float32, dist []float32) {

	if node.OK() {
		hdr := std_msgs.Header{
			Seq:     seq,
			Stamp:   ros.NewTime(uint32(time.Now().Unix()), uint32(time.Now().UnixNano())),
			FrameId: "laser_frame",
		}
		pkt := sensor_msgs.LaserScan{
			Header:         hdr,
			AngleMin:       DEG2RAD * minAngle,
			AngleMax:       DEG2RAD * maxAngle,
			AngleIncrement: DEG2RAD * (maxAngle - minAngle) / float32(len(dist)),
			TimeIncrement:  0,
			ScanTime:       float32(1) / 12,
			RangeMin:       0.08,
			RangeMax:       10,
			Ranges:         dist,
		}
		pub.Publish(&pkt)
		node.SpinOnce()
	}

}
