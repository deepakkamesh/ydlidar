// Write commands directly to serial interface.
package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
	"time"

	serial "github.com/deepakkamesh/go-serial/serial"
)

func main() {
	fmt.Println("Welcome to LIDAR CLI")
	log.SetFlags(log.Lmicroseconds)
	in := bufio.NewReader(os.Stdin)

	c := serial.OpenOptions{
		//	PortName:              "/dev/ttyUSB0",
		PortName:              "/dev/tty.SLAB_USBtoUART",
		BaudRate:              128000,
		DataBits:              8,
		StopBits:              1,
		MinimumReadSize:       1,
		InterCharacterTimeout: 200,
		ParityMode:            serial.PARITY_NONE,
	}
	ser, err := serial.Open(c)
	if err != nil {
		log.Fatalf("Error opening %s", err)
	}
	time.Sleep(time.Millisecond * 100)

	ser.SetDTR(false)
	// Uncomment to write bytes to file.
	/*	dump, err := os.OpenFile("scan.dump", os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
		if err != nil {
			panic(err)
		}*/

	// Read the serial line in a goroutine.
	go func() {
		for {
			buf := make([]byte, 10)
			n, err := ser.Read(buf)
			if err != nil {
				fmt.Printf("Error reading %s", err)
				continue
			}
			log.Printf("Got %d bytes buf:%v  buf:%x ", n, buf, buf)
			// Write scan data to file. Ensure len(buf) == 1
			/*			if _, err := dump.Write(buf); err != nil {
							fmt.Printf("%v", err)
						}
						dump.Sync() */
		}
	}()

	// Command read and process loop.
START:
	for {
		line, err := in.ReadString('\n')
		if err != nil {
			log.Printf("Failed to read stdin: %v", err)
		}
		// Split input into string slice.
		inputs := strings.Split(strings.Trim(line, "\n "), " ")

		// Convert string into bytes slice.
		var bytes []byte
		for i := 0; i < len(inputs); i++ {
			c, err := strconv.ParseUint(inputs[i], 16, 8)
			if err != nil {
				log.Printf("Error converting input: %v", err)
				continue START
			}
			bytes = append(bytes, byte(c))
		}

		// Check command.
		switch bytes[1] {
		case 0x60:
			if err := ser.SetDTR(true); err != nil {
				log.Printf("Error setting DTR:%v", err)
			}

		case 0x65:
			if err := ser.SetDTR(false); err != nil {
				log.Printf("Error setting DTR:%v", err)
			}
		}

		// Write to serial line.
		if _, err := ser.Write(bytes); err != nil {
			log.Printf("failed to send to serial: %v", err)
		}
		log.Printf("Sent %v", bytes)
	}
}
