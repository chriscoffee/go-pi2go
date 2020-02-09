package main

import (
	"fmt"
	"os"
	"time"

	"github.com/stianeikeland/go-rpio/v4"
)

var (
	pin = rpio.Pin(20) // front sonar
)

func getDistance() {
	if err := rpio.Open(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}

	defer rpio.Close()

	pin.Output()
	time.Sleep(1 * time.Millisecond)
	fmt.Println(pin.Read())
	pin.Toggle()
	fmt.Println(pin.Read())
}

func main() {
	getDistance()
}
