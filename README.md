# ydlidar G2 Go API.

This was forked from an X4 repo to be made compatible with the YDLIDAR G2 and maybe more YDLIDAR prodcuts in the future.


To find your device port, run the following command:
```plaintext
sudo dmesg | grep tty
```


You can pass in your device port as the first argument to the program.
```plaintext
go run main.go /dev/ttyUSB0
```

## Future Plans
* Post-processing of lidar data and .las/.laz file creation
* Add more YDLIDAR products
* Add more features to the API