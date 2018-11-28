## Running the examples

### rviz_simulation.go
This relies on the https://github.com/akio/rosgo package to provide client library for ROS. Download the library
and 

 # to generate the gengo binary
 go install github.com/akio/rosgo/gengo

# Generate the msg types
go generate 

# Run the binary
go run rviz_simulation.go
