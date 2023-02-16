package ydlidar

import (
	"github.com/stretchr/testify/assert"
	"log"
	"math"
	"testing"
)

// TestGetHealthStatus tests the GetHealthStatus function.
func TestDistanceCalculation(t *testing.T) {

	// Contains distance data
	testSamplePacket := []byte{0x64, 0xE5, 0x6F}

	testReadableDistance := (uint16(testSamplePacket[2]) << 6) + (uint16(testSamplePacket[1]) >> 2)

	assert.Equal(t, 7161, testReadableDistance, "they should be equal")

}

func TestIntensityCalculation(t *testing.T) {
	// Contains intensity data
	testSamplePacket := []byte{0x64, 0xE5, 0x6F}

	intensityByte1 := testSamplePacket[0]
	intensityByte2 := testSamplePacket[1] & 0x3

	if intensityByte1 == 100 {
		log.Printf("TEST INTENSITY BYTE 1: %v", intensityByte1)
	}

	if intensityByte2 == 1 {
		log.Printf("TEST INTENSITY BYTE 2: %v", intensityByte2)
	}

	testReadableIntensity := uint16(testSamplePacket[0]) + uint16(testSamplePacket[1]&0x3)*256
	if testReadableIntensity == 356 {
		log.Printf("TEST READABLE INTENSITY: %v", testReadableIntensity)
	}

	assert.Equal(t, 356, testReadableIntensity, "they should be equal")
}

func TestAngleAnalysis(t *testing.T) {
	// Sample data
	lsn := 40 // 40 samples

	fsa := 0x6FE5 // 223.78125
	lsa := 0x79BD // 243.46875

	dist1 := 1000   // 1000mm
	distLsn := 8000 // 8000mm

	// First-level analysis
	angleStart := float64(fsa>>1) / 64
	angleEnd := float64(lsa>>1) / 64
	angleDiff := math.Mod(angleEnd-angleStart, 360)

	// Second-level analysis
	angCorrect1 := 0.0
	if dist1 != 0 {
		angCorrect1 = math.Atan((21.8*(155.3-float64(dist1)))/(155.3*float64(dist1))) * 180 / math.Pi
	}
	angCorrectLsn := 0.0
	if distLsn != 0 {
		angCorrectLsn = math.Atan((21.8*(155.3-float64(distLsn)))/(155.3*float64(distLsn))) * 180 / math.Pi
	}

	// Calculate intermediate angles
	angles := make([]float64, lsn-2)
	for i := 2; i < lsn; i++ {
		angles[i-2] = angleDiff/float64(lsn-1)*float64(i-1) + angleStart + angCorrect1
		if i == lsn {
			angles[i-2] += angCorrectLsn
		}
	}

	// Assert results
	expectedStart := 223.78
	expectedEnd := 243.47
	expectedDiff := 19.69
	assert.InDelta(t, expectedStart, angleStart, 0.01)
	assert.InDelta(t, expectedEnd, angleEnd, 0.01)
	assert.InDelta(t, expectedDiff, angleDiff, 0.01)
	expectedAngCorrect1 := -6.7622
	expectedAngCorrectLsn := -7.8374
	assert.InDelta(t, expectedAngCorrect1, angCorrect1, 0.0001)
	assert.InDelta(t, expectedAngCorrectLsn, angCorrectLsn, 0.0001)
	expectedAngles := []float64{217.0178, 219.2851, 221.5524, 223.8197, 226.0870, 228.3543, 230.6216, 232.8889, 235.1562, 237.4235, 239.6908, 241.9581, 244.2254, 246.4927, 248.7600, 251.0273, 253.2946, 255.5619, 257.8292, 260.0965, 262.3638, 264.6311, 266.8984, 269.1657, 271.4330, 273.7003, 275.9676, 278.2349, 280.5022, 282.7695, 285.0368, 287.3041, 289.5714, 291.8387, 294.1060, 296.3733, 298.6406, 300.9079, 303.1752}
	assert.InDeltaSlice(t, expectedAngles, angles, 0.0001)
}
