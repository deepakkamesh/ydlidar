
// Automatically generated from the message definition "std_msgs/Time.msg"
package std_msgs
import (
    "bytes"
    "encoding/binary"
    "github.com/akio/rosgo/ros"
)


type _MsgTime struct {
    text string
    name string
    md5sum string
}

func (t *_MsgTime) Text() string {
    return t.text
}

func (t *_MsgTime) Name() string {
    return t.name
}

func (t *_MsgTime) MD5Sum() string {
    return t.md5sum
}

func (t *_MsgTime) NewMessage() ros.Message {
    m := new(Time)
	m.Data = ros.Time{}
    return m
}

var (
    MsgTime = &_MsgTime {
        `time data
`,
        "std_msgs/Time",
        "cd7166c74c552c311fbcc2fe5a7bc289",
    }
)

type Time struct {
	Data ros.Time `rosmsg:"data:time"`
}

func (m *Time) Type() ros.MessageType {
	return MsgTime
}

func (m *Time) Serialize(buf *bytes.Buffer) error {
    var err error = nil
    binary.Write(buf, binary.LittleEndian, m.Data.Sec)
    binary.Write(buf, binary.LittleEndian, m.Data.NSec)
    return err
}


func (m *Time) Deserialize(buf *bytes.Reader) error {
    var err error = nil
    {
        if err = binary.Read(buf, binary.LittleEndian, &m.Data.Sec); err != nil {
            return err
        }

        if err = binary.Read(buf, binary.LittleEndian, &m.Data.NSec); err != nil {
            return err
        }
    }
    return err
}
