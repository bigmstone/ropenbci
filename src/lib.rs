use std::{
    sync::mpsc::{channel, Receiver},
    thread,
    thread::JoinHandle,
    time::Duration,
};

use {
    serde::{Deserialize, Serialize},
    serialport::{new as new_port, SerialPort},
};

pub struct OpenBCI {
    port: Box<dyn SerialPort>,
}

// TODO: At the moment I do not provide a way to configure the headset from this library. I
// configure it with the OpenBCI GUI and then use this. Hacky, but works.
impl OpenBCI {
    pub fn new() -> Self {
        // TODO: I should really make this come from a config, but you know how it is
        let port = new_port("/dev/tty.usbserial-DM025943", 115_200)
            .timeout(Duration::from_millis(1000))
            .open()
            .expect("Failed to open port");
        Self { port }
    }

    // TODO: This needs better error handling
    pub fn start(mut self) -> (Receiver<Reading>, JoinHandle<()>) {
        let (sender, receiver) = channel();
        self.port
            .write_all(&[0x76, 0x0A])
            .expect("Couldn't write value");
        let thread = thread::spawn(move || {
            self.port
                .write_all(&[0x62, 0x0A])
                .expect("Couldn't write value");
            let mut buffer = vec![];
            let mut packet_buffer = vec![];
            let mut packets: [Option<Packet>; 2] = Default::default();

            loop {
                let mut temp_buffer: [u8; 64] = [0; 64];
                self.port
                    .read(&mut temp_buffer)
                    .expect("Couldn't read from serial");
                buffer.extend(&temp_buffer);

                let mut purge_index = 0;

                for (index, byte) in buffer.iter().enumerate() {
                    if *byte == 0xA0 && index + 32 < buffer.len() {
                        let mut data: [u8; 32] = Default::default();
                        data.copy_from_slice(&buffer[index..index + 32]);
                        packet_buffer.push(Packet::from_bytes(&data));
                        purge_index = index + 32;
                    }
                }

                if purge_index > 0 {
                    buffer.drain(0..purge_index);
                }

                while packet_buffer.len() >= 2 {
                    let packet = packet_buffer.remove(0);

                    // If sample number is odd but we don't have a first packet then we've received
                    // packets out of order somehow and need to just discard this packet. The
                    // discarding happens by virtue of not processing it but it still being removed
                    // by the line above.
                    if packet.sample_number % 2 == 0 && packets[0].is_none() {
                        continue;
                    } else if packet.sample_number % 2 == 1 && packets[0].is_none() {
                        packets[0] = Some(packet);
                    } else if packet.sample_number % 2 == 0 && packets[1].is_none() {
                        packets[1] = Some(packet);
                    }

                    if packets[0].is_some() && packets[1].is_some() {
                        let packets = [packets[0].take().unwrap(), packets[1].take().unwrap()];
                        let reading = Reading::from_packets(packets);

                        sender.send(reading).unwrap();
                    }
                }
            }
        });

        (receiver, thread)
    }
}

impl Drop for OpenBCI {
    fn drop(&mut self) {
        self.port
            .write_all(&[0x73, 0x0A])
            .expect("Couldn't write value");
    }
}

#[derive(Debug)]
struct Packet {
    _header: u8,       // Byte 1: Packet Counter
    sample_number: u8, // Byte 2: Sample Number
    chan_1: i32,       // Bytes 3-5: Data value for EEG channel 1
    chan_2: i32,       // Bytes 6-8: Data value for EEG channel 2
    chan_3: i32,       // Bytes 9-11: Data value for EEG channel 3
    chan_4: i32,       // Bytes 12-14: Data value for EEG channel 4
    chan_5: i32,       // Bytes 15-17: Data value for EEG channel 5
    chan_6: i32,       // Bytes 18-20: Data value for EEG channel 6
    chan_7: i32,       // Bytes 21-23: Data value for EEG channel 6
    chan_8: i32,       // Bytes 24-26: Data value for EEG channel 8
    acc_x: u16,        // Bytes 27-28: Data value for accelerometer channel X
    acc_y: u16,        // Bytes 29-30: Data value for accelerometer channel Y
    acc_z: u16,        // Bytes 31-32: Data value for accelerometer channel Z
}

impl Packet {
    fn from_bytes(bytes: &[u8; 32]) -> Self {
        Self {
            _header: bytes[0],
            sample_number: bytes[1],
            chan_1: i24toi32(&bytes[2..=4]),
            chan_2: i24toi32(&bytes[5..=7]),
            chan_3: i24toi32(&bytes[8..=10]),
            chan_4: i24toi32(&bytes[11..=13]),
            chan_5: i24toi32(&bytes[14..=16]),
            chan_6: i24toi32(&bytes[17..=19]),
            chan_7: i24toi32(&bytes[20..=22]),
            chan_8: i24toi32(&bytes[23..=25]),
            acc_x: bytes[26] as u16 | ((bytes[27] as u16) << 8),
            acc_y: bytes[28] as u16 | ((bytes[29] as u16) << 8),
            acc_z: bytes[30] as u16 | ((bytes[31] as u16) << 8),
        }
    }
}

// TODO: Need to make this available for varied channel count
#[derive(Debug, Serialize, Deserialize)]
pub struct Reading {
    pub sample_numbers: (u8, u8),
    pub chan_1: i32,
    pub chan_2: i32,
    pub chan_3: i32,
    pub chan_4: i32,
    pub chan_5: i32,
    pub chan_6: i32,
    pub chan_7: i32,
    pub chan_8: i32,
    pub chan_9: i32,
    pub chan_10: i32,
    pub chan_11: i32,
    pub chan_12: i32,
    pub chan_13: i32,
    pub chan_14: i32,
    pub chan_15: i32,
    pub chan_16: i32,
    pub acc_x: u16,
    pub acc_y: u16,
    pub acc_z: u16,
}

impl Reading {
    fn from_packets(packets: [Packet; 2]) -> Self {
        Self {
            sample_numbers: (packets[0].sample_number, packets[1].sample_number),
            chan_1: packets[0].chan_1,
            chan_2: packets[0].chan_2,
            chan_3: packets[0].chan_3,
            chan_4: packets[0].chan_4,
            chan_5: packets[0].chan_5,
            chan_6: packets[0].chan_6,
            chan_7: packets[0].chan_7,
            chan_8: packets[0].chan_8,
            chan_9: packets[1].chan_1,
            chan_10: packets[1].chan_2,
            chan_11: packets[1].chan_3,
            chan_12: packets[1].chan_4,
            chan_13: packets[1].chan_5,
            chan_14: packets[1].chan_6,
            chan_15: packets[1].chan_7,
            chan_16: packets[1].chan_8,
            acc_x: packets[1].acc_x,
            acc_y: packets[1].acc_y,
            acc_z: packets[1].acc_z,
        }
    }
}

fn i24toi32(bytes: &[u8]) -> i32 {
    if bytes.len() != 3 {
        panic!("Byte array isn't of length 3");
    }
    let mut result: i32 = ((0xFF & bytes[0] as i32) << 16)
        | ((0xFF & bytes[1] as i32) << 8)
        | (0xFF & bytes[2] as i32);
    if (result & 0x00800000) > 0 {
        result = -(result & 0x007FFFFF);
    } else {
        result &= 0x00FFFFFF;
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_small_i24toi32() {
        let positive_array: [u8; 3] = [0x07, 0x86, 0x9E];
        let positive_value = 493214;
        let result = i24toi32(&positive_array);
        assert_eq!(positive_value, result);

        let negative_array: [u8; 3] = [0x87, 0x86, 0x9E];
        let negative_value = -493214;
        let result = i24toi32(&negative_array);
        assert_eq!(negative_value, result);
    }

    #[test]
    fn test_large_i24toi32() {
        let positive_array: [u8; 3] = [0x7F, 0xFF, 0xFF];
        let positive_value = 8388607;
        let result = i24toi32(&positive_array);
        assert_eq!(positive_value, result);

        let negative_array: [u8; 3] = [0xFF, 0xFF, 0xFF];
        let negative_value = -8388607;
        let result = i24toi32(&negative_array);
        assert_eq!(negative_value, result);
    }

    #[test]
    fn test_tiny_i24toi32() {
        let positive_array: [u8; 3] = [0x00, 0x00, 0x01];
        let positive_value = 1;
        let result = i24toi32(&positive_array);
        assert_eq!(positive_value, result);

        let negative_array: [u8; 3] = [0x80, 0x00, 0x01];
        let negative_value = -1;
        let result = i24toi32(&negative_array);
        assert_eq!(negative_value, result);
    }
}
