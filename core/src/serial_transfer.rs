use crate::memory::MemoryAccess;

pub struct SerialTransfer {
    buffer: Vec<u8>,
    serial_transfer_data: u8,
    serial_transfer_control: u8,
}

impl SerialTransfer {
    pub fn new() -> Self {
        Self {
            buffer: Vec::new(),
            serial_transfer_data: 0,
            serial_transfer_control: 0,
        }
    }

    pub fn buffer(&self) -> &[u8] {
        &self.buffer
    }
}

impl MemoryAccess for SerialTransfer {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0xFF01 => self.serial_transfer_data,
            0xFF02 => self.serial_transfer_control,
            _ => unreachable!("Invalid serial transfer address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF01 => self.serial_transfer_data = byte,
            0xFF02 => {
                self.serial_transfer_control = byte;

                // Allows us to inspect the serial port for testing, as Blarg's tests
                // write the output to the serial port
                if self.serial_transfer_control == 0x81 {
                    self.buffer.push(self.serial_transfer_data);
                }
            },
            _ => unreachable!("Invalid serial transfer address: {:04x}", addr),
        }
    }
}

