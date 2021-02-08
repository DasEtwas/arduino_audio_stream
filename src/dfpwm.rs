//! https://wiki.vexatos.com/dfpwm
//! https://github.com/OpenPrograms/gamax92-Programs/blob/master/dfpwm/lib/dfpwm.lua
//! https://github.com/ChenThread/dfpwm
//! https://github.com/Vexatos/Computronics/blob/1.12/src/main/java/pl/asie/lib/audio/DFPWM.java
//! https://github.com/gamax92/LionRay/blob/master/src/main/java/DFPWM.java
//!
//! DFPWM is a codec with a constant 8:1 compression rate, thus enabling storing decent lengths of audio
//! on an atmega2560 with 256KB of program memory.

const RESP_PREC: u32 = 10;
const RESP_INC: i32 = 1;
const RESP_DEC: i32 = 1;
const LPF_STRENGTH: i32 = 140;

#[derive(Copy, Clone)]
pub struct DfpwmPredictor {
    /// q (charge)
    level: i32,
    /// s (strength)
    response: i32,
    /// b'
    last_bit: bool,
}

impl DfpwmPredictor {
    fn predict(&mut self, bit: bool) {
        let target = if bit { 127 } else { -128 } as i32;

        let mut new_level = self.level
            + ((self.response * (target - self.level) + (1 << (RESP_PREC - 1))) >> RESP_PREC);

        if self.level == new_level && self.level != target {
            new_level += if bit { 1 } else { -1 };
        }

        let (response_delta, response_target) = if bit == self.last_bit {
            (RESP_INC, (1 << RESP_PREC) - 1)
        } else {
            (RESP_DEC, 0)
        };

        let mut new_response =
            self.response + (response_delta * (response_target - self.response) + 128) / 256;

        if self.response == new_response && self.response != response_target {
            if response_target < self.response {
                new_response -= 1;
            }
            if response_target > self.response {
                new_response += 1;
            }
        }

        if RESP_PREC > 8 {
            new_response = new_response.max(2 << (RESP_PREC - 8));
        }

        self.response = new_response;
        self.level = new_level;
        self.last_bit = bit;
    }
}

pub struct DfpwmDecoder {
    predictor: DfpwmPredictor,
    filter_lastlevel: i32,
    lpf_level: i32,
}

impl DfpwmDecoder {
    pub fn new() -> DfpwmDecoder {
        DfpwmDecoder {
            predictor: DfpwmPredictor {
                level: 0,
                response: 1,
                last_bit: false,
            },
            filter_lastlevel: 0,
            lpf_level: 0,
        }
    }

    #[inline]
    pub fn decompress_byte(&mut self, data: u8, output: &mut [i32; 8]) {
        decompress_byte(
            &mut self.filter_lastlevel,
            &mut self.lpf_level,
            output,
            &mut self.predictor,
            data,
        );
    }
}

fn decompress_byte(
    filter_lastlevel: &mut i32,
    lpf_level: &mut i32,
    output_buf: &mut [i32; 8],
    predictor: &mut DfpwmPredictor,
    mut data: u8,
) {
    for idx in 0..8 {
        let bit = (data & 1) != 0;
        let lastbit = predictor.last_bit;
        predictor.predict(bit);
        data >>= 1;

        let blevel = if bit == lastbit {
            predictor.level
        } else {
            (*filter_lastlevel + predictor.level + 1) / 2
        };

        *filter_lastlevel = predictor.level;

        *lpf_level += (LPF_STRENGTH * (blevel - *lpf_level) + 128) / 256;

        output_buf[idx] = *lpf_level;
    }
}

pub struct DfpwmEncoder {
    predictor: DfpwmPredictor,
}

impl DfpwmEncoder {
    pub fn new() -> DfpwmEncoder {
        DfpwmEncoder {
            predictor: DfpwmPredictor {
                level: 0,
                response: 1,
                last_bit: false,
            },
        }
    }

    #[inline]
    pub fn compress(&mut self, input: &[i32; 8]) -> u8 {
        let mut output = 0;
        for &sample in input {
            let bit = sample > self.predictor.level
                || (sample == self.predictor.level && self.predictor.level == 127);
            output >>= 1;
            if bit {
                output |= 128;
            }

            self.predictor.predict(bit);
        }
        output
    }
}
