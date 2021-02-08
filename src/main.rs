use crate::dfpwm::DfpwmEncoder;
use clap::{App, Arg};
use hound::{SampleFormat, WavReader};
use serialport::{DataBits, Parity, SerialPortType, StopBits};
use std::fs::File;
use std::io::{BufReader, Write};
use std::io::{BufWriter, Read};
use std::num::Wrapping;
use std::thread::{sleep, spawn, yield_now};
use std::time::{Duration, Instant};

mod dfpwm;

/// Should be equal to or a divisor of the value in arduino code. Must be a multiple of eight
const PLAYBACK_BUFFER_SIZE: usize = 512; // 256, 128, ..

fn main() {
    let matches = App::new("arduino_audio_stream")
        .arg(
            Arg::with_name("INPUT_FILE")
                .required(true)
                .takes_value(true)
                .long_help("input WAVE format file, is automatically converted to mono")
                .index(1),
        )
        .arg(
            Arg::with_name("OUTPUT_FILE")
                .takes_value(true)
                .required_if("mode", "stored")
                .help("Output path of generated header file")
                .index(2),
        )
        .arg({
            Arg::with_name("mode")
                .short("m")
                .long("mode")
                .takes_value(true).required(true)
                .possible_values(&["stream", "stored"])
                .case_insensitive(true)
                .long_help("stream - Corresponds to \"STREAM_\" modes in the arduino code.\n         Sends audio in chunks over serial.\nstored - Corresponds to \"STORED_\" modes in the arduino code.\n         Generates a header file at OUTPUT_FILE with audio data and exits.\n")
        })
        .arg({
            Arg::with_name("format")
                .short("f")
                .long("format")
                .takes_value(true).required(true)
                .possible_values(&["pcm", "dfpwm"])
                .case_insensitive(true)
                .long_help("pcm   - Sends arduino raw audio samples in unsigned byte format.\ndfpwm - Sends arduino DFPWM-encoded data.\n        This reduces bandwidth by a factor of 8 but takes more processing power, thus reducing the possible sample rate.\n")
        })
        .arg(
            Arg::with_name("baudrate")
                .short("r")
                .required_if("mode", "stream")
                .long("baudrate")
                .default_value("2000000")
                .possible_values(&[
                    "2000000", "1000000", "500000", "250000", "230400", "115200", "74880", "57600",
                    "38400", "19200", "9600", "4800", "2400", "1200", "300",
                ])
                .help("Serial baud rate in bits/sec"),
        )
        .arg(
            Arg::with_name("port")
                .short("-p")
                .long("port")
                .required_if("mode", "stream")
                .takes_value(true)
                .help("Serial port"),
        )
        .arg(
            Arg::with_name("maxsize")
                .short("s")
                .long("maxsize")
                .default_value("240000")
                .long_help("The generated binary header will be truncated to this size.")
                .validator(|s| {
                    s.parse::<u32>()
                        .map_err(|e| e.to_string())
                        .and_then(|size| {
                            if size > 240000 {
                                // even using this size yielded verification failure using avrdude
                                Err("Size exceeds 243KB".to_owned())
                            } else {
                                Ok(())
                            }
                        })
                }),
        )
        .get_matches();

    assert_eq!(PLAYBACK_BUFFER_SIZE % 8, 0);

    enum Mode {
        /// Stream audio over serial
        Stream,
        /// Generate a header file containing the given file to upload it to the arduino
        Stored,
    }

    enum Format {
        /// Dynamic Filter Pulse Width Modulation <https://wiki.vexatos.com/dfpwm>
        Dfpwm,
        /// 8 bit per sample PCM data (eg. WAVE file)
        Pcm,
    }

    let mode = match matches.value_of("mode").unwrap() {
        "stream" => Mode::Stream,
        "stored" => Mode::Stored,
        _ => unreachable!(),
    };
    let format = match matches.value_of("format").unwrap() {
        "dfpwm" => Format::Dfpwm,
        "pcm" => Format::Pcm,
        _ => unreachable!(),
    };
    let file_name = matches.value_of("INPUT_FILE").unwrap();

    let samples_per_byte = match format {
        Format::Dfpwm => 8,
        Format::Pcm => 1,
    };

    let mut input_file_reader =
        BufReader::new(File::open(file_name).expect("Failed to open audio file"));

    let wav_reader = WavReader::new(&mut input_file_reader).expect("Failed to read WAVE file");
    let spec = wav_reader.spec();

    println!("Sample rate: {}Hz", spec.sample_rate);

    let mut audio = match spec.sample_format {
        SampleFormat::Float => wav_reader
            .into_samples::<f32>()
            .map(|v| v.unwrap())
            .collect::<Vec<f32>>()
            // convert to mono
            .chunks_exact(spec.channels as usize)
            .map(|channels| channels.iter().sum::<f32>() / spec.channels as f32)
            // map range
            .map(|v| {
                // convert from -1.0..1.0 to -127..127
                ((v * u8::MAX as f32) as i32)
                    .min(u8::MAX as i32)
                    .max(u8::MIN as i32) as i8
            })
            .collect::<Vec<i8>>(),
        SampleFormat::Int => {
            let divisor = (1 << (spec.bits_per_sample - 8)) as i32;

            wav_reader
                .into_samples::<i32>()
                .map(|v| v.unwrap() as i64)
                .collect::<Vec<i64>>()
                // convert to mono
                .chunks_exact(spec.channels as usize)
                .map(|channels| (channels.iter().sum::<i64>() / spec.channels as i64) as i32)
                // map range
                .map(|v| (v / divisor) as i8)
                .collect::<Vec<i8>>()
        }
    };

    let binary = match format {
        Format::Dfpwm => {
            let mut encoder = DfpwmEncoder::new();
            audio.extend_from_slice(&[0i8; 7]);

            audio
                .chunks_exact(8)
                .map(|chunk| {
                    // read pcm s8 data into i32 buffer for encoding
                    let mut input: [i32; 8] = [0; 8];
                    input
                        .iter_mut()
                        .zip(chunk.iter())
                        .for_each(|(input, wav)| *input = *wav as i32);

                    encoder.compress(&input)
                })
                .collect::<Vec<u8>>()
        }
        Format::Pcm => audio
            .into_iter()
            .map(|sample| (Wrapping(sample as u8) + Wrapping(128)).0)
            .collect::<Vec<u8>>(),
    };

    let sample_rate = spec.sample_rate;

    match mode {
        Mode::Stored => {
            println!("Generating sound data C++ file");

            // atmega2560 has about 252KB of free program memory left
            let max_size = matches.value_of("maxsize").unwrap().parse::<u32>().unwrap() as usize;
            if binary.len() > max_size {
                println!(
                    "Will truncate to {} bytes ({:.3} seconds)",
                    max_size,
                    max_size as f32 * samples_per_byte as f32 / sample_rate as f32
                );
            } else {
                println!(
                    "Fit {:.3} seconds of audio in header file",
                    max_size as f32 * samples_per_byte as f32 / sample_rate as f32
                );
            }
            let audio = &binary[..binary.len().min(max_size)];

            let mut file = BufWriter::new(
                File::create(
                    matches
                        .value_of("OUTPUT_FILE")
                        .expect("no output file given"),
                )
                .expect("Failed to create output file"),
            );

            let splits = (audio.len() + i16::MAX as usize - 1) / i16::MAX as usize;

            writeln!(
                &mut file,
                "// Generated binary data storage for file \"{}\"\n\n#include <avr/pgmspace.h>\n#define SAMPLE_RATE {}\nconst uint32_t SOUNDDATA_LENGTH = {};\n",
                file_name,
                sample_rate,
                audio.len()
            ).expect("Failed to write to file");

            for (i, audio) in audio.chunks(i16::MAX as usize).enumerate() {
                writeln!(
                    &mut file,
                    r#"const uint8_t SOUNDDATA{}[{}] PROGMEM = {{"#,
                    i,
                    audio.len()
                )
                .expect("Failed to write to file");
                let wrap = 20;
                for bytes in audio.chunks(wrap) {
                    for byte in bytes {
                        write!(&mut file, "{:3},", byte).expect("Failed to write to file");
                    }
                    writeln!(&mut file).expect("Failed to write to file");
                }
                writeln!(&mut file, "}};\n\n").expect("Failed to write to file");
            }

            writeln!(
                &mut file,
                r#"void sounddataRead(uint8_t* sram_buf, int len, uint32_t sounddataIndex) {{
  int rest;
  int read = 0;
  while (read < len) {{
    rest = 32767 - sounddataIndex % 32767;
    int readLen = min(rest, len - read);

    switch (sounddataIndex / 32767) {{
{switch_case}    }}

    read += readLen;
    sounddataIndex = (sounddataIndex + readLen) % SOUNDDATA_LENGTH;
  }}
}}"#, switch_case = {
                            let mut s = String::new();
                            for i in 0..splits {
                                s.push_str(
                                    format!("      case {}: memcpy_PF(&sram_buf[read], pgm_get_far_address(SOUNDDATA{}) + sounddataIndex % 32767, readLen); break;\n", i, i)
                                        .as_str(),
                                );
                            }
                            s
                        }
                    )
                    .expect("Failed to write to file");

            file.flush().expect("Failed to write to file");

            println!("Done writing header file to '{}'", file_name);
        }
        Mode::Stream => {
            let ports = serialport::available_ports();

            match &ports {
                Ok(ports) => {
                    match ports.len() {
                        0 => println!("No ports found."),
                        1 => println!("Found 1 port:"),
                        n => println!("Found {} ports:", n),
                    };
                    for p in ports {
                        println!("  {}", p.port_name);
                        match &p.port_type {
                            SerialPortType::UsbPort(info) => {
                                println!("    Type: USB");
                                println!("    VID:{:04x} PID:{:04x}", info.vid, info.pid);
                                println!(
                                    "     Serial Number: {}",
                                    info.serial_number.as_ref().map_or("", String::as_str)
                                );
                                println!(
                                    "      Manufacturer: {}",
                                    info.manufacturer.as_ref().map_or("", String::as_str)
                                );
                                println!(
                                    "           Product: {}",
                                    info.product.as_ref().map_or("", String::as_str)
                                );
                            }
                            SerialPortType::BluetoothPort => {
                                println!("    Type: Bluetooth");
                            }
                            SerialPortType::PciPort => {
                                println!("    Type: PCI");
                            }
                            SerialPortType::Unknown => {
                                println!("    Type: Unknown");
                            }
                        }
                    }

                    let port_name = matches.value_of("port").unwrap();

                    let port_info = ports
                        .iter()
                        .find(|p| p.port_name == port_name)
                        .expect("Failed to find serial port");

                    let baud_rate = matches
                        .value_of("baudrate")
                        .unwrap()
                        .parse::<u32>()
                        .unwrap();

                    let required_baud_rate = sample_rate as usize * 8 / samples_per_byte;
                    if required_baud_rate > baud_rate as usize {
                        eprintln!(
                            "Baud rate is insufficient for realtime playback (required: {})",
                            required_baud_rate
                        );
                    }

                    // resets arduino over serial using DTR pin
                    // https://github.com/arduino/Arduino/blob/b3d609f4de066289088739747a743076cf4ea023/arduino-core/src/processing/app/Serial.java#L98
                    let mut port = serialport::new(&port_info.port_name, baud_rate)
                        .timeout(Duration::from_secs(2))
                        .stop_bits(StopBits::One)
                        .data_bits(DataBits::Eight)
                        .parity(Parity::None)
                        .open()
                        .expect("Failed to open serial port");

                    port.write_data_terminal_ready(false)
                        .expect("Failed to send reset signal");
                    port.write_data_terminal_ready(true)
                        .expect("Failed to send reset signal");

                    println!("Resetting arduino..");

                    // increase this value if serial comm doesnt start
                    sleep(Duration::from_millis(1500));

                    println!("Connection is considered open");
                    println!("=============== Arduino serial output and warnings ===============");

                    spawn({
                        let mut clone = port.try_clone().expect("failed to clone serial port");
                        // prints arduino serial output
                        move || loop {
                            sleep(Duration::from_millis(5));

                            if let Ok(available) = clone.bytes_to_read() {
                                let mut recv_buf = vec![0; available as usize];

                                if let Ok(()) = clone.read_exact(&mut recv_buf) {
                                    let s = unsafe { std::str::from_utf8_unchecked(&recv_buf) };
                                    print!("{}", s);

                                    // these help warnings are not guaranteed due work because the message may be fragmented
                                    {
                                        let sample_rate_pattern = "sample rate of ";
                                        if s.contains(sample_rate_pattern) {
                                            if let Some((idx, _)) =
                                                s.match_indices(sample_rate_pattern).next()
                                            {
                                                let start_idx = idx + sample_rate_pattern.len();
                                                if let Some((end_idx, _)) =
                                                    s[start_idx..].match_indices("Hz").next()
                                                {
                                                    if let Ok(number) = s
                                                        [start_idx..start_idx + end_idx]
                                                        .trim()
                                                        .parse::<f32>()
                                                    {
                                                        if number as u32 != sample_rate {
                                                            eprintln!("audio_stream: Sample rate on arduino is mismatched! ({} vs {}) Change in arduino code or supply different input file", number, sample_rate);
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        if s.contains("Decoding serial as PCM u8")
                                            && !matches!(format, Format::Pcm)
                                        {
                                            eprintln!("audio_stream: Format is mismatched! (arduino expects PCM) (send rate will also cause noticeable stuttering)");
                                        }
                                        if s.contains("Decoding serial as DFPWM")
                                            && !matches!(format, Format::Dfpwm)
                                        {
                                            eprintln!("audio_stream: Format is mismatched! (arduino expects DFPWM)");
                                        }
                                    }
                                }
                            }
                        }
                    });

                    let send_buf_size: usize = PLAYBACK_BUFFER_SIZE / samples_per_byte as usize;
                    let mut buf = vec![0u8; send_buf_size];

                    // play endless loop
                    let mut binary_iter = binary.iter().cloned().cycle();

                    let mut last_time = Instant::now();

                    loop {
                        // send (PLAYBACK_BUFFER_SIZE / SAMPLES_PER_BYTE) bytes at the rate they are consumed by the chip
                        while last_time.elapsed()
                            < Duration::from_secs_f64(
                                (samples_per_byte * send_buf_size) as f64 / sample_rate as f64,
                            )
                        {
                            std::thread::yield_now();
                        }
                        last_time = Instant::now();

                        let write_len = port.bytes_to_write().unwrap() as usize;
                        let send_buf =
                            &mut buf[..send_buf_size.min(send_buf_size.saturating_sub(write_len))];

                        send_buf
                            .iter_mut()
                            .zip(&mut binary_iter)
                            .for_each(|(buf, audio)| *buf = audio);

                        port.write_all(send_buf).expect("Failed to send serial");
                    }
                }
                Err(e) => {
                    eprintln!("Error listing serial ports {:?}", e);
                }
            }
        }
    }
}
