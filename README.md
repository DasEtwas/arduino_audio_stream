## audio_stream

Audio playback via USB Serial or PROGMEM on pin 10 of the Arduino MEGA2560 board.

This project contains [Rust](https://www.rust-lang.org/) code to handle host-side serial communication and audio encoding as well as generation of C++ header files containing audio to be stored into the atmega2560's PROGMEM.

Each playback mode (`STREAM_PCM`, `STREAM_DFPWM`, `STORED_PCM`, `STORED_DFPWM`) has a host-side utility.

The CLI written in [Rust](https://www.rust-lang.org/) can be installed to be available anywhere in your shell like this:
```shell
git clone https://github.com/DasEtwas/arduino_audio_stream
cargo install --path . --force
```
(use `cargo uninstall arduino_audio_stream` respectively)

### Streaming

Using the streaming modes, `arduino_audio_stream` can be used to stream pcm u8 formatted or [DFPWM](https://wiki.vexatos.com/dfpwm) encoded audio.

```shell
arduino_audio_stream myaudio.wav arduino-ide/audio_stream/sounddata.h --format pcm --mode stream --port /dev/ttyACM0 
```
(corresponding arduino mode: `STREAM_PCM`)

### Using PROGMEM

To store the file "myaudio.wav" in a header file besides the given example project, consider the following example usage:

```shell
arduino_audio_stream myaudio.wav arduino-ide/audio_stream/sounddata.h --format dfpwm --mode stored
```
(corresponding arduino mode: `STORED_DFPWM`)

### Technical details
`audio_stream` uses Timer 2 for 62500Hz PWM for analog output on pin 10, of which the OCR2A register is modulated in 8-bit mode to achieve audio output (duty-cycle modulation).
Timer 4 is used to call the Output Compare Match A interrupt at the sample rate by setting it to CTC mode and setting OCR4A.


## License
MIT