# audio_stream

Audio playback via USB Serial or PROGMEM on pin 10 of the Arduino MEGA2560 board.

This project contains [Rust](https://www.rust-lang.org/) code to handle host-side serial communication and audio encoding as well as generation of C++ header files containing audio to be stored into the atmega2560's PROGMEM.

Each playback mode (`STREAM_PCM`, `STREAM_DFPWM`, `STORED_PCM`, `STORED_DFPWM`) has a host-side utility.

Sample rate is determined by the WAVE file's sample rate. Multi-channel audio is averaged into mono.

The CLI written in [Rust](https://www.rust-lang.org/) can be installed to be available anywhere in your shell like this:
```shell
git clone https://github.com/DasEtwas/arduino_audio_stream
cargo install --path . --force
```
(use `cargo uninstall arduino_audio_stream` to uninstall respectively)

An [Arduino IDE](https://www.arduino.cc/en/Main.Software) cpp file containing arduino audio playback code can be found here [audio_stream.ino](arduino-ide/audio_stream/audio_stream.ino).

### Streaming

Using the streaming modes, `arduino_audio_stream` can be used to stream pcm u8 or [DFPWM](https://wiki.vexatos.com/dfpwm) encoded audio.

Example streaming myaudio.wav as pcm:
```shell
arduino_audio_stream myaudio.wav arduino-ide/audio_stream/sounddata.h --format pcm --mode stream --port /dev/ttyACM0 
```
(corresponding arduino mode: `STREAM_PCM`)

This will open `/dev/ttyACM0` and stream the chunked audio in PCM format to the arduino.

**NOTE**: Generally, PCM allows for higher sample rates than DFPWM because decoding takes a while, and should be preferred for realtime playback for higher quality.

**NOTE**: The streaming may be baud-rate sensitive. The higher the better.

### Using PROGMEM

To store the file "myaudio.wav" in a header file besides the given example project, consider the following example usage:

```shell
arduino_audio_stream myaudio.wav arduino-ide/audio_stream/sounddata.h --format dfpwm --mode stored
```

After enabling the `#define STORED_DFPWM` in [audio_stream.ino](arduino-ide/audio_stream/audio_stream.ino), compile and upload. Sample rate and data length are included automatically.

**NOTE**: To achieve storing any proper length of continous audio on the device, one needs to use DFPWM which has an 8:1 compression ratio and low sample rates (eg. 16kHz or less). Higher sample rate sounds better up to 24-32kHz. Example: To fit 2:20 of audio onto the mega2560, I used a sample rate of 12900Hz and dfpwm encoding.

### Electrical

Especially when using DFPWM, adding low-pass filtering and amplifier on pin 10 is recommended. The more bass the signal has, the more pronounced are squeaking artifacts (due to DFPWM being more sensitive at higher amplitudes).

### Technical details
`audio_stream` uses Timer 2 for 62500Hz PWM for analog output on pin 10, of which the OCR2A register is modulated in 8-bit mode to achieve audio output (duty-cycle modulation).
Timer 4 is used to call the Output Compare Match A interrupt at the sample rate by setting it to CTC mode and setting OCR4A.

### Basic overview of DFPWM

DFPWM stands for Dynamic Filter Pulse Width Modulation. It uses 3 input parameters for a basic LPF and signal response. Each bit corresponds to one output sample.
A 1 bit tells the predictor to increase amplitude, whereas a 0 bit tells it to decrease amplitude. Multiple of the same values (0 or 1) following eachother
cause an increase in `response` (sensitivity). A silent signal is just repeated ones and zeroes.

The best parameters where chosen by trying out every one by bruteforcing after analying noise using the [RMSE](https://en.wikipedia.org/wiki/Root-mean-square_deviation) of the input and DFPWM output. (`RESP_PREC = 10`, `RESP_INC = 1`, `RESP_DEC = 1` are found to have the least RMSE and coincidentally are also the values in >1.7.10 minecraft versions of the minecraft mod Computronics mod including the DFPWM codec :P)

### Credits

* https://www.maxpierson.me/2009/05/29/generate-real-time-audio-on-the-arduino-using-pulse-code-modulation/
* https://forum.arduino.cc/index.php?topic=485507.0
* https://wiki.vexatos.com/dfpwm (great minecraft mod btw, I used to listen to noisy audio all day in game)

## License
MIT
