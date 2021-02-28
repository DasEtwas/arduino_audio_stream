// define one of these to determine mode of operation
//#define STREAM_PCM
//#define STREAM_DFPWM
//#define STORED_PCM
#define STORED_DFPWM

#if defined(STORED_PCM) || defined(STORED_DFPWM)
#include "sounddata.h"
#else

//////////////////// adjustable variables ////////////////////
// tested maximum sample rates before audio begins skipping at a baud rate of 2_000_000:
// STREAM_DFPWM: 42000Hz
// STREAM_PCM:   90000Hz
#define SAMPLE_RATE 48000UL
#endif
#define BAUD_RATE 2000000

// must be a multiple of 8
#define PLAYBACK_BUFFER_SIZE 512
#define BUFFER_COUNT 3

// DFPWM parameters
#define RESP_PREC 10
#define RESP_INC 1
#define RESP_DEC 1
#define LPF_STRENGTH 140
//////////////////////////////////////////////////////////////

volatile int currentReadBuffer = 0;
volatile int currentWriteBuffer = 0;
volatile int8_t buffers[BUFFER_COUNT][PLAYBACK_BUFFER_SIZE];
volatile boolean bufferEmpty[BUFFER_COUNT];

typedef struct {
  int16_t level { 0 };
  int16_t response { 0 };
  boolean lastBit { false };

  void predict(boolean input) {
    int16_t target = input ? 127 : -128;

    int16_t newLevel = (uint16_t) ((uint32_t) level + (uint16_t) (((uint32_t) response * (target - level) + (1 << (RESP_PREC - 1))) >> RESP_PREC));

    if (level == newLevel && level != target) {
      newLevel += input ? 1 : -1;
    }

    int16_t responseDelta;
    int16_t responseTarget;

    if (input == lastBit) {
      responseDelta  = RESP_INC;
      responseTarget = (1 << RESP_PREC) - 1;
    } else {
      responseDelta  = RESP_DEC;
      responseTarget = 0;
    }

    int16_t newResponse = (uint16_t) ((uint32_t) response + ((uint32_t)responseDelta * (responseTarget - response) + 128) / 256);

    if (response == newResponse && response != responseTarget) {
      if (responseTarget < response) {
        newResponse -= 1;
      }
      if (responseTarget > response) {
        newResponse += 1;
      }
    }

    if (RESP_PREC > 8 && newResponse < (2 << (RESP_PREC - 8))) {
      newResponse = 2 << (RESP_PREC - 8);
    }

    response = newResponse;
    level = newLevel;
    lastBit = input;
  }
} DfpwmPredictor;

typedef struct {
  DfpwmPredictor* predictor;
  int16_t filterLastLevel { 0 };
  int16_t lpfLevel { 0 };

  void dfpwmDecode(uint8_t data, int16_t* output) {
    for (int i = 0; i < 8; i++) {
      boolean b = ((data >> i) & 1) != 0;

      boolean lastBit = predictor->lastBit;
      predictor->predict(b);

      int16_t blevel;

      if (b == lastBit) {
        blevel = predictor->level;
      } else {
        blevel = (filterLastLevel + predictor->level + 1) / 2;
      }

      filterLastLevel = predictor->level;

      lpfLevel += (LPF_STRENGTH * (blevel - lpfLevel) + 128) / 256;

      output[i] = lpfLevel;
    }
  }
} DfpwmDecoder;

DfpwmPredictor predictor;
DfpwmDecoder decoder;

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(50);

  decoder.predictor = &predictor;

  memset(&bufferEmpty[0], true, BUFFER_COUNT);
  memset(&buffers[0][0], 0, BUFFER_COUNT * PLAYBACK_BUFFER_SIZE);

  pinMode(10, OUTPUT); //Speaker on pin 10

  //disable interrupts while registers are configured
  cli();

  //set Timer2 to fast PWM mode (doubles PWM frequency)
  bitSet(TCCR2A, WGM20);
  bitSet(TCCR2A, WGM21);

  // sets prescaler to 1: 62.5kHz
  bitSet(TCCR2B, CS20);
  bitClear(TCCR2B, CS21);
  bitClear(TCCR2B, CS22);

  // enable pin 10 output
  bitClear(TCCR2A, COM2A0); bitSet(  TCCR2A, COM2A1);

  // stores { enum register value, divisor }
  const uint16_t prescalers[5][2] = {
    {1, 1},
    {2, 8},
    {3, 64},
    {4, 256},
    {5, 1024}
  };
  uint8_t prescaler = 1;
  uint32_t ocr = 1;

  for (int i = 0; i < 5; i++) {
    uint16_t divisor = prescalers[i][1];

    uint32_t count = F_CPU / (divisor * SAMPLE_RATE) - 1;

    if (count < (1UL << 16) - 1) {
      if (count + 1 < (1UL << 16) - 1) {
        // round up if more precise
        if (F_CPU - (count + 1) * divisor * SAMPLE_RATE
            > (count + 2) * divisor * SAMPLE_RATE - F_CPU)
        {
          count += 1;
        }
      }

      ocr = count;
      prescaler = prescalers[i][0];

      Serial.print("Chosen values for sample rate of ");
      Serial.print(SAMPLE_RATE);
      Serial.print("Hz: ");
      Serial.print("OCR = ");
      Serial.print(ocr);
      Serial.print(", prescaler = ");
      Serial.print(divisor);
      Serial.print(", effective sample rate: ");
      Serial.print((float) F_CPU / (divisor * (count + 1)));
      Serial.println("Hz");
      Serial.flush();

      break;
    }
  }

#ifdef STREAM_PCM
  Serial.println("Decoding serial as PCM u8");
#endif
#ifdef STREAM_DFPWM
  Serial.println("Decoding serial as DFPWM");
#endif

  // disconnect pins from our interrupt timer (COM4{A,B,C}{0, 1}
  TCCR4A = 0;
  TCCR4B = prescaler & 0b111;
  TCCR4C = 0;
  TIMSK4 = 0;
  TIFR4 = 0;

  // wave generation mode 4: CTC, counts up to value stored in OCR4A
  bitSet(TCCR4B, WGM42);
  bitSet(TIMSK4, OCIE4A); // Output Compare Interrupt Enable A
  // 16 bit register (output compare A of timer 4)
  OCR4A = (uint16_t) (ocr & 0xFFFF);

  //enable interrupts
  sei();
}

int playback = 0;
int stallCounter = 4 * -PLAYBACK_BUFFER_SIZE;
boolean playbackStarted = false;

ISR(TIMER4_COMPA_vect) {
  if (bufferEmpty[currentReadBuffer]) {
    if (playbackStarted) {
      stallCounter++;
      if (stallCounter > PLAYBACK_BUFFER_SIZE) {
        Serial.println("Playback is stalling, reduce sample rate!");
        stallCounter = 0;
      }
    }

    currentReadBuffer = (currentReadBuffer + 1) % BUFFER_COUNT;
  } else {
    playbackStarted = true;
    if (playback >= PLAYBACK_BUFFER_SIZE) {
      bufferEmpty[currentReadBuffer] = true;
      playback = 0;
      currentReadBuffer = (currentReadBuffer + 1) % BUFFER_COUNT;
      return;
    }

    // set duty cycle of timer 2 wave (-> analog voltage)
#if defined(STREAM_PCM) || defined(STORED_PCM)
    OCR2A = buffers[currentReadBuffer][playback];
#endif
#if defined(STREAM_DFPWM) || defined(STORED_DFPWM)
    OCR2A = buffers[currentReadBuffer][playback] + 128;
#endif

    playback += 1;
  }
}

// must be 8
#define DFPWM_BUF_SIZE 8
// stores decoded dfpwm samples
int16_t dfpwmBuf[DFPWM_BUF_SIZE];

#if defined(STORED_PCM) || defined(STORED_DFPWM)
uint32_t storedIndex = 0;
#endif

void loop() {
  if (currentWriteBuffer == currentReadBuffer) {
    // decoder is going too fast
    return;
  }
  if (bufferEmpty[currentWriteBuffer]) {
    /////////////////////// pcm modes ///////////////////////
#ifdef STREAM_PCM
    uint8_t download[PLAYBACK_BUFFER_SIZE];
    int read = 0;
    while (read < PLAYBACK_BUFFER_SIZE) {
      read += Serial.readBytes(&download[read], PLAYBACK_BUFFER_SIZE - read);
    }
    memcpy(&buffers[currentWriteBuffer][0], &download[0], PLAYBACK_BUFFER_SIZE);
#endif
#ifdef STORED_PCM
    uint8_t download[PLAYBACK_BUFFER_SIZE];
    sounddataRead(&download[0], PLAYBACK_BUFFER_SIZE, storedIndex);

    storedIndex = (storedIndex + PLAYBACK_BUFFER_SIZE) % SOUNDDATA_LENGTH;
    memcpy(&buffers[currentWriteBuffer][0], &download[0], PLAYBACK_BUFFER_SIZE);
#endif

    /////////////////////// dfpwm modes ///////////////////////
#ifdef STREAM_DFPWM
    uint8_t download[PLAYBACK_BUFFER_SIZE / DFPWM_BUF_SIZE];

    int read = 0;
    while (read < PLAYBACK_BUFFER_SIZE / DFPWM_BUF_SIZE) {
      read += Serial.readBytes(&download[read], PLAYBACK_BUFFER_SIZE / DFPWM_BUF_SIZE - read);
    }
#endif
#ifdef STORED_DFPWM
    uint8_t download[PLAYBACK_BUFFER_SIZE / DFPWM_BUF_SIZE];

    sounddataRead(&download[0], PLAYBACK_BUFFER_SIZE / DFPWM_BUF_SIZE, storedIndex);

    storedIndex = (storedIndex + PLAYBACK_BUFFER_SIZE / DFPWM_BUF_SIZE) % SOUNDDATA_LENGTH;
#endif
#if defined(STREAM_DFPWM) || defined(STORED_DFPWM)
    for (int i = 0; i < PLAYBACK_BUFFER_SIZE / DFPWM_BUF_SIZE; i++) {
      decoder.dfpwmDecode(download[i], &dfpwmBuf[0]);

      for (int j = 0; j < DFPWM_BUF_SIZE; j++) {
        int16_t sample = dfpwmBuf[j];
        int16_t clipped;

        if (sample < 127) {
          if (sample > -128) {
            clipped = sample;
          } else {
            clipped = -128;
          }
        } else {
          clipped = 127;
        }

        buffers[currentWriteBuffer][i * DFPWM_BUF_SIZE + j] = (int8_t) clipped;
      }
    }
#endif

    // buffer is now filled
    bufferEmpty[currentWriteBuffer] = false;
  }

  currentWriteBuffer = (currentWriteBuffer + 1) % BUFFER_COUNT;
}
