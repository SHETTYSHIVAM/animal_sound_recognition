#include <Arduino.h>
#include "driver/i2s.h"

#define I2S_PORT I2S_NUM_0

// ============================================================
// INMP441 connections
// ============================================================

#define I2S_BCLK 26
#define I2S_LRCL 25
#define I2S_DIN  33


// ============================================================
// Audio configuration
// ============================================================

#define SAMPLE_RATE   16000
#define AUDIO_DURATION 2
#define AUDIO_SAMPLES (SAMPLE_RATE * AUDIO_DURATION)

#define N_MELS            26
#define N_MFCC        13
#define N_FFT         512
#define HOP_LENGTH    160
#define WIN_LENGTH    400
#define BUFFER_SIZE 512

// Number of MFCC frames that fit in one 2-second capture.
// FIX: this was used throughout the file but never defined.
#define N_FRAMES (1 + (AUDIO_SAMPLES - WIN_LENGTH) / HOP_LENGTH)


int16_t audioBuffer[AUDIO_SAMPLES];

// ============================================================
// FEATURE BUFFER
// ============================================================

#define NUM_FEATURES 26

float features[NUM_FEATURES];

// FFT buffers
float fftReal[N_FFT];
float fftImag[N_FFT];

// Mel filter bank
// FIX (DRAM overflow): a dense [N_MELS][N_FFT/2+1] matrix costs ~26.7KB, and
// a triangular filter is zero almost everywhere in it. Store only the bin
// edges and compute each filter's (small) triangular weight on the fly
// inside computeMFCCFrame() instead.
int melBinsGlobal[N_MELS + 2];

// FIX: mfcc must be 2D (coefficient x frame) — code indexes it as mfcc[c][frameIndex]
// everywhere. The original declared it as a flat float[NUM_MFCC], which does not compile.
float mfcc[N_MFCC][N_FRAMES];


// ==================================================
// GENERATED DECISION TREE
// ==================================================

#define MONKEY 0
#define NOISE 1
#define PEACOCK 2
#define WILD_BOAR 3

int predict(float features[26]) {
    if (features[21] <= 8.886531f) {
        if (features[11] <= -10.511693f) {
            return MONKEY;
        } else {
            if (features[3] <= 26.956030f) {
                if (features[12] <= -14.814839f) {
                    return MONKEY;
                } else {
                    if (features[1] <= 70.735317f) {
                        if (features[23] <= 2.592306f) {
                            return PEACOCK;
                        } else {
                            if (features[10] <= 4.140853f) {
                                if (features[21] <= 8.820612f) {
                                    return WILD_BOAR;
                                } else {
                                    return MONKEY;
                                }
                            } else {
                                return MONKEY;
                            }
                        }
                    } else {
                        if (features[3] <= 4.894433f) {
                            return WILD_BOAR;
                        } else {
                            if (features[2] <= -16.971284f) {
                                return NOISE;
                            } else {
                                if (features[7] <= 6.618885f) {
                                    return MONKEY;
                                } else {
                                    return WILD_BOAR;
                                }
                            }
                        }
                    }
                }
            } else {
                if (features[12] <= 4.637996f) {
                    if (features[6] <= -2.428390f) {
                        return MONKEY;
                    } else {
                        if (features[16] <= 8.633044f) {
                            return MONKEY;
                        } else {
                            return PEACOCK;
                        }
                    }
                } else {
                    return WILD_BOAR;
                }
            }
        }
    } else {
        if (features[24] <= 10.826775f) {
            if (features[0] <= -536.752686f) {
                if (features[10] <= -8.886228f) {
                    if (features[15] <= 56.748425f) {
                        return MONKEY;
                    } else {
                        return PEACOCK;
                    }
                } else {
                    if (features[24] <= 7.087828f) {
                        if (features[6] <= -3.155016f) {
                            return MONKEY;
                        } else {
                            return WILD_BOAR;
                        }
                    } else {
                        return PEACOCK;
                    }
                }
            } else {
                if (features[2] <= -10.782848f) {
                    if (features[4] <= -29.575546f) {
                        return PEACOCK;
                    } else {
                        return MONKEY;
                    }
                } else {
                    if (features[7] <= 15.699990f) {
                        return WILD_BOAR;
                    } else {
                        return MONKEY;
                    }
                }
            }
        } else {
            if (features[16] <= 23.058392f) {
                if (features[23] <= 9.331968f) {
                    if (features[10] <= -0.376899f) {
                        return WILD_BOAR;
                    } else {
                        return MONKEY;
                    }
                } else {
                    return PEACOCK;
                }
            } else {
                if (features[23] <= 12.923589f) {
                    return MONKEY;
                } else {
                    return WILD_BOAR;
                }
            }
        }
    }
}

const char* getClassName(int prediction) {

    switch (prediction) {

        case MONKEY:
            return "MONKEY";

        case PEACOCK:
            return "PEACOCK";

        case WILD_BOAR:
            return "WILD_BOAR";

        case NOISE:
            return "NOISE";

        default:
            return "UNKNOWN";
    }
}

// ============================================================
// SETUP I2S
// ============================================================

void setupI2S() {

  i2s_config_t i2s_config = {

    .mode = (i2s_mode_t)(
      I2S_MODE_MASTER |
      I2S_MODE_RX
    ),

    .sample_rate = SAMPLE_RATE,

    .bits_per_sample =
      I2S_BITS_PER_SAMPLE_32BIT,

    .channel_format =
      I2S_CHANNEL_FMT_ONLY_LEFT,

    .communication_format =
      I2S_COMM_FORMAT_I2S,

    .intr_alloc_flags =
      ESP_INTR_FLAG_LEVEL1,

    .dma_buf_count = 8,

    .dma_buf_len = 64,

    .use_apll = false,

    .tx_desc_auto_clear = false,

    .fixed_mclk = 0
  };


  // Install I2S driver

  esp_err_t result =
    i2s_driver_install(
      I2S_PORT,
      &i2s_config,
      0,
      NULL
    );


  if (result != ESP_OK) {

    Serial.println(
      "ERROR: I2S driver installation failed"
    );

    while (true) {
      delay(1000);
    }
  }


  // I2S pins

  i2s_pin_config_t pin_config = {

    .bck_io_num = I2S_BCLK,

    .ws_io_num = I2S_LRCL,

    .data_out_num =
      I2S_PIN_NO_CHANGE,

    .data_in_num = I2S_DIN
  };


  result = i2s_set_pin(
    I2S_PORT,
    &pin_config
  );


  if (result != ESP_OK) {

    Serial.println(
      "ERROR: I2S pin configuration failed"
    );

    while (true) {
      delay(1000);
    }
  }


  // Clear old samples

  i2s_zero_dma_buffer(I2S_PORT);

  Serial.println("I2S initialized.");
}



// ============================================================
// CAPTURE AUDIO
// ============================================================

void captureAudio() {

  int samplesCaptured = 0;

  int32_t i2sBuffer[512];


  Serial.println();
  Serial.println("Listening...");


  while (
    samplesCaptured < AUDIO_SAMPLES
  ) {

    size_t bytesRead = 0;


    // Read from INMP441

    esp_err_t result = i2s_read(
      I2S_PORT,
      i2sBuffer,
      sizeof(i2sBuffer),
      &bytesRead,
      portMAX_DELAY
    );


    if (result != ESP_OK) {

      Serial.println(
        "ERROR: I2S read failed"
      );

      return;
    }


    int samplesRead =
      bytesRead / sizeof(int32_t);


    // Copy samples into audio buffer

    for (
      int i = 0;
      i < samplesRead &&
      samplesCaptured < AUDIO_SAMPLES;
      i++
    ) {

      // Convert INMP441 32-bit I2S
      // to 16-bit PCM

      int32_t sample =
        i2sBuffer[i] >> 14;


      // Clip

      if (sample > 32767)
        sample = 32767;

      if (sample < -32768)
        sample = -32768;


      audioBuffer[
        samplesCaptured++
      ] = (int16_t)sample;
    }
  }


  Serial.println(
    "Audio capture complete."
  );
}


// ============================================================
// REMOVE DC OFFSET
// ============================================================

void removeDCOffset() {

  int64_t sum = 0;


  for (
    int i = 0;
    i < AUDIO_SAMPLES;
    i++
  ) {

    sum += audioBuffer[i];
  }


  float mean =
    (float)sum / AUDIO_SAMPLES;


  for (
    int i = 0;
    i < AUDIO_SAMPLES;
    i++
  ) {

    float value =
      audioBuffer[i] - mean;


    if (value > 32767)
      value = 32767;

    if (value < -32768)
      value = -32768;


    audioBuffer[i] =
      (int16_t)value;
  }
}

// ============================================================
// MEL SCALE
// ============================================================

float hzToMel(float hz)
{
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

float melToHz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

// ============================================================
// BIT REVERSAL
// ============================================================

void fftBitReverse()
{
    int j = 0;

    for (int i = 1; i < N_FFT; i++)
    {
        int bit = N_FFT >> 1;

        while (j & bit)
        {
            j ^= bit;
            bit >>= 1;
        }

        j ^= bit;

        if (i < j)
        {
            float temp = fftReal[i];
            fftReal[i] = fftReal[j];
            fftReal[j] = temp;

            temp = fftImag[i];
            fftImag[i] = fftImag[j];
            fftImag[j] = temp;
        }
    }
}

// ============================================================
// FFT
// ============================================================

void fft()
{
    fftBitReverse();

    for (int len = 2; len <= N_FFT; len <<= 1)
    {
        float angle = -2.0f * PI / len;

        float wlenReal = cosf(angle);
        float wlenImag = sinf(angle);

        for (int i = 0; i < N_FFT; i += len)
        {
            float wReal = 1.0f;
            float wImag = 0.0f;

            for (int j = 0; j < len / 2; j++)
            {
                int u = i + j;
                int v = i + j + len / 2;

                float vReal =
                    fftReal[v] * wReal -
                    fftImag[v] * wImag;

                float vImag =
                    fftReal[v] * wImag +
                    fftImag[v] * wReal;

                fftReal[v] = fftReal[u] - vReal;
                fftImag[v] = fftImag[u] - vImag;

                fftReal[u] += vReal;
                fftImag[u] += vImag;

                float nextReal =
                    wReal * wlenReal -
                    wImag * wlenImag;

                float nextImag =
                    wReal * wlenImag +
                    wImag * wlenReal;

                wReal = nextReal;
                wImag = nextImag;
            }
        }
    }
}

// ============================================================
// HANN WINDOW
// ============================================================

float hannWindow(int n)
{
    return 0.5f *
           (1.0f -
            cosf((2.0f * PI * n) /
                 (WIN_LENGTH - 1)));
}

// ============================================================
// INITIALIZE MEL FILTER BANK
// ============================================================

void initMelFilters()
{
    float minFreq = 0.0f;
    float maxFreq = SAMPLE_RATE / 2.0f;

    float minMel = hzToMel(minFreq);
    float maxMel = hzToMel(maxFreq);

    float melPoints[N_MELS + 2];

    for (int i = 0; i < N_MELS + 2; i++)
    {
        melPoints[i] =
            minMel +
            (maxMel - minMel) *
            i / (N_MELS + 1);
    }

    // Store bin edges only (melBinsGlobal). computeMFCCFrame() computes each
    // filter's triangular weight on the fly from these edges instead of
    // looking it up in a dense matrix.
    for (int i = 0; i < N_MELS + 2; i++)
    {
        float hz = melToHz(melPoints[i]);

        int bin =
            floorf(
                ((N_FFT + 1) * hz) /
                SAMPLE_RATE
            );

        if (bin < 0)
            bin = 0;

        if (bin > N_FFT / 2)
            bin = N_FFT / 2;

        melBinsGlobal[i] = bin;
    }
}

// ============================================================
// COMPUTE MFCC FOR ONE FRAME
// ============================================================

void computeMFCCFrame(
    int frameIndex,
    int16_t *input
)
{
    int start =
        frameIndex * HOP_LENGTH;

    // --------------------------------------------------------
    // Clear FFT buffer
    // --------------------------------------------------------

    memset(fftReal, 0, sizeof(fftReal));
    memset(fftImag, 0, sizeof(fftImag));

    // --------------------------------------------------------
    // Apply Hann window
    // --------------------------------------------------------

    for (int i = 0; i < WIN_LENGTH; i++)
    {
        fftReal[i] =
            input[start + i] *
            hannWindow(i);
    }

    // --------------------------------------------------------
    // FFT
    // --------------------------------------------------------

    fft();

    // --------------------------------------------------------
    // Power spectrum
    // --------------------------------------------------------

    float powerSpectrum[N_FFT / 2 + 1];

    for (int k = 0; k <= N_FFT / 2; k++)
    {
        powerSpectrum[k] =
            (fftReal[k] * fftReal[k] +
             fftImag[k] * fftImag[k])
            / N_FFT;
    }

    // --------------------------------------------------------
    // Mel filter bank (triangular weight computed on the fly
    // from melBinsGlobal, instead of a stored dense matrix)
    // --------------------------------------------------------

    float melEnergy[N_MELS];

    for (int m = 0; m < N_MELS; m++)
    {
        int left = melBinsGlobal[m];
        int center = melBinsGlobal[m + 1];
        int right = melBinsGlobal[m + 2];

        float energy = 0.0f;

        // Rising slope
        for (int k = left; k < center; k++)
        {
            if (center != left)
            {
                float weight =
                    (float)(k - left) /
                    (float)(center - left);

                energy += powerSpectrum[k] * weight;
            }
        }

        // Falling slope
        for (int k = center; k < right; k++)
        {
            if (right != center)
            {
                float weight =
                    (float)(right - k) /
                    (float)(right - center);

                energy += powerSpectrum[k] * weight;
            }
        }

        // Avoid log(0)
        if (energy < 1e-10f)
            energy = 1e-10f;

        melEnergy[m] =
            logf(energy);
    }

    // --------------------------------------------------------
    // DCT-II
    // --------------------------------------------------------

    for (int c = 0; c < N_MFCC; c++)
    {
        float sum = 0.0f;

        for (int m = 0; m < N_MELS; m++)
        {
            sum +=
                melEnergy[m] *
                cosf(
                    PI *
                    c *
                    (m + 0.5f) /
                    N_MELS
                );
        }

        mfcc[c][frameIndex] = sum;
    }
}

// ============================================================
// EXTRACT FEATURES (mean + std of each MFCC coefficient)
// ============================================================
//
// FIX: original signature was extractFeatures(float *audioInput) but the
// code called it with 3 args and passed the raw int16_t audioBuffer. This
// version takes the int16_t capture buffer directly and passes it straight
// into computeMFCCFrame() (which now does the int16->float promotion itself,
// one window at a time, instead of needing a full-buffer float copy — that
// copy was the main cause of the DRAM overflow at link time). The redundant
// DC-removal step was dropped since removeDCOffset() already runs on
// audioBuffer before this is called.

void extractFeatures(int16_t *audioInput, float *featureOutput)
{
    // --------------------------------------------------------
    // 1. Calculate MFCC per frame
    // --------------------------------------------------------

    for (int frame = 0; frame < N_FRAMES; frame++)
    {
        computeMFCCFrame(frame, audioInput);
    }

    // --------------------------------------------------------
    // 2. Mean + Standard deviation across frames
    // --------------------------------------------------------

    for (int c = 0; c < N_MFCC; c++)
    {
        float sum = 0.0f;

        for (int frame = 0; frame < N_FRAMES; frame++)
        {
            sum += mfcc[c][frame];
        }

        float meanMFCC = sum / N_FRAMES;

        // Standard deviation
        float variance = 0.0f;

        for (int frame = 0; frame < N_FRAMES; frame++)
        {
            float diff = mfcc[c][frame] - meanMFCC;
            variance += diff * diff;
        }

        variance /= N_FRAMES;

        float stdMFCC = sqrtf(variance);

        // First 13 = mean
        featureOutput[c] = meanMFCC;

        // Next 13 = std
        featureOutput[N_MFCC + c] = stdMFCC;
    }
}



// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);


  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "Animal Sound Classifier"
  );

  Serial.println(
    "INMP441 + ESP32"
  );

  Serial.println(
    "========================================"
  );


  // Initialize microphone

  setupI2S();

  // FIX: this was never called, so melBinsGlobal[] stayed all-zero and
  // every mel energy collapsed to log(1e-10) — predictions were garbage
  // even though the code compiled and ran.
  initMelFilters();


  Serial.println();
  Serial.println(
    "Model ready."
  );

  Serial.println(
    "Classes:"
  );

  Serial.println(
    "MONKEY"
  );

  Serial.println(
    "PEACOCK"
  );

  Serial.println(
    "WILD_BOAR"
  );

  Serial.println(
    "NOISE"
  );


  delay(2000);
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // 1. Capture 2 seconds of audio
  // ----------------------------------------------------------

  captureAudio();


  // ----------------------------------------------------------
  // 2. Remove DC offset
  // ----------------------------------------------------------

  removeDCOffset();


  // ----------------------------------------------------------
  // 3. Extract MFCC features
  // ----------------------------------------------------------
  //
  // features[0]  ... features[12]
  //     = MFCC means
  //
  // features[13] ... features[25]
  //     = MFCC standard deviations
  //
  // ----------------------------------------------------------

  extractFeatures(
    audioBuffer,
    features
  );


  // ----------------------------------------------------------
  // 4. Print features for debugging
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("Features:");

  for (
    int i = 0;
    i < NUM_FEATURES;
    i++
  ) {

    Serial.print(
      "features["
    );

    Serial.print(i);

    Serial.print("] = ");

    Serial.println(
      features[i],
      6
    );
  }


  // ----------------------------------------------------------
  // 5. Decision Tree prediction
  // ----------------------------------------------------------

  int prediction =
    predict(features);


  // ----------------------------------------------------------
  // 6. Display prediction
  // ----------------------------------------------------------

  Serial.println();
  Serial.println(
    "============================"
  );

  Serial.print(
    "Prediction: "
  );

  Serial.println(getClassName(prediction));

  Serial.println(
    "============================"
  );


  // ----------------------------------------------------------
  // 7. Small delay before next prediction
  // ----------------------------------------------------------

  delay(500);
}
