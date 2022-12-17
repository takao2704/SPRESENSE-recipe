/*
 *  fft_autoencoder.ino - Autoencoder sample for a pipe anomaly detection
 *  Copyright 2022 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

// libraries
#include <Audio.h>
#include <FFT.h>
#include <SDHCI.h>
SDClass SD;

#include <DNNRT.h>
DNNRT dnnrt;


#include <LTE.h>
// initialize the library instance
LTE lteAccess;
LTEScanner scannerNetworks;
LTEClient client;

/*
#include <ArduinoHttpClient.h> 
HttpClient HttpClient; 

#include <ArduinoJson.h>
*/

#include <stdio.h> 

#define FFT_LEN 1024

// APN name
#define APP_LTE_APN "soracom.io"  // replace your APN

/* APN authentication settings
 * Ignore these parameters when setting LTE_NET_AUTHTYPE_NONE.
 */

#define APP_LTE_USER_NAME "sora"  // replace with your username
#define APP_LTE_PASSWORD "sora"   // replace with your password

// APN IP type
#define APP_LTE_IP_TYPE (LTE_NET_IPTYPE_V4V6)  // IP : IPv4v6
// #define APP_LTE_IP_TYPE (LTE_NET_IPTYPE_V4) // IP : IPv4
// #define APP_LTE_IP_TYPE (LTE_NET_IPTYPE_V6) // IP : IPv6

// APN authentication type
#define APP_LTE_AUTH_TYPE (LTE_NET_AUTHTYPE_CHAP)  // Authentication : CHAP
// #define APP_LTE_AUTH_TYPE (LTE_NET_AUTHTYPE_PAP) // Authentication : PAP
// #define APP_LTE_AUTH_TYPE (LTE_NET_AUTHTYPE_NONE) // Authentication : NONE

/* RAT to use
 * Refer to the cellular carriers information
 * to find out which RAT your SIM supports.
 * The RAT set on the modem can be checked with LTEModemVerification::getRAT().
 */

#define APP_LTE_RAT (LTE_NET_RAT_CATM)  // RAT : LTE-M (LTE Cat-M1)
// #define APP_LTE_RAT (LTE_NET_RAT_NBIOT) // RAT : NB-IoT



LTEUDP lteUdp;
char host[] = "harvest.soracom.io";
int port = 8514;
int ad_value = 0;
// URL, path & port (for example: arduino.cc)
//char server[] = "harvest.soracom.io";

unsigned long prev;
static const unsigned long interval = 10000;

// モノラル、1024サンプルでFFTを初期化
FFTClass<AS_CHANNEL_MONO, FFT_LEN> FFT;

AudioClass *theAudio = AudioClass::getInstance();

void avgFilter(float dst[FFT_LEN]) {
  static const int avg_filter_num = 8;
  static float pAvg[avg_filter_num][FFT_LEN];
  static int g_counter = 0;
  if (g_counter == avg_filter_num) g_counter = 0;
  for (int i = 0; i < FFT_LEN; ++i) {
    pAvg[g_counter][i] = dst[i];
    float sum = 0;
    for (int j = 0; j < avg_filter_num; ++j) {
      sum += pAvg[j][i];
    }
    dst[i] = sum / avg_filter_num;
  }
  ++g_counter;
}

void udpsend(int ad_value) {
  Serial.println("UDP Send Start");
  if (lteUdp.begin(port) == 1) {
    if (lteUdp.beginPacket(host, port) == 1) {
      Serial.println("UDP Data make Start");
      char ad_str[10];
      sprintf(ad_str, "ad=%04x", ad_value);
      lteUdp.write(ad_str, 7);
      if (lteUdp.endPacket() == 1) {
        Serial.println("UDP Data Send OK");
        delay(100);
      } else {
        Serial.println("UDP Data Send NG(endPacket)");
      }
    } else {
      Serial.println("UDP Data make NG(beginPacket)");
    }
    lteUdp.stop();
    Serial.println("UDP Send Stop");
  }
}

void ltesetup() {
  char apn[LTE_NET_APN_MAXLEN] = APP_LTE_APN;
  LTENetworkAuthType authtype = APP_LTE_AUTH_TYPE;
  char user_name[LTE_NET_USER_MAXLEN] = APP_LTE_USER_NAME;
  char password[LTE_NET_PASSWORD_MAXLEN] = APP_LTE_PASSWORD;

  Serial.println("LTE networks scanner");

  while (true) {

    //　モデム電源をONしLTE通信機能を有効化する

    if (lteAccess.begin() != LTE_SEARCHING) {
      Serial.println("Could not transition to LTE_SEARCHING.");
      Serial.println("Please check the status of the LTE board.");
      for (;;) {
        sleep(1);
      }
    }

    //　APNへの接続開始
    if (lteAccess.attach(APP_LTE_RAT,
                         apn,
                         user_name,
                         password,
                         authtype,
                         APP_LTE_IP_TYPE)
        == LTE_READY) {
      Serial.println("attach succeeded.");
      break;
    }

    /* If the following logs occur frequently, one of the following might be a cause:
     * - APN settings are incorrect
     * - SIM is not inserted correctly
     * - If you have specified LTE_NET_RAT_NBIOT for APP_LTE_RAT,
     *   your LTE board may not support it.
     * - Rejected from LTE network
     */
    Serial.println("An error has occurred. Shutdown and retry the network attach process after 1 second.");
    lteAccess.shutdown();
    sleep(1);
  }
}


void setup() {
  Serial.begin(115200);
  // SDカードの入力を待つ
  while (!SD.begin()) { Serial.println("Insert SD card"); };

  ltesetup();

  Serial.println("Initialize DNNRT");
  // SDカード上にある学習済モデルを読み込む
  File nnbfile = SD.open("model.nnb");
  ;
  if (!nnbfile) {
    Serial.print("nnb not found");
    while (1)
      ;
  }
  // 学習済モデルでDNNRTを開始する
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.print("DNN Runtime begin fail: " + String(ret));
    while (1)
      ;
  }
  // ハミング窓、モノラル、オーバーラップ50%
  FFT.begin(WindowHamming, AS_CHANNEL_MONO, (FFT_LEN / 2));

  Serial.println("Init Audio Recorder");
  // 入力をマイクに設定
  theAudio->begin();
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC);
  // 録音設定：フォーマットはPCM (16ビットRAWデータ)、
  // DSPコーデックの場所の指定 (SDカード上のBINディレクトリ)、
  // サンプリグレート 48000Hz、モノラル入力
  int err = theAudio->initRecorder(AS_CODECTYPE_PCM,
                                   "/mnt/sd0/BIN", AS_SAMPLINGRATE_48000, AS_CHANNEL_MONO);
  if (err != AUDIOLIB_ECODE_OK) {
    Serial.println("Recorder initialize error");
    while (1)
      ;
  }

  Serial.println("Start Recorder");
  theAudio->startRecorder();  // 録音開始

  //実行時間の初期化
  prev = 0;
}


void loop() {
  static const uint32_t buffering_time =
    FFT_LEN * 1000 / AS_SAMPLINGRATE_48000;
  static const uint32_t buffer_size = FFT_LEN * sizeof(int16_t);
  static const int ch_index = AS_CHANNEL_MONO - 1;
  static char buff[buffer_size];  // 録音データを格納するバッファ
  static float pDst[FFT_LEN];     // FFT演算結果を格納するバッファ
  uint32_t read_size;

  // マイクやファン、パイプ、マイクの状態で数値は大きく変動します
  // 実測をしてみて適切と思われる数値に設定してください
  float maxSpectrum;                   // FFT演算結果を見て調整
  static const float threshold = 1.0;  // RSMEのばらつきを見て調整

  // buffer_sizeで要求されたデータをbuffに格納する
  // 読み込みできたデータ量は read_size に設定される
  int ret = theAudio->readFrames(buff, buffer_size, &read_size);
  if (ret != AUDIOLIB_ECODE_OK && ret != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA) {
    Serial.println("Error err = " + String(ret));
    theAudio->stopRecorder();
    exit(1);
  }

  if (read_size < buffer_size) {
    delay(buffering_time);
    return;
  }

  FFT.put((q15_t *)buff, FFT_LEN);  //FFTを実行
  FFT.get(pDst, 0);                 // FFT演算結果を取得
  avgFilter(pDst);                  // 過去のFFT演算結果で平滑化

  // DNNRTの入力データにFFT演算結果を設定
  DNNVariable input(FFT_LEN / 8);
  float *dnnbuf = input.data();

  //maxSpectrumの算出
  maxSpectrum = 1e-9;
  for (int i = 0; i < FFT_LEN / 8; ++i) {
    maxSpectrum = max(pDst[i], maxSpectrum);
  }

  for (int i = 0; i < FFT_LEN / 8; ++i) {
    pDst[i] /= maxSpectrum;  // 0.0~1.0に正規化
    dnnbuf[i] = pDst[i];
  }
  // 推論を実行
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0);
  // 二乗平均平方根誤差(RSME)を計算
  float sqr_err = 0.0;
  for (int i = 0; i < FFT_LEN / 8; ++i) {
    float err = pDst[i] - output[i];
    sqr_err += sqrt(err * err / (FFT_LEN / 8));
  }

  // RSMEの結果を平均化
  static const int delta_average = 16;  // 平均回数
  static float average[delta_average];
  static uint8_t gCounter = 0;

  average[gCounter++] = sqr_err;
  if (gCounter == delta_average) gCounter = 0;
  float avg_err = 0.0;
  for (int i = 0; i < delta_average; ++i) {
    avg_err += average[i];
  }
  avg_err /= delta_average;
  //  Serial.println("Result: " + String(avg_err, 7));

  // 閾値でOK/NGを判定
  bool bNG = false;
  avg_err > threshold ? bNG = true : bNG = false;
  //  if (bNG) Serial.println("Fault on the machine");

  //  定期的にFFTデータと判定結果をUDPで送信
  unsigned long curr = millis();    // 現在時刻を取得
  if ((curr - prev) >= interval) {  // 前回実行時刻から実行周期以上経過していたら

    Serial.println("UDP Send Start");
    if (lteUdp.begin(port) == 1) {
      if (lteUdp.beginPacket(host, port) == 1) {
        Serial.println("UDP Data make Start");
        //pDstに入っているfloat(4bytes)128個分のデータをudpの送信データに設定する
        lteUdp.write((byte*) pDst,4*128);
        float s_avg_err[1];
        s_avg_err[0] = avg_err;
        lteUdp.write((byte*) s_avg_err,4);
        Serial.println(avg_err);
/** データ確認用シリアル出力
        for (int i = 0; i < 10; ++i) {
          Serial.print(pDst[i]);
          Serial.println();
        }
**/
        if (lteUdp.endPacket() == 1) {
          Serial.println("UDP Data Send OK");
          delay(100);
        } else {
          Serial.println("UDP Data Send NG(endPacket)");
        }
      } else {
        Serial.println("UDP Data make NG(beginPacket)");
      }
      lteUdp.stop();
      Serial.println("UDP Send Stop");
    }
    prev += interval;  // 前回実行時刻に実行周期を加算
  }
}
