# c542rc-bno085

NUCLEO-C542RC（STM32C542RCT6）でBNO085をSPI接続し、3軸加速度（X/Y/Z）と校正済み3軸角速度（X/Y/Z）をCAN FDへ送信するファームウェア。STM32 HAL2とSH-2/SHTPを使用する。

## CAN FD仕様

| 項目 | 設定 |
|---|---|
| フレーム | CAN FD、標準11 bit ID、データフレーム |
| CAN ID | `0x500`（1280） |
| ペイロード長 | 18 byte（CAN FD DLC 20 byte、末尾2 byteは0パディング） |
| 通信速度 | Nominal 1 Mbps、BRSなし（`FD_NO_BRS`） |
| 送信要求間隔 | 最短5 ms（最大約200 Hz） |
| 再送 | 自動再送なし（DAR）、最新値を単発送信 |
| DBC | [`bno085_can.dbc`](bno085_can.dbc) |

| オフセット | 信号 | 型 / サイズ | 内容・物理値 |
|---|---|---|---|
| 0–3 | `timestamp_ms` | uint32 (4B), little endian | 起動からのミリ秒時刻（`HAL_GetTick()`） |
| 4 | `seq` | uint8 (1B) | CAN送信フレームの連番（256で循環） |
| 5 | `status` | uint8 (1B) | ステータス（`0=INIT`, `1=OK`, `2=WARN`, `3=ERROR`） |
| 6–11 | `accel_x, y, z` | int16 × 3 (6B), little endian | raw × 0.001 m/s² |
| 12–17 | `gyro_x, y, z` | int16 × 3 (6B), little endian | raw × 0.001 rad/s |
| 18–19 | （パディング） | uint8 × 2 (2B) | `0x00`（20 byte DLC整合用） |

- 前回の送信受付後に加速度・角速度の両方が更新され、未完了の送信がない場合に次を送信する。両者の計測時刻を一致させる処理はない。
- ACK欠落・通信エラー・調停負けによる失敗時も同じデータを再送せず、次の送信枠で最新値へ進む。ACK検査とCANのエラー管理は有効。
- 100 ms以上古いサンプル、未来時刻、NaN/Inf、表現範囲外の値は送信しない。未完了要求はサンプルの経過時間100 msで取消を要求する。
- 固定5 ms周期や全サンプルの配送は保証しない。`seq`はCAN送信要求の受付ごとに1加算される。
- PC側 `imu_can_decoder` の18バイトCAN FDフォーマットに対応する。

FDCAN設定：kernel clock 144 MHz、分周1、prescaler 1、TSEG1=125、TSEG2=18、SJW=1、sample point 87.5%。

## 配線・給電

### BNO085

| BNO側の信号／印字 | NUCLEO表記 | STM32ピン | 用途 |
|---|---|---|---|
| INT / H_INTN | A0 | PA0 | Active LOW割り込み入力 |
| RST / RESET | A1 | PA1 | Active LOWリセット出力 |
| CS | A2 | PA4 | Active LOWチップ選択 |
| PS0 / WAKE | A3 | PB0 | 起動時HIGH、通信時の起床制御 |
| SCL / SCK | D13 | PA5 | SPI1 SCK |
| SDA / MISO | D12 | PA6 | SPI1 MISO |
| AD0 / MOSI | D11 | PA7 | SPI1 MOSI |
| PS1 | 3V3 | — | SPIモード選択（HIGH） |
| GND | GND | — | 共通GND |

SDA/SCL/AD0は使用モジュール上の印字。別のモジュールではSPI信号との対応を確認する。
モジュールの電源端子には、そのモジュールの仕様に合う電源を接続する。GPIO信号は3.3 V系。
**PS0/WAKEとPS1は起動時に両方HIGHが必要。A3の接続を省略しない。**
PA5は基板の緑LED LD1とも共有するため、LED用GPIOとして操作しない。

### CAN

基板内MCP2562FDを使用し、外部バスはCN18へ接続する。

| CN18端子 | 接続先 |
|---|---|
| 1 | CAN-H |
| 2 | CAN-L |
| 3 | GND |

終端抵抗はバスの両端に各120 Ω。NUCLEOを終端に置く場合はJP9を装着する。
基板内接続はRX=PB8、TX=PB9、STBY=PE2（LOW）。SB36/RX・SB34/TX・SB32/STBYは標準の接続を使用する。

### USB給電

| 電源入力 | JP1位置 |
|---|---|
| ST-LINK USB（CN1） | STLK：1–2 |
| USER USB（CN13） | USB1：7–8 |

USER USB給電でもBNO/CANは動作する。USER USBの通信機能は実装していないため、PCのシリアルポートとしては認識されない。
診断・書き込みにはST-LINK USBを使う。USB1給電と併用する場合は、USB1から給電してLD7の点灯を確認した後にST-LINK USBをPCへ接続する。
端子番号は基板リビジョンと[ST公式マニュアル](https://www.st.com/resource/en/user_manual/um3615-stm32c5.pdf)を照合する。

## センサー取得・異常時の動作

- SPI1：Mode 3、8 bit、MSB first、2.25 MHz。全二重DMAとGPIOによるCS制御を使用する。
- 加速度・校正済み角速度のレポート要求周期は各5,000 µs。実際の周期はBNOからの設定応答で確認する。
- 起動・再初期化時にジャイロ動的校正を有効化し、読み戻しを確認する。他の校正設定は保持し、校正データの明示的な保存は行わない。
- 精度ステータスだけを理由にCAN送信は停止しない。
- BNOリセットはLOW 10 ms、解除後100 ms待機。両センサーの設定と初回データを確認して送信を開始する。
- BNOリセット通知、SPI通信異常、いずれかのレポートの1秒途絶で、未送信データを破棄して再初期化する。
- 初期化失敗時は1秒後に再試行し、3回連続失敗後は5秒間隔とする。BNOの電源をソフトから切る機能はない。
- CAN Bus-Offは100 msごとに復帰を試みる。
- IWDGのタイムアウトは公称約8秒。メイン処理からのみ更新し、デバッグ停止中も動作する。

内部資源：HSE 24 MHz → SYSCLK 144 MHz、LPDMA1 CH0/CH1=SPI RX/TX、CH2=USART2 TX、EXTI0=BNO INT、TIM2=1 MHz時刻。スタックは4 KiB。

## 診断

ST-LINK VCP（USART2 PA2/PA3、115200 bps、8N1）へ起動状態と5秒周期のHealth情報を出力する。UARTはDMA送信で、バッファ満杯時はログを破棄してセンサー処理を継続する。

| Health項目 | 意味 |
|---|---|
| `online`、`a`、`g`、`acc` | センサー取得状態、加速度／角速度件数、各精度ステータス |
| `queued` | CAN送信要求の受付数 |
| `sent` | ACKを伴う送信完了数 |
| `unsent` | ソフトからの取消要求なしで終了した未送信数（ACK欠落・通信エラー・調停負けなど） |
| `expired` | ソフトからの取消要求後に終了した未送信数 |
| `busoff`、`lec` | Bus-Off検出数、記録された最終CANエラー |
| `initfail`、`recover`、`spi` | BNO初期化失敗数、再初期化数、SPI障害コード |
| `stale`、`invalid`、`uart`、`drop`、`stack_free` | 古い／不正サンプル数、UARTエラー数、ログ破棄数、スタック未使用量 |

`queued`は送信成功数ではない。`lec=3`はSTM32がACKを認識できなかったことを示し、相手機器がACKを出さなかったかどうかまでは判定できない。`lec`は過去のエラーを保持するため、現在の状態は各カウンターの増分と併せて確認する。

## ビルド・書き込み

必要なツール：CMake 3.30以上、Ninja、Arm GNU Toolchain（`arm-none-eabi-gcc/g++/objcopy/size`）、STM32CubeProgrammer（C542対応版。使用実績は2.22.0）。

```sh
cd stm32/c542rc-bno085
make build
```

`build/firmware/` に `c542rc-bno085.elf`、`.bin`、`.hex` を生成する。ST-LINK経由でELFを書き込み・照合し、リセットする。

```sh
STM32_Programmer_CLI -c port=SWD mode=UR freq=1000 \
  -w build/firmware/c542rc-bno085.elf -v -rst
```

CubeMX2では `c542rc-bno085.ioc2` を開き、CMake / GCC / Debugで `c542rc-bno085_cmake` へ生成する。
**生成先を直接ビルドせず、上記Makefileを使う。** CMake hookが生成されたテンプレートmainを `App/main.c` に置き換え、HALのDMA設定を適用する。
再生成時はアプリが使うDMA・TIM2・割り込みを重複割当せず、リンカースクリプトのスタック4 KiBを維持する。
通常運用は `BNO_HARDWARE_SELFTEST=OFF`（既定値）。ONはCAN停止・BNO再初期化・ウォッチドッグ再起動を起こす異常注入試験用。

ホストテスト：

```sh
make test
```

CAN codec、サンプル鮮度、生成設定、SH-2/SHTP境界、単発送信・取消処理、SPI DMAを検査する。Cコンパイラー、Python 3、AddressSanitizer / UndefinedBehaviorSanitizerが必要。

## ソース構成・ライセンス

| パス | 内容 |
|---|---|
| `App/` | センサー取得、DMA、CAN送信、ウォッチドッグ |
| `Drivers/sh2/` | SH-2/SHTPドライバー |
| `protocols/`、`bno085_can.dbc` | CAN codec・信号定義 |
| `c542rc-bno085.ioc2`、`c542rc-bno085_cmake/` | CubeMX2設定、生成コード、ST/CMSIS依存コード |
| `cmake/`、`Makefile` | アプリを組み込むビルド設定 |
| `tests/` | ホストテスト |
| `tools/` | PC側の収録補助ツール |

SH-2/SHTPとCAN codecの元プロジェクトは `../../sandbox/stm32-f303k8`。各ソースおよびST/CMSIS同梱のライセンス表記に従う。
