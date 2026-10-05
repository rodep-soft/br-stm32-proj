
#include <Arduino.h>
#include <Servo.h>

// --- 設定 ---
constexpr uint8_t SERVO_PIN = 9;       // PWM対応ピン番号
constexpr int MIN_ANGLE = 0;           // 最小角度 [deg]
constexpr int MAX_ANGLE = 180;         // 最大角度 [deg]
constexpr int ANGLE_STEP = 5;          // 1回のキー入力で変化する角度 [deg]

Servo myServo;
int currentAngle = 90;                 // 初期角度（中央値）

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // シリアル接続待機（ Leonardo等のネイティブUSB対応ボード用 ）
    }

    myServo.attach(SERVO_PIN);
    myServo.write(currentAngle);

    Serial.println("--- サーボ制御開始 ---");
    Serial.println("操作方法:");
    Serial.println(" [w] : 角度上昇 (+5 deg)");
    Serial.println(" [s] : 角度下降 (-5 deg)");
    Serial.println(" [q] : 現在位置で静止 / 保持");
    Serial.print("初期角度: ");
    Serial.println(currentAngle);
}

void loop() {
    if (Serial.available() > 0) {
        char key = Serial.read();

        // 改行コード（CR / LF）は無視
        if (key == '\r' || key == '\n') {
            return;
        }

        switch (key) {
            case 'w':
            case 'W':
                currentAngle += ANGLE_STEP;
                if (currentAngle > MAX_ANGLE) {
                    currentAngle = MAX_ANGLE;
                    Serial.print("[上限到達] ");
                }
                myServo.write(currentAngle);
                Serial.print("角度上昇 -> 現在値: ");
                Serial.println(currentAngle);
                break;

            case 's':
            case 'S':
                currentAngle -= ANGLE_STEP;
                if (currentAngle < MIN_ANGLE) {
                    currentAngle = MIN_ANGLE;
                    Serial.print("[下限到達] ");
                }
                myServo.write(currentAngle);
                Serial.print("角度下降 -> 現在値: ");
                Serial.println(currentAngle);
                break;

            case 'h':
            case 'H':


              Serial.println("--- サーボ制御開始 ---");
              Serial.println("操作方法:");
              Serial.println(" [w] : 角度上昇 (+5 deg)");
              Serial.println(" [s] : 角度下降 (-5 deg)");
              Serial.println(" [q] : 現在位置で静止 / 保持");
              Serial.print("初期角度: ");
              Serial.println(currentAngle);

              break;


            case 'q':
            case 'Q':
                // サーボモーターは write() した角度を自動保持するため、
                // 現在角度を再送信して位置を確実に固定・維持
                myServo.write(currentAngle);
                Serial.print("静止（位置保持） -> 現在値: ");
                Serial.println(currentAngle);
                break;

            default:
                // 未定義のキーが押された場合
                break;
        }
    }
}


