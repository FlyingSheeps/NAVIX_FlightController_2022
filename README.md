# NAVIX_FlightController_2022
Flying robots maker club NAVIX at Nagoya University joined the competition with this flight controller system.

NAVIX-X4_Machina.xlsm
機体設計に用いた諸元などを記載
一応頑張ってモデル計算しようとした痕跡がある．（今後xflrで計算したモデルに置換予定）

NAVIX-X4_Machina
製図関係データ．AutoCADがあれば.scrファイルをそのまま読み込める．

Machina_program
ArduinoIDE用プログラム．ESP32にUSBシリアル変換を用いて書き込む．
ライブラリとかは各自の環境で用意してください．現状そのまま書き込めないので注意．

machina_PCB
ESP32を用いたフライトコントローラー基板．
microSDスロット
I2C
SBUS端子
PWM6端子
LED2個2系統
ブートボタン
リセットボタン
ESCからの電源をマイコン向け3.3Vに降圧する電源系
を備える．
JLCPCBにmachinev2.zip渡して0.6mm厚二層で頼むと同じやつが届くよ．

MITライセンスだよ！　まあそのまま使う人いないと思うけど．