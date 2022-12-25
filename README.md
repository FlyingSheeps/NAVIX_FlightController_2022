# NAVIX_FlightController_2022
Flying robots club NAVIX at Nagoya University joined the competition with this flight controller system.

NAVIX-X4_Machina.xlsm  
機体設計に用いた諸元などを記載  

NAVIX-X4_Machina  
製図関係データ．AutoCADがあれば.scrファイルをそのまま読み込める． 

Machina_program  
ArduinoIDE用プログラム．ESP32にUSBシリアル変換を用いて書き込む．  
ライブラリとかは各自の環境で用意してください．現状そのまま書き込めないので注意．  
20221215：追記　最新版BolderFlightSystems/SBUSには対応していません．  
簡単には，SbusRxのコンストラクタの記述が変わって，引数に&Serial1の他rxpin,txpin,trueが必要です．

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
パーツは全て秋月で購入可能．

MITライセンスだよ！　まあそのまま使う人いないと思うけど．
