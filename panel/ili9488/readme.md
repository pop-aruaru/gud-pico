# UARTへデバック出力
__#define GUD_DEBUG 1__  
RASPBERRYPI_PICO2の場合、UARTを115200bpsで初期化する。
# IPSパネルを使用する場合
__#define IPS__  
LCDドライバをインバートモードで初期化する  
``参照）5.2.17. Display Inversion ON (21h)``
# BOOT-SELスイッチ
__#define BOOTSEL     26__  
定義したGPIOのスイッチを押すとpicoがダウンロードモードに入る。
XIAOの場合はタッチパネルのIRQを利用。
# RGB565対応
ILI9488は、表示もインターフェースのデータもRGB565を扱えない。
Linuxから送られるデータがRGB565の時はRGB888に変換してLCDドライバに送る