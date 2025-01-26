＃変更点
   #　MCUボードを指定できるように変更
      PICO,PICO2,XIAO-RP2040,XIAO-RP2350
   # VGA LCDの追加
　    panelディレクトリを作成し、LCD DriverにST7796とILI9488を使用した作成、mi0283qtをILI9341として追加
   # RGB888対応
      ili9488とst7796で、RGB666を使用できるようにRGB888を追加
# GPIOの指定
  各ボードのGPIOの指定はcmake時のPICO_BOARDの指定によりdefineされるマクロによって選択するようにmainソースファイルに記述。
  seeedのxiao-rp2040の場合：　#ifdef SEEED_XIAO_RP2040　以下に記述
  以下同様に
  seeedのxiao-rp2450の場合：　#ifdef SEEED_XIAO_RP2350
  raspberrypiのpicoの場合 ：　#ifdef RASPBERRYPI_PICO
  raspberrypiのpico2の場合：　#ifdef RASPBERRYPI_PICO2
## ビルド手順
  buildディレクトリを作成し、cmake、makeで作成する。
  cd build
  cmake -DLCD_TARGET=3 -DPICO_BOARD=pico ソースディレクトリ
  make -j4
  MCUボードごとにbuildディレクトリを分けると良い。
  build-pico2,build-pico,build-xiao_rp2040など
## cmakeコマンド使用例
  cmake  -DPICO_PAORD=seeed_xiao_rp2040 ../
  cmake -DPICO_BOARD=seeed_xiao_rp2350　../
  cmake -DPICO_BOARD=pico2 ../
  cmake -DPICO_BOARD=pico ../
  
# 実行ファイルを書き込む
  picotool load panel/xx/xx.uf2
  picotool reboot
  
# 動作確認

1. gudデバイスのモード確認
   modetest -M gud
   modesに320x240が入っているか確認
   RG16: RGB565
   RG24: RGB888
2. テストパターン
       modetest -M gud -s 35@33:320x240@RG16
       modetest -M gud -s 35@33:320x480@RG16
       modetest -M gud -s 35@33:480x320@XR24
3.
   fbset -fb /dev/fb0 -match
   fbset -fb /dev/fb1 -match
4. JPEGファイルの表示
   sudo fbi -d /dev/fb0 -T 2 ~/Ryogoku_Kokugikan_1909.jpg
   sudo fbi -d /dev/fb0 -T 2 ~/imx217.jpg
   sudo fbi -d /dev/fb0 -T 1 ~/20240803012442.jpg 
   sudo fbi -d /dev/fb1 -T 2 ~/Ryogoku_Kokugikan_1909.jpg
   sudo fbi -d /dev/fb1 -T 2 ~/imx217.jpg
   sudo fbi -d /dev/fb1 -T 1 ~/20240803012442.jpg 
 
5. 画面クリア
   dd if=/dev/zero of=/dev/fb0 > /dev/null 2>&1
   dd if=/dev/zero of=/dev/fb1 > /dev/null 2>&1
6. 動画の表示
   mpv --vo=drm --drm-connector=0.USB-2 ~/sample-5s.mp4
    mpv --drm-device=/dev/fb0 ~/sample-5s.mp4
