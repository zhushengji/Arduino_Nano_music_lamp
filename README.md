# Arduino_Nano_music_lamp
基于Arduinonano的多功能律动灯
支持多种显示模式
* 物料清单
 * 1.主控：Arduino Nano
 * 2.电位器*2
 * 3.按钮*1
 * 4.ws2812灯带*1
 * 4.max9814麦克风*1
 * 接线方式
 * 1.电位器两端引脚不分正反，分别接到5v，gnd，中间引脚一个接A1一个接A2
 * 2.按钮一个引脚接D3，一个接GND
 * 3.ws2812 +接5v，-接GND，DIN接D2
 * 4.max9814：
   ** GND-GND
   ** VDD-5V
   ** Gain-3v3
   ** OUT-A0
   ![硬件及接线](https://user-images.githubusercontent.com/32239713/153748771-4baa29ec-a4e6-4533-9b8b-5c0ad738d294.png)
