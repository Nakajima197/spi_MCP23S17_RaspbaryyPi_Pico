# spi_MCP23S17_RaspbaryyPi_Pico

# 5相のSTEP駆動プログラム

## 機能
 - Raspberry Pi PICO でMCP23S17を駆動する
 - 5chのPWMを生成する
 - ボタン操作でPWMのDutyを変更する
 - ボタン操作でPWMのパターンを変える
 - ボタン操作でPWMのパターンの変更周期を変える
 - LEDで動作確認できる

## 配線
```
PICO  -  MCP23S17
  17   ->    11
  16   <-    14
  19   ->    13
  18   ->    12
  20   ->    18

  21,22,26,27,28 -> ボタン
```

## PWMピン
  PICO GP{5, 6, 7, 8, 9}
