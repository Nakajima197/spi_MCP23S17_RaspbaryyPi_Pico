//
// 5相のSTEP駆動プログラム
//
// 機能
// ・Raspberry Pi PICO でMCP23S17を駆動する
// ・5chのPWMを生成する
// ・ボタン操作でPWMのDutyを変更する
// ・ボタン操作でPWMのパターンを変える
// ・ボタン操作でPWMのパターンの変更周期を変える
// ・LEDで動作確認できる
//
// 配線
// PICO  -  MCP23S17
//  17   ->    11
//  16   <-    14
//  19   ->    13
//  18   ->    12
//  20   ->    18
//
// PWMピン
//  PICO GP{5, 6, 7, 8, 9}
//
// 履歴
// 2024/03/21 ver. 0.1 Nakajima Masao
// 

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "pico/binary_info.h"
#include "hardware/spi.h"

// MCP23S17のレジスタアドレス
#define IODIRA 0x00      // 入出力方向の設定（ポートA）
#define IODIRB 0x01      // 入出力方向の設定（ポートB）
#define GPIOA 0x12       // ポートAのデータ読み書き
#define GPIOB 0x13       // ポートBのデータ読み書き
#define M_WRITE 0x40     // WRITE mode
#define M_READ 0x41      // READ mode
#define A_MASK 0b0000000 // 全て出力
#define B_MASK 0b0000000 // 全て出力

// PWM関連設定
#define max 8        // 8チャネル
#define wrap 4095    // この値でPWMで使えるビット幅が決まる(4095:30KHz)
#define imax 5
#define msdef 250 // 1/4周期[msec]
#define msmax 4000
#define msmin 15 //

#define RESET_PIN 20 // MCP23S17 RESET

#define PI 3.141592654
// wrap値の設定、125KHz(システム・クロックは125MHz 1000(999+1)で割ると125KHz)
// 1023に設定すれば10bitの分解能になる
// 4096 -> 125M / 5000 ≒ 30K (12bit)

/*
GPIO	    0	1	2	3	4	5	6	7
PWMチャネル 0A	0B	1A	1B	2A	2B	3A	3B
GPIO	    8	9	10	11	12	13	14	15
PWMチャネル 4A	4B	5A	5B	6A	6B	7A	7B

https://rikei-tawamure.com/entry/2021/02/08/213335
*/

uint slice_num[max];

// PWMピン
uint8_t ph_pin[5] = {5, 6, 7, 8, 9};

bool LV[imax];
uint8_t ipin[imax] = {21, 22, 26, 27, 28}; // ボタンをつなぐPIN
int ms = msdef, l;
bool L = true, pflag = false;
int J = 0, K = 0;
int duty = wrap / 2;    // 50%
uint16_t sl_time = 200; // 変更周期

// 回転のみの駆動テーブル
// Coil 6-10
// -1 : 反転
//  0 : 停止
//  1 : 正転
signed char ptn[20][5] = {
    {-1, 0, 1, 1, 0},
    {-1, 0, 1, 0, 0},
    {-1, 0, 1, 0, -1},
    {0, 0, 1, 0, -1},
    {0, 1, 1, 0, -1},
    {0, 1, 0, 0, -1},
    {0, 1, 0, -1, -1},
    {0, 1, 0, -1, 0},
    {1, 1, 0, -1, 0},
    {1, 0, 0, -1, 0},
    {1, 0, -1, -1, 0},
    {1, 0, -1, 0, 0},
    {1, 0, -1, 0, 1},
    {0, 0, -1, 0, 1},
    {0, -1, -1, 0, 1},
    {0, -1, 0, 0, 1},
    {0, -1, 0, 1, 1},
    {0, -1, 0, 1, 0},
    {-1, -1, 0, 1, 0},
    {-1, 0, 0, 1, 0}};

// ボタン状態のチェック
void chk_btn()
{
    uint32_t gpio_states = gpio_get_all(); // GPIOピンの状態を一度で取得
    for (int q = 0; q < imax; q++)         // ピン状態の読み取り
    {
        LV[q] = gpio_states & (1u << ipin[q]);
    }

    if (!LV[0]) // ms up left 2
    {
        ms += l;
        if (ms > msmax)
            ms = msmax;
        l += 5; // 押し続けたら加速
        printf("A %d[ms] ", ms);
        pflag = true;
    }
    else if (!LV[2]) // ms down
    {
        ms -= l;
        if (ms < msmin)
            ms = msmin;
        l += 5; // 押し続けたら加速
        printf("B %d ", ms);
        pflag = true;
    }

    else if (!LV[3]) // RESET
    {
        ms = msdef;
        printf("%5.1f[%%] ", (float)duty / (float)wrap * 100);
        duty = wrap / 2;
        printf("R %d[ms] ", ms);

        if (K == 0) // 方向転換
            K = 1;  // 逆転
        else if (K == 1)
        {
            K = 2; // 停止
            printf("=== STOP ===");
        }
        else
            K = 0; // 正転

        printf("Rot: %d ", K); // centor 3

        l = 10;
        pflag = true;
    }

    else if (!LV[4]) // duty up
    {
        duty = duty + l;
        if (duty >= wrap)
            duty = wrap;
        printf("D %5.1f[%%] ", (float)duty / (float)wrap * 100);
        l += 5; // 押し続けたら加速
        pflag = true;
    }
    else if (!LV[1]) // duty down
    {
        duty = duty - l;
        if (duty < 5)
            duty = 5;
        printf("E %5.1f[%%] ", (float)duty / (float)wrap * 100);
        l += 5; // 押し続けたら加速
        pflag = true;
    }
    else
    {
        l = 5; // 加速をリセット
        pflag = false;
    }
}

struct repeating_timer st_timer2;

// ボタン状態取得用タイマー割り込み
bool Timer2(struct repeating_timer *t)
{
    // timer_flag1 = true;
    chk_btn();
    return true;
}

// ２進表示
void print_bin(uint8_t value)
{
    int8_t buf[9] = {0}; // 文字配列をスタックに確保
    for (char i = 0; i < 8; ++i)
    {
        buf[i] = (int8_t)value & (128 >> i) ? '1' : '0'; // 順次 0, 1 文字に変換する
    }
    printf("%s ", buf);
}

void set_mcp_pwm(int n)
{
    uint8_t a = 0, b = 0;
    int ch;
    for (int q = 0; q < 5; q++)
    {
        ch = ph_pin[q] % 2;
        int nn = ptn[n][q];     // 出力レベル
        int mm = ph_pin[q] / 2; // スライス番号
        // printf("%d/%d:", ph_pin[q], ptn[n][q]);
        // printf("%d %d %d %d\n", ph_pin[q], nn, mm, ch);
        if (nn == 0)
        {
            // a = a & (0 << ph_pin[q]);
            pwm_set_chan_level(mm, ch, 0); // PWM off
        }
        else // 1 and -1
        {
            // a = a & (0 << ph_pin[q]);
            pwm_set_chan_level(mm, ch, duty); // PWM (REV or FOW)
            if (nn == 1)
            {
                if (ph_pin[q] < 8)
                {
                    a = a + (1 << ph_pin[q]);
                }
                else
                {
                    b = b + (1 << (ph_pin[q] - 8));
                }
            }
        }
    }
    print_bin(b); // bit 8-15
    print_bin(a); // bit 0-7

    uint8_t out_buf[3];
    out_buf[0] = M_WRITE;
    out_buf[1] = GPIOA;
    out_buf[2] = a;
    spi_write_blocking(spi0, out_buf, 3);

    out_buf[1] = GPIOB;
    out_buf[2] = b;
    spi_write_blocking(spi0, out_buf, 3);

    // printf("%d %5.1f\n", n, ((float)duty / (float)wrap) * 100);

    uint8_t buffer[3], aa, bb;
    out_buf[0] = M_READ;
    out_buf[1] = GPIOA;
    out_buf[2] = 0;
    spi_write_read_blocking(spi0, out_buf, buffer, 3); // PORTA
    aa = buffer[2];

    printf("a %d /", buffer[2]);
    out_buf[1] = GPIOB;
    out_buf[2] = 0;
    spi_write_read_blocking(spi0, out_buf, buffer, 3); // PORTB
    bb = buffer[2];

    printf("%d %5.1f ", n, ((float)duty / (float)wrap) * 100);
    printf("a:%2x b:%2x\n", aa, bb);
}

void spi_start()
{
    // CS
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1); // CS high (inactive)
    // MCP23S17 RESET
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    gpio_put(RESET_PIN, 1);
    sleep_ms(20);
    gpio_put(RESET_PIN, 0); // ここでRESET
    sleep_ms(20);
    gpio_put(RESET_PIN, 1);
    sleep_ms(20);

    printf("SPI master CLK: %d  CS: %d  RX: %d  TX: %d\n",
           PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN);

    // Enable SPI 0 at 1 MHz and connect to GPIOs
    // uint8_t out_buf[3];
    // spi_init(spi_default, 1 * 1000 * 1000); // 1MHz
    spi_init(spi_default, 1 * 1000 * 1000); // 500KHz
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);

    // MCP23S17はSPIモード1で動く
    // spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // SPIモード0
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // SPIモード1
    //  spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST); // SPIモード2
    //  spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST); // SPIモード3
    
    //   Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

    uint8_t out_buf[3];

    out_buf[0] = M_WRITE;
    out_buf[1] = IODIRA;
    out_buf[2] = 0x00; // 全て出力

    spi_write_blocking(spi0, out_buf, 3);

    out_buf[1] = IODIRB;
    out_buf[2] = 0x00; // 全て出力

    spi_write_blocking(spi0, out_buf, 3);
}

// Main
int main()
{
    stdio_init_all();
    sleep_ms(500);

    printf("\nPWM GP0-15 test start\n");

    spi_start();

    // add_repeating_timer_ms(sl_time, Timer1, NULL, &st_timer1); // 時間変更用
    add_repeating_timer_ms(sl_time, Timer2, NULL, &st_timer2); // ボタン状態取得用

    for (int i = 0; i < imax; i++) // GPIO入力ピン
    {
        gpio_init(ipin[i]);
        gpio_set_dir(ipin[i], GPIO_IN);
        gpio_pull_up(ipin[i]);
    }

    // LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    int i;
    bool L = true;

    pwm_set_mask_enabled(0b11111111); // チャネル同期
    sleep_ms(20);

    // GPIO 0 - 15 に PWM に割り当てられるように指示
    for (i = 0; i < max; i++)
    {
        gpio_set_function(i * 2, GPIO_FUNC_PWM);     // *A
        gpio_set_function(i * 2 + 1, GPIO_FUNC_PWM); // *B
        // どの PWM スライスが GPIO に接続されているかを確認
        slice_num[i] = pwm_gpio_to_slice_num(i * 2); // スライスは２つのGPIOで共用

        // 16ビットカウンタなのでwrapに設定できる最大値は65535
        // PWM周波数 f = sysclock / ((wrap+1) * clkdiv)
        // clkdiv = sysclock / ((wrap+1) * f )
        pwm_set_clkdiv(slice_num[i], 1.0); // このクロックの分割はfloat

        pwm_set_wrap(slice_num[i], wrap);                       // wrap値の設定、125KHz(システム・クロックは125MHz 1000(999+1)で割ると500KHz)
        pwm_set_chan_level(slice_num[i], PWM_CHAN_A, 2 / wrap); // チャネル A の出力
        pwm_set_chan_level(slice_num[i], PWM_CHAN_B, 2 / wrap); // チャネル B の出力

        pwm_set_enabled(slice_num[i], true);
    }

    int J = 0;

    // Loop forever
    while (true)
    {
        set_mcp_pwm(J);
        if (K == 0)
        {
            J++; // 正転

            if (J >= 20)
            {
                J = 0;
            }
        }
        else if (K == 1)
        {
            J--; // 逆転
            if (J < 0)
            {
                J = 19;
            }
        }
        else
        {
            ; // Jを変化させない（停止）
        }

        // LED
        L = !L;
        gpio_put(PICO_DEFAULT_LED_PIN, L);
        sleep_ms(ms); // msec
    }
}
