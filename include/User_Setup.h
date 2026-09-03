// 防止重复定义 (当 platformio.ini 的 build_flags 中也定义了这些宏时)

#define USER_SETUP_LOADED  // 防止库加载默认的 User_Setup.h
#define ST7789_DRIVER      // 使用 ST7789 驱动
#define TFT_WIDTH  240     // 屏幕宽
#define TFT_HEIGHT 320     // 屏幕高 (根据你的屏幕实际尺寸调整，常见有240*280, 240*320, 135*240)
#define TFT_RGB_ORDER TFT_BGR
#define TFT_INVERSION_OFF  // 关闭反转（ST7789 初始化默认开启反转，此屏幕需要关闭）

// 启用字体（drawString 使用字体 2 需要 LOAD_FONT2）
#define LOAD_GLCD   // 字体 1（默认启用）
#define LOAD_FONT2  // 字体 2（drawString 第4个参数为 2 时使用）


#define INMP441_SCK_PIN   25 // BCLK
#define INMP441_WS_PIN    26 // LRCLK
#define INMP441_SD_PIN    13 // DOUT
#define MAX98357_BCLK_PIN 27 // BCLK
#define MAX98357_LRC_PIN  14 // LRCLK
#define MAX98357_DIN_PIN  22 // DIN


#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5
#define TFT_DC   16
#define TFT_RST  17

#define TFT_BL   4         // 背光引脚
#define TFT_BACKLIGHT_ON HIGH

// ========== 触摸屏 (XPT2046) 配置 ==========
#define TOUCH_CS 21        // 触摸片选引脚 (T_CS)
#define SPI_TOUCH_FREQUENCY  1000000  // XPT2046 触摸 SPI 频率 (建议 1-2.5MHz)



