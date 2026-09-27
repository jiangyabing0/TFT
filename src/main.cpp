#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <LittleFS.h>
#include <SD.h>
#include <TJpg_Decoder.h>
#include <WiFi.h>         // 核心WiFi库
#include <time.h>          // 时间库
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "soc/soc.h"       // 禁用欠压检测器
#include "soc/rtc_cntl_reg.h" // 禁用欠压检测器
#include "esp_system.h"    // esp_reset_reason(): 读取上次复位原因
#include <ESPAsync_WiFiManager.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <AsyncWebSocket.h>
#include <freertos/stream_buffer.h>  // 音频播放缓冲 (StreamBuffer)
#include <Preferences.h>   // NVS：保存触摸校准参数（掉电不丢）

// 触摸引脚（确保与 User_Setup.h 或实际接线一致）
// 说明：platformio.ini 用 "-include include/User_Setup.h" 强制包含了 User_Setup.h，
// 那里的 TOUCH_CS / SD_CS 才是权威定义；这里只是兜底，用 #ifndef 防止两处定义不一致。
#ifndef TOUCH_CS
#define TOUCH_CS  21
#endif
#define TOUCH_IRQ -1   // 不用可设为 -1
#ifndef SD_CS
#define SD_CS 32
#endif

// 你家的 Wi-Fi 账号密码
const char* ssid = "CMCC-402";
const char* password = "jiangyabing";
// 设置 NTP 服务器（国内建议用阿里云或腾讯云）
const char* ntpServer = "ntp.aliyun.com";
const long  gmtOffset_sec = 8 * 3600; // 中国时区 UTC+8
const int   daylightOffset_sec = 0;   // 无夏令时
const int sample_rate = 16000;
const int bits_per_sample = 16;

TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// 异步 Web 服务器（端口 80）
AsyncWebServer server(80);
AsyncDNSServer dnsServer;      // DNS 服务器（配网门户需要）
// 注意：ESPAsync_WiFiManager 的构造函数会调用 WiFi.mode()，
// 不能在全局构造阶段创建（此时 WiFi 子系统尚未初始化），
// 否则会导致 RTCWDT_RTC_RESET 看门狗复位（启动死循环）。
// 因此这里只声明指针，在 setup() 中创建。
ESPAsync_WiFiManager* wifiManager = nullptr;

// TFT/SPI 互斥锁：Web 服务器任务（/display、/upload、/delete 等路由）与 Arduino loop 任务
// 都会绘制屏幕，而 TFT_eSPI 不是线程安全的，必须串行化访问，防止 SPI 并发冲突导致崩溃/掉线
SemaphoreHandle_t tftMutex = NULL;

// ================= 全局变量与状态 =================
#define MAX_IMAGES 50    // 限制最大扫描数量，防止内存溢出
String imageList[MAX_IMAGES];
int imageCount = 0;
int currentSelectedIndex = -1; // 当前查看的图片索引

enum ViewMode { MODE_LIST, MODE_IMAGE };
ViewMode currentView = MODE_LIST; // 当前界面模式
bool sdCardReady = false;         // SD 卡是否挂载成功
// ================= 绿点反馈相关变量 =================
bool isTouching = false;
uint16_t lastX = 0, lastY = 0;
uint16_t bgBuffer[9][9]; // 缓存绿点区域的背景颜色 (9x9像素)
const int DOT_RADIUS = 4; // 绿点半径
const int DOT_SIZE = DOT_RADIUS * 2 + 1; // 9x9
bool forceClearDot = false; // 强制清除点标志（用于界面切换时）

// ================= 相册列表布局参数 =================
// 屏幕 rotation=1（横屏）时 tft.width()=320、tft.height()=240。
// 这几个常量同时被 drawImageList()（画列表）和 handleTouch()（判断手指点到了哪一行）使用，
// 放在一起能保证两边永远一致，不会出现“看得见却点不中”。
// 标题占 0~LIST_HEADER_H，列表从 LIST_HEADER_H 开始每行 LIST_ROW_H 像素，
// 底部留 LIST_FOOTER_H 给“还有更多文件”的提示。
#define LIST_HEADER_H 46
#define LIST_FOOTER_H 16
#define LIST_ROW_H    18

// 一页最多显示几行（按屏幕实际高度算，换屏或改 rotation 都不用动别的代码）
static int listRowsPerPage() {
  int rows = (tft.height() - LIST_HEADER_H - LIST_FOOTER_H) / LIST_ROW_H;
  if (rows < 1) rows = 1;
  return rows;
}

// 去掉路径开头的 '/'，只留文件名（屏幕显示 / 网页列目录时用）
static String fileNameOf(const String &path) {
  return path.startsWith("/") ? path.substring(1) : path;
}

// ================= 触摸校准 & 坐标方向（修正“触摸位置与显示位置 X/Y 对调”） =================
// 现象：手指按屏幕左边，绿点/选中行却跑到上边（X/Y 对调）；点屏幕右半边干脆没反应。
// 原因：TFT_eSPI 触摸部分内置的是一组 ILI9341 的“示例”校准值
//       （Extensions/Touch.h: x0=300 x1=3600 y0=300 y1=3600, rotate=1, invert_x=开, invert_y=关），
//       其中 rotate=1 的含义是“把触摸读到的 X/Y 对调一次”。
//       本机是 240x320 的 ST7789 + XPT2046，屏幕 rotation=1（横屏）时恰好不需要这次对调，
//       于是被多对调了一次 → 报出来的坐标就成了实际位置的 (y,x)，表现为 X/Y 相反；
//       更麻烦的是：对调后算出的“x”其实是手指的竖直位置，一旦超过屏幕高度 240，就会被
//       getTouch() 里的 `if (x_tmp >= _width || y_tmp >= _height) return false;` 当越界丢掉，
//       于是屏幕右边那一片怎么点都没反应。
// 修正：setup() 里调用 initTouch()，用 tft.setTouch() 装上下面这组本机实测可用的参数：
//       rotate=0（不再对调） + invert_y=开（垂直镜像）。
//       推导过程（对着实测现象反推，两块拼起来就是答案）：
//         ① 用库默认值(rotate=1,invert_x=开)时是“X/Y 对调”  → 说明本屏的触摸原始轴
//            正好是库假设的那两个轴“互换”了（把 rotate 关掉就补回来了）；
//         ② 关掉对调后又变成“水平、垂直都镜像”（相当于整屏转了 180°）
//            → 在 rotate=0 的公式里，把两个镜像位都翻转即可，
//              即 invert_x 由“开”改“关”、invert_y 由“关”改“开” → flags=0x04。
//       若哪块屏还有个别像素级偏差，串口发 c 做一次四点校准即可一劳永逸。
// 参数格式与 TFT_eSPI 的 calibrateTouch()/setTouch() 完全一致：
//   [0]=x0  [1]=x1(跨度)  [2]=y0  [3]=y1(跨度)  [4]=标志位
//   标志位 bit0=rotate(交换 X/Y)  bit1=invert_x(水平镜像)  bit2=invert_y(垂直镜像)
// 参数存在 NVS(Preferences) 里，掉电不丢；串口监视器（115200）随时可以调，改完立刻生效：
//   c = 跑一次四点校准（库会自动测出全部参数并存盘，最准，建议先做一次）
//   x = 交换/取消交换 X/Y      h = 水平镜像开关     v = 垂直镜像开关
//   p = 打印当前参数           r = 恢复出厂默认
//   d = 开关调试输出（打印 原始值 → 屏幕坐标，用来判断是对调还是镜像）
//   s<y> = 模拟点击屏幕 y 处（如 s60），用来验证点击链路，不需要手指
// 调好的判断标准：手指按哪里，绿点就出现在哪里；四个角都点得到。
#define TOUCH_Z_THRESHOLD 600   // 压力阈值：手指没按时 z 只有 200~400 左右
#define IMG_BAR_H 30            // 图片界面底部“返回”灰条的高度（绘制与命中判断共用）
// 默认参数版本号：每次调整下面 TOUCH_CAL_DEFAULT 就 +1。
// 作用：NVS 里存的是“带版本号”的参数，版本对不上就丢弃旧的、改用新的默认值，
//       否则改了代码重新烧写，开机还是读回上一次存错的参数，会以为改动没生效。
#define TOUCH_CAL_VERSION 2

static const uint16_t TOUCH_CAL_DEFAULT[5] = {300, 3600, 300, 3600, 0x04};
static uint16_t touchCalData[5];
Preferences touchPrefs;

void printTouchConfig() {
  Serial.printf("[触摸] 校准参数 x0=%u x1=%u y0=%u y1=%u flags=0x%02X (交换X/Y=%d 水平镜像=%d 垂直镜像=%d)\n",
                touchCalData[0], touchCalData[1], touchCalData[2], touchCalData[3], touchCalData[4],
                (touchCalData[4] & 0x01) ? 1 : 0, (touchCalData[4] & 0x02) ? 1 : 0, (touchCalData[4] & 0x04) ? 1 : 0);
}

// 保存当前参数到 NVS（下次开机自动生效）
void saveTouchConfig() {
  touchPrefs.putBytes("cal", touchCalData, sizeof(touchCalData));
  touchPrefs.putUChar("ver", TOUCH_CAL_VERSION);   // 记下版本，避免升级后被旧参数覆盖
}

// 把参数装进 TFT_eSPI 并记下（tft.setTouch() 只是记参数，不画屏）
void applyTouchConfig() {
  tft.setTouch(touchCalData);
  saveTouchConfig();
}

// setup() 里调用：优先用 NVS 里存过的参数，没有（或版本对不上）就用新的默认值
void initTouch() {
  bool ok = touchPrefs.begin("touch", false);
  if (!ok) Serial.println("[触摸] NVS 打开失败，使用默认校准参数");

  memcpy(touchCalData, TOUCH_CAL_DEFAULT, sizeof(touchCalData));

  uint8_t ver = touchPrefs.getUChar("ver", 0);
  if (ver == TOUCH_CAL_VERSION && touchPrefs.getBytesLength("cal") == sizeof(touchCalData)) {
    touchPrefs.getBytes("cal", touchCalData, sizeof(touchCalData));
    Serial.println("[触摸] 使用 NVS 里保存的校准参数");
  } else {
    // 首次运行，或者固件里的默认参数改过（版本号变了）：
    // 旧版参数直接作废，改用新的默认值并落盘。这样改完代码重新烧写就能立刻看到效果。
    touchPrefs.putBytes("cal", touchCalData, sizeof(touchCalData));
    touchPrefs.putUChar("ver", TOUCH_CAL_VERSION);
    Serial.println("[触摸] 使用新的默认校准参数（串口发 c 可做精确校准）");
  }

  tft.setTouch(touchCalData);
  printTouchConfig();
  Serial.println("[触摸] 若绿点不跟手：串口发 c 做四点校准；x=对调X/Y  h=水平镜像  v=垂直镜像  r=恢复默认  p=打印参数  d=调试输出  s<y>=模拟点击");
}

// 调试开关：'d' 打开后，每次触摸都会打印“原始值 → 换算后的屏幕坐标”，串口里限速 5 次/秒。
// 用途：手指按住屏幕上某个已知位置（例如左上角），看打印出来的坐标就能立刻判断
//       是“X/Y 对调”还是“水平/垂直镜像”，一次把参数改对：
//         raw 的 X 变大时屏幕 x 反而变小 → 水平镜像(invert_x) 方向不对
//         raw 的 Y 变大时屏幕 y 反而变小 → 垂直镜像(invert_y) 方向不对
static bool touchDebug = false;
void printTouchDebug(uint16_t x, uint16_t y) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 200) return;   // 限速，别把串口刷爆
  lastMs = millis();
  uint16_t rx = 0, ry = 0;
  tft.getTouchRaw(&rx, &ry);             // 仅在调试时多读一次原始值
  Serial.printf("[触摸调试] 原始 raw=(%u,%u) → 屏幕 (%u,%u)  屏幕=%dx%d flags=0x%02X\n",
                rx, ry, x, y, tft.width(), tft.height(), touchCalData[4]);
}

// ===== 串口模拟点击（调试用，见 handleTouchSerial() 的 's' 命令）=====
// 串口发 “s60”（数字是屏幕 y 坐标）＝ 模拟一次“手指按在 (屏幕中间, y) 上再抬起”。
// 它不会绕过任何逻辑：readScreenTouch() 直接把坐标喂给 updateTouchFeedback()/handleTouch()，
// 和真手指走的是同一条路径（绿点反馈 → 命中判断 → 状态机 → 松手复位），
// 因此可以用来在没实物/没法触屏时验证“点列表出不出图、点返回条能不能回去”，
// 也是本次“第一次点击之后再也点不动”那个 bug 的现成回归测试。
static int8_t   simTouchPhase = 0;   // 0=不模拟 1=按下（本轮循环） 2=抬起（下一轮循环自动结束）
static uint16_t simTouchX = 0, simTouchY = 0;

// 触摸的统一读取入口：返回的坐标就是“屏幕坐标”（和 tft.drawXXX 同一套坐标系）
// 绿点反馈与翻页/返回的命中判断都走这里，两边不可能再各用一套坐标而“点不中”。
bool readScreenTouch(uint16_t *x, uint16_t *y) {
  if (simTouchPhase == 1) {          // 模拟按下：直接给出坐标，其余流程完全照旧
    *x = simTouchX;
    *y = simTouchY;
    return true;
  }
  uint16_t tx = 0, ty = 0;
  if (!tft.getTouch(&tx, &ty, TOUCH_Z_THRESHOLD)) return false;
  *x = tx;
  *y = ty;
  if (touchDebug) printTouchDebug(tx, ty);
  return true;
}


// ================= 1. SD卡基础读写功能 =================
// SD 卡和 TFT/触摸共用同一条 SPI 总线（VSPI: SCK=18, MISO=19, MOSI=23），
// 靠各自的 CS 分时复用，所以这里只能复用同一个 SPI 对象，不能再 begin 一条新总线。
//
// 挂载失败常见原因（按出现概率排序）：
//   ① 卡的文件系统不是 FAT16/FAT32 —— ESP32 的 FatFs 没有开 exFAT（FF_FS_EXFAT=0），
//      64GB 以上的 SDXC 卡默认就是 exFAT，必须重新格式化成 FAT32；
//   ② SPI 速率太高 —— 20MHz 在杜邦线/面包板上很容易失败；
//   ③ 接线不对（CS=32 / SCK=18 / MISO=19 / MOSI=23 / 3V3 / GND，MISO 必须接回 ESP32）；
//   ④ 供电不足 —— 卡初始化瞬间电流可达 100mA+，要和 TFT 共用稳定的 3.3V。
//
// 因此下面按 20 → 10 → 4 → 1 MHz 逐档降速重试，并把驱动内部的报错打印出来
// （需要 platformio.ini 里的 -DCORE_DEBUG_LEVEL=2，否则 sd_diskio 的 log_e/log_w 不会输出）。
static const uint32_t SD_SPI_SPEEDS[] = {20000000, 10000000, 4000000, 1000000};
static const int SD_SPI_SPEED_COUNT = sizeof(SD_SPI_SPEEDS) / sizeof(SD_SPI_SPEEDS[0]);

// 卡类型名字，方便在串口里区分是 MMC / 标准 SD / 高容量卡
static const char* sdCardTypeName(sdcard_type_t type) {
  switch (type) {
    case CARD_MMC:  return "MMC";
    case CARD_SD:   return "SDSC";
    case CARD_SDHC: return "SDHC/SDXC";
    default:        return "UNKNOWN";
  }
}

bool initSDCard() {
  // 拉高其他设备的 CS 引脚，防止总线冲突
  pinMode(TFT_CS, OUTPUT); digitalWrite(TFT_CS, HIGH); 
  pinMode(TOUCH_CS, OUTPUT); digitalWrite(TOUCH_CS, HIGH); 

  // SD 卡的 CS 也要先置为输出并拉高，和 TFT/触摸的 CS 一起保证总线空闲
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);

  Serial.printf("[SD] SPI 引脚: SCK=%d MISO=%d MOSI=%d CS=%d\n", TFT_SCLK, TFT_MISO, TFT_MOSI, SD_CS);

  for (int i = 0; i < SD_SPI_SPEED_COUNT; i++) {
    uint32_t hz = SD_SPI_SPEEDS[i];

    // 上一次失败可能已经在驱动内部注册了卷/挂载点，先彻底卸载再重试
    SD.end();
    delay(50);

    if (!SD.begin(SD_CS, SPI, hz)) {
      Serial.printf("[SD] %u MHz 挂载失败，自动降速重试\n", (unsigned)(hz / 1000000));
      continue;
    }

    sdcard_type_t type = SD.cardType();
    uint64_t sizeMB = SD.cardSize() / (1024ULL * 1024ULL);
    if (type == CARD_NONE || sizeMB == 0) {
      // begin() 返回 true 但卡信息读不到，视为失败，继续降速
      Serial.printf("[SD] %u MHz 挂载后卡信息异常，继续降速重试\n", (unsigned)(hz / 1000000));
      SD.end();
      delay(50);
      continue;
    }

    Serial.printf("[SD] 挂载成功：速率=%u MHz 类型=%s 容量=%llu MB\n",
                  (unsigned)(hz / 1000000), sdCardTypeName(type), sizeMB);
    return true;
  }

  // 所有速率都失败：把排查方向直接打印出来，省得来回猜
  Serial.println("[SD] 所有速率均挂载失败！请依次检查：");
  Serial.println("     ① 卡格式：必须是 FAT16/FAT32（用 SD Card Formatter 格式化，Windows 里选 FAT32）；64G 以上 SDXC 默认 exFAT，ESP32 不支持");
  Serial.println("     ② 接线：CS->GPIO32, SCK->GPIO18, MISO->GPIO19, MOSI->GPIO23, VCC->3V3, GND->GND（MISO 必须接回 ESP32）");
  Serial.println("     ③ 供电：卡初始化瞬时电流大，3V3 就近加 100uF 电容；纯 3.3V 模块不要接 5V");
  Serial.println("     ④ 看上面驱动打印：[E][sd_diskio.cpp] Card Failed!/GO_IDLE_STATE failed = 卡没应答(接线/供电/坏卡)；f_mount failed:(13) = 文件系统不对");
  return false;
}

void writeFile(const char *path, const char *message) {
  File file = SD.open(path, FILE_WRITE);
  if (file) { file.print(message); file.close(); Serial.println("写入成功"); }
}

void appendFile(const char *path, const char *message) {
  File file = SD.open(path, FILE_APPEND);
  if (file) { file.print(message); file.close(); Serial.println("追加成功"); }
}

void readFile(const char *path) {
  File file = SD.open(path);
  if (file) {
    while (file.available()) Serial.write(file.read());
    file.close();
  }
}

void deleteFile(const char *path) {
  if (SD.remove(path)) Serial.println("删除成功");
}

// ================= 2. 相册列表逻辑 =================
// 取出文件在 SD 卡里的完整路径（形如 "/1.jpg"）。
// 这里必须用 path() 而不是 name()：
//   name()  = pathToFileName(path())，只返回最后一段文件名 "1.jpg"，
//             没有开头的 '/'；而 ESP32 的 SD 是基于 VFS 的，
//             VFSImpl::open() 会直接拒绝不以 '/' 开头的路径（打日志
//             "%s does not start with /"），于是 SD.open("1.jpg") 永远失败。
//   结果就是：列表能列出来，但一点开就 "Jpeg file not found"（图片区域全黑）。
String sdFilePath(File &file) {
  String p = file.path();
  if (!p.startsWith("/")) p = "/" + p;
  return p;
}

void scanSDCard() {
  imageCount = 0;
  currentSelectedIndex = -1;

  File root = SD.open("/");
  if (!root) {
    Serial.println("[SD] 打开根目录失败");
    return;
  }

  File file = root.openNextFile();
  while (file && imageCount < MAX_IMAGES) {
    String path = sdFilePath(file);
    String lowerPath = path; lowerPath.toLowerCase();
    // 过滤出图片文件，并跳过 macOS 自带的 "._xxx.jpg" 隐藏文件
    if (!file.isDirectory() &&
        (lowerPath.endsWith(".jpg") || lowerPath.endsWith(".jpeg")) &&
        !lowerPath.startsWith("/_")) {
      imageList[imageCount] = path;
      imageCount++;
    }
    // 及时关闭：SD 默认同时只能打开 5 个文件（max_files=5），
    // 不关的话第 6 张图之后 openNextFile() 会直接返回空，扫描就断了
    file.close();
    file = root.openNextFile();
  }
  file.close();
  root.close();

  Serial.printf("扫描到 %d 张图片\n", imageCount);
  for (int i = 0; i < imageCount; i++) {
    Serial.printf("      [%d] %s\n", i, imageList[i].c_str());
  }
}

// SD 卡挂载失败时屏幕上的排查提示（也要放在 setup() 最后画，否则同样会被 WiFi 提示擦掉）
void drawSdErrorScreen() {
  xSemaphoreTake(tftMutex, portMAX_DELAY);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("SD Card Mount Failed!", 10, 10);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("1) Format card as FAT32", 10, 45);
  tft.drawString("2) Check CS=32 / wiring", 10, 70);
  tft.drawString("3) Check 3.3V power", 10, 95);
  xSemaphoreGive(tftMutex);
}

void drawImageList() {
  // 画屏幕要和 Web 任务串行（TFT_eSPI 不是线程安全的）
  xSemaphoreTake(tftMutex, portMAX_DELAY);

  const int rows = listRowsPerPage();
  const int w = tft.width();

  tft.fillScreen(TFT_BLACK);

  // ---- 标题 ----
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("SD Photo Album", 10, 8);
  // 分隔线画在 y = LIST_HEADER_H - 2 = 44：
  // 右上角的时间每秒都会用“带背景色”的方式重绘（y=25~41），
  // 线画在 40 就会被时间文字的背景一个像素一个像素地啃掉一段。
  tft.drawLine(0, LIST_HEADER_H - 2, w, LIST_HEADER_H - 2, TFT_WHITE);

  // ---- 一张照片都没有 ----
  if (imageCount == 0) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("No .jpg on SD card", 10, LIST_HEADER_H + 16);
    tft.setTextSize(1);
    xSemaphoreGive(tftMutex);
    return;
  }

  // ---- 文件列表：一行一个（行位置和 handleTouch() 用同一套常量算） ----
  tft.setTextSize(1);
  for (int i = 0; i < imageCount && i < rows; i++) {
    tft.setCursor(10, LIST_HEADER_H + i * LIST_ROW_H);
    // 当前正在看的 / 刚看过的用绿色标出来，方便一眼找到
    tft.setTextColor(i == currentSelectedIndex ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
    tft.print(i + 1);
    tft.print(". ");
    tft.print(fileNameOf(imageList[i]));   // 只显示文件名，"1.jpg" 比 "/1.jpg" 清爽
  }

  if (imageCount > rows) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("... more files not shown", 10, tft.height() - LIST_FOOTER_H);
  }

  xSemaphoreGive(tftMutex);
}


// ================= JPEG 渲染回调（TJpg_Decoder 的“画笔”） =================
// TJpg_Decoder 本身【不】认识 TFT_eSPI：它只负责解码，每解出一块像素（MCU）就调用
// 这个回调函数交给 sketch 自己往屏幕上画。这个回调必须用 TJpgDec.setCallback() 注册，
// 因为库内部保存它的成员 tft_output 的初始值是 nullptr（见 TJpg_Decoder.h:121）；
// 忘了注册就会在解出第一块像素时跳到地址 0 —— 串口打印
// "Guru Meditation Error: Core 1 panic'ed (InstrFetchProhibited) PC=0x00000000"，
// 然后芯片立刻重启。这正是“点击屏幕列表就崩溃复位”的原因。
// 返回值：1 = 继续解码下一块；0 = 让解码器提前结束（drawSdJpg() 此时返回 JDR_INTR）。
bool tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  // 图片比屏幕还高时，从这一块开始已经跑到屏幕下方了，直接停止解码，省掉大量无用功
  if (y >= tft.height()) return 0;

  // pushImage() 会自动把超出屏幕的部分裁掉，这里不用自己做边界判断。
  // 颜色字节序已经在解码时由 TJpgDec.setSwapBytes(true) 换好了（见 setup()）。
  tft.pushImage(x, y, w, h, bitmap);

  return 1; // 继续解码下一块
}

void showSelectedImage(int index) {
  if (index < 0 || index >= imageCount) return;
  
  // 绘制屏幕必须和 Web 任务（/display、/delete）串行，否则 SPI 会互相打架
  xSemaphoreTake(tftMutex, portMAX_DELAY);

  Serial.printf("[图片] 显示 #%d: %s\n", index, imageList[index].c_str());

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Loading...", 10, 10, 2);

  // 关键：给 SD 卡让出总线
  digitalWrite(TFT_CS, HIGH); 
  digitalWrite(TOUCH_CS, HIGH);

  // 使用 TJpg_Decoder 从 SD 卡绘制图片
  // imageList[] 里存的是带 '/' 的完整路径（如 "/1.jpg"），SD.open() 才能打开；
  // 解码/打开失败时屏幕上给出提示，方便区分是文件问题还是接线问题
  uint32_t t0 = millis();
  JRESULT jres = TJpgDec.drawSdJpg(0, 0, imageList[index]);
  // JDR_INTR(1) 不是失败：图片比屏幕高时 tftOutput() 会返回 0 让解码提前结束，
  // 此时图片已经画出来了，和 JDR_OK 一样算成功；只有其它返回值才是真的出错。
  if (jres != JDR_OK && jres != JDR_INTR) {
    // JDR_INPUT_FMT(2)=不是 JPEG、JDR_FS_READ_ERROR(5)=SD 读失败、JDR_NOT_OPENED(9)=文件打不开
    Serial.printf("[图片] 打开/解码失败(%d): %s\n", (int)jres, imageList[index].c_str());
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("Decode failed!", 10, 40, 2);
  } else {
    Serial.printf("[图片] 显示完成: %s  结果=%d  用时 %lu ms\n",
                  fileNameOf(imageList[index]).c_str(), (int)jres, (unsigned long)(millis() - t0));
  }

  // 底部绘制返回提示（按屏幕真实高度定位：横屏 320x240 时 y=290 已经在屏幕外了）
  // 高度用 IMG_BAR_H，和 handleTouch() 里的命中判断共用同一个常量，保证“点得到”
  const int barY = tft.height() - IMG_BAR_H;
  tft.fillRect(0, barY, tft.width(), IMG_BAR_H, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.drawString("Tap HERE to return", 50, barY + 7, 2);

  xSemaphoreGive(tftMutex);
}
// 保存背景并画绿点
void drawTouchDot(uint16_t x, uint16_t y) {
    // 绿点区域的背景是【刚刚从屏幕上读回来】的，所以它是有效可还原的：
    // 把“界面已重绘、不要还原旧背景”的标志清掉，
    // 不然会出现“图片上粘着一颗擦不掉的绿点”（标志还留着 true，
    // 松手时 restoreTouchDot() 直接跳过还原）。
    forceClearDot = false;

    // 边界约束：防止画到屏幕边缘外导致内存越界
    int startX = constrain(x - DOT_RADIUS, 0, tft.width() - DOT_SIZE);
    int startY = constrain(y - DOT_RADIUS, 0, tft.height() - DOT_SIZE);
    lastX = startX + DOT_RADIUS; // 更新实际的中心点
    lastY = startY + DOT_RADIUS;

    // 1. 读取并保存背景颜色
    for (int i = 0; i < DOT_SIZE; i++) {
        for (int j = 0; j < DOT_SIZE; j++) {
            bgBuffer[i][j] = tft.readPixel(startX + i, startY + j);
        }
    }

    // 2. 画绿点
    tft.fillCircle(lastX, lastY, DOT_RADIUS, TFT_GREEN);
}

// 恢复背景（擦除绿点）
void restoreTouchDot() {
    if (forceClearDot) {
        forceClearDot = false; // 界面已重绘，无需恢复旧背景
        return;
    }
    int startX = lastX - DOT_RADIUS;
    int startY = lastY - DOT_RADIUS;
    
    // 将缓存的背景颜色写回屏幕
    for (int i = 0; i < DOT_SIZE; i++) {
        for (int j = 0; j < DOT_SIZE; j++) {
            tft.drawPixel(startX + i, startY + j, bgBuffer[i][j]);
        }
    }
}

// 核心触摸更新逻辑（代替原来的 delay(200) 防抖）
void updateTouchFeedback() {
    uint16_t x, y;
    // 统一走 readScreenTouch()：内部已经装好校准参数，返回的就是屏幕坐标。
    // （原来直接调 tft.getTouch()，用的是 TFT_eSPI 内置的 ILI9341 示例参数，
    //   里面 rotate=1 会多做一次 X/Y 对调 → 这就是“触摸位置和显示位置对调”的根源）
    bool currentlyTouching = readScreenTouch(&x, &y);

    if (currentlyTouching) {
        if (!isTouching) {
            // 第一次按下
            isTouching = true;
            drawTouchDot(x, y);
        } else if (abs(x - lastX) > 2 || abs(y - lastY) > 2) {
            // 手指移动超过2像素，先恢复旧位置，再在新位置画点
            restoreTouchDot();
            drawTouchDot(x, y);
        }
    } else {
        // 松开手指
        if (isTouching) {
            restoreTouchDot();
            isTouching = false;
        }
    }
}
// ================= 3. 触摸与状态机 =================
// 【重要修复】actionTriggered 必须定义在函数作用域，原因：
//   原来的写法是在 if (isTouching) { ... } 和 else { ... } 两个大括号里
//   各写了一句 “static bool actionTriggered = false;”，而 C++ 里块作用域的 static
//   是两块各自独立的变量：if 分支里那个被置为 true 之后，永远不可能被 else 分支里的
//   “actionTriggered = false” 复位（改的是另一个变量）。
//   表现就是：开机后第一次按压之后，屏幕上再怎么点都没有反应 ——
//   点列表不出图、点 “Tap HERE to return” 也回不到列表。
//   现在提到函数外面只留一份，松开手指时就能正确复位。
static bool actionTriggered = false;   // 本次按压是否已经处理过（防止一次按压重复触发）
void handleTouch() {
  // 每次循环先处理绿点反馈
  updateTouchFeedback();

  // 如果当前有触摸动作，才去判断是否触发翻页/返回逻辑
  if (isTouching) {
    // 简单防抖：只在第一次按下的瞬间触发逻辑，移动时不反复触发
    if (!actionTriggered) {
      actionTriggered = true;
      
      if (currentView == MODE_LIST) {
        // 点击列表区域：行位置必须用 drawImageList() 画每一行时的同一套几何参数来算。
        // 原来写死的 45 / (45 + 15 * 18) / 18 是竖屏时代的数字，有三个问题：
        //   ① 45 与真实的标题高度 LIST_HEADER_H(46) 差 1 像素；
        //   ② 15 行 × 18 = 315 超过横屏高度 240，连最后一行以外的空白也算“第 N 行”；
        //   ③ 行号因此整体偏移 —— 表现为“看得见却点不中 / 点错行”。
        const int visibleRows = listRowsPerPage();
        if (lastY >= LIST_HEADER_H && lastY < LIST_HEADER_H + visibleRows * LIST_ROW_H) {
          int row = (lastY - LIST_HEADER_H) / LIST_ROW_H;
          if (row >= 0 && row < imageCount) {
            currentSelectedIndex = row;
            currentView = MODE_IMAGE;
            // 串口留个痕迹：点了第几行、要打开哪个文件，方便对照“是不是点了却没反应”
            Serial.printf("[触摸] 列表点击: 屏幕 y=%u → 第 %d 行 -> %s\n",
                          lastY, row, imageList[row].c_str());
            
            // 界面即将完全重绘，强制取消绿点恢复，防止花屏
            forceClearDot = true;
            isTouching = false; 
            
            showSelectedImage(row); // 显示图片
          } else {
            Serial.printf("[触摸] 该行没有图片: row=%d, 已扫描 %d 张（一页显示 %d 行）\n",
                          row, imageCount, visibleRows);
          }
        } else {
          Serial.printf("[触摸] 没点到列表行: y=%u，有效范围 %d~%d（一页 %d 行）\n",
                        lastY, LIST_HEADER_H, LIST_HEADER_H + visibleRows * LIST_ROW_H - 1, visibleRows);
        }
      } 
      else if (currentView == MODE_IMAGE) {
        // 点击底部灰条返回：灰条顶边是 tft.height() - IMG_BAR_H（横屏 240 时 = 210）。
        // 原来判的是 lastY > 280，在 240 高的横屏上永远不成立 → 点“Tap HERE to return”返回不了。
        if (lastY >= tft.height() - IMG_BAR_H) {
          currentView = MODE_LIST;
          Serial.printf("[触摸] 点击返回条 (y=%u >= %d) → 回到列表\n", lastY, tft.height() - IMG_BAR_H);
          
          // 界面即将完全重绘，强制取消绿点恢复
          forceClearDot = true;
          isTouching = false; 
          
          drawImageList();
        }
      }
    }
  } else {
    // 松开后重置触发标志（这里改的就是上面那个函数级变量，下一轮按压才能再次触发）
    actionTriggered = false;
  }

  // 串口模拟点击的收尾：本轮循环按“按下”处理完，下一轮就按“抬起”来处理，
  // 让 updateTouchFeedback() 走一遍真正的松手路径（恢复绿点 + isTouching=false），
  // 这样状态机的“释放复位”逻辑也被覆盖到，和真手指的效果一致。
  if (simTouchPhase == 1)      simTouchPhase = 2;
  else if (simTouchPhase == 2) simTouchPhase = 0;
}


// ================= 4. 触摸校准 / 方向微调（串口命令触发） =================
// 四点校准：依次点屏幕上四个角出现的箭头，TFT_eSPI 会根据测到的原始值
// 自动判断触摸轴是否需要 X/Y 对调、是否需要水平/垂直镜像（见 Extensions/Touch.cpp
// 里 calibrateTouch() 的 “touchCalibration_rotate = false; if(abs(...) > abs(...))” 一段），
// 所以它能把“X/Y 对调”连同偏移、镜像一起纠正过来；结果存进 NVS，下次开机自动生效。
void runTouchCalibration() {
  xSemaphoreTake(tftMutex, portMAX_DELAY);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Touch Calibration", 10, 10);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Tap each arrow and HOLD until the next one appears", 10, 45);
  tft.drawString("order: top-left, bottom-left, top-right, bottom-right", 10, 61);
  xSemaphoreGive(tftMutex);
  delay(1500);

  // 注意：calibrateTouch() 会一直等到四个角都点完才返回（没点就卡在这里），
  //       只有串口主动发 'c' 才会走到这，所以不会影响正常开机。
  // 画箭头 + 读触摸的全过程都必须独占 TFT/SPI（TFT_eSPI 不是线程安全的）：
  // 这期间 Web 里会画屏的请求（/display、/delete）会稍微等一下，代价可以接受，
  // 不加锁的话两个任务同时用 SPI 会把屏幕画花、触摸采样也会被干扰。
  uint16_t newCal[5] = {0, 0, 0, 0, 0};
  xSemaphoreTake(tftMutex, portMAX_DELAY);
  tft.calibrateTouch(newCal, TFT_MAGENTA, TFT_BLACK, 15);
  memcpy(touchCalData, newCal, sizeof(touchCalData));
  tft.setTouch(touchCalData);
  saveTouchConfig();

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Calibrated & saved", 10, 60);
  xSemaphoreGive(tftMutex);

  printTouchConfig();
  delay(1000);

  // 校准过程把界面画花了，按当前状态重画一次
  forceClearDot = true;
  if (!sdCardReady) {
    drawSdErrorScreen();
  } else if (currentView == MODE_LIST || currentSelectedIndex < 0) {
    drawImageList();
  } else {
    showSelectedImage(currentSelectedIndex);
  }
}

// 串口命令（115200）：实时开关 X/Y 对调、镜像，或跑四点校准
//   c = 四点校准并保存      x = 交换/取消交换 X/Y
//   h = 水平镜像开关        v = 垂直镜像开关
//   p = 打印当前参数        r = 恢复出厂默认
//   d = 开关触摸调试输出（打印 原始值 → 屏幕坐标，用来判断对调/镜像）
//   s<y> = 模拟一次点击：s60 表示“点屏幕 y=60 的地方”，走的是和真手指一样的代码路径
void handleTouchSerial() {
  while (Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 'c':
        Serial.println("[触摸] 开始四点校准：请依次点屏幕四个角出现的箭头");
        runTouchCalibration();
        break;
      case 'd':
        touchDebug = !touchDebug;
        Serial.printf("[触摸] 调试输出: %s\n", touchDebug ? "开（按住屏幕看串口打印）" : "关");
        break;
      case 'x':
        touchCalData[4] ^= 0x01;   // bit0: 交换 X/Y
        applyTouchConfig();
        printTouchConfig();
        break;
      case 'h':
        touchCalData[4] ^= 0x02;   // bit1: 水平镜像
        applyTouchConfig();
        printTouchConfig();
        break;
      case 'v':
        touchCalData[4] ^= 0x04;   // bit2: 垂直镜像
        applyTouchConfig();
        printTouchConfig();
        break;
      case 'p':
        printTouchConfig();
        break;
      case 's': {
        // 模拟点击：后面跟屏幕 y 坐标，例如 s60（x 自动取屏幕中线）
        uint16_t y = 0;
        while (Serial.available()) {
          int d = Serial.peek();
          if (d < '0' || d > '9') break;
          y = (uint16_t)(y * 10 + (Serial.read() - '0'));
        }
        simTouchX = tft.width() / 2;
        simTouchY = y;
        simTouchPhase = 1;
        Serial.printf("[触摸] 模拟点击 (x=%u, y=%u)：下一轮循环里按下并抬起一次\n", simTouchX, simTouchY);
        break;
      }
      case 'r':
        memcpy(touchCalData, TOUCH_CAL_DEFAULT, sizeof(touchCalData));
        applyTouchConfig();
        Serial.println("[触摸] 已恢复默认参数");
        printTouchConfig();
        break;
      default:
        break;   // 换行、其它字符一律忽略
    }
  }
}


// 时间显示区域（避免闪烁的关键：只更新变化的区域）
// 时间显示在屏幕右上角，避免与图片重叠
#define DATE_X 200
#define DATE_Y 5
#define TIME_X 200
#define TIME_Y 25


void printLocalTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("获取时间失败");
    return;
  }
  
  // 格式化时间字符串
  char timeStr[20];
  char dateStr[20];
  // 24小时制: 时:分:秒
  strftime(timeStr, 20, "%H:%M:%S", &timeinfo);
  // 日期: 年-月-日
  strftime(dateStr, 20, "%Y-%m-%d", &timeinfo);

  // 只在时间变化时才更新屏幕，避免不必要的重绘
  static char lastTimeStr[20] = "";
  static char lastDateStr[20] = "";
  
  bool timeChanged = (strcmp(timeStr, lastTimeStr) != 0);
  bool dateChanged = (strcmp(dateStr, lastDateStr) != 0);
  
  if (!timeChanged && !dateChanged) {
    return; // 时间和日期都没变，不重绘
  }
  
  // 需要刷新屏幕区域：加 TFT 互斥锁（防止与 Web 任务中的 displayImage 并发绘制）
  xSemaphoreTake(tftMutex, portMAX_DELAY);

  // 日期变化时才更新日期
  if (dateChanged) {
    strcpy(lastDateStr, dateStr);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(dateStr, DATE_X, DATE_Y, 2);
  }
  
  // 时间变化时才更新时间
  if (timeChanged) {
    strcpy(lastTimeStr, timeStr);
    // 使用带背景色的文字绘制，避免先擦除再绘制造成的闪烁
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(timeStr, TIME_X, TIME_Y, 2);
    tft.setTextSize(1); // 恢复字号
  }

  // 释放 TFT 互斥锁
  xSemaphoreGive(tftMutex);
}

// 从 LittleFS 读取控制网页（HTML）
String getControlPage() {
  if (!LittleFS.exists("/index.html")) {
    return "<h1>index.html 不存在，请先上传 data 文件夹</h1>";
  }
  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    return "<h1>无法打开 index.html</h1>";
  }
  String html = file.readString();
  file.close();
  return html;
}

AsyncWebSocket wsAudio("/audio");
// 下面三个标志会被不同任务读写（Arduino loop 任务 / async_tcp 回调 / 播放任务），加 volatile 防止被优化
volatile bool isPlaying = false; // 状态标志：true=播放(说)，false=录音(听)
volatile bool wsAudioConnected = false; // 是否有网页连上了 /audio
volatile bool micStreamEnabled = true;  // 网页是否希望接收 ESP32 麦克风的声音（网页可发 "listen:0/1" 关闭）

// 每个音频包 1024 采样点 = 2048 字节 = 16kHz 下约 64ms（与网页端的包长一致）
#define AUDIO_CHUNK_BYTES 2048
// 功放播放缓冲大小（16kHz/16bit 单声道 ≈ 32KB/s，这里留 8KB ≈ 250ms 抗抖动）
#define AUDIO_SPK_BUFFER_BYTES 8192

// 待播放的音频数据：WebSocket 回调只做“非阻塞入队”，由独立的播放任务写 I2S。
// 千万不要在 WebSocket 回调里调用 i2s_write(portMAX_DELAY)：
// 回调运行在 async_tcp 任务里，阻塞它会导致网页卡死、看门狗复位，
// 以及 WebSocket 发送队列积压（终端会打印
// "[E][AsyncWebSocket.cpp] Too many messages queued: closing connection" 并把连接踢掉）。
StreamBufferHandle_t spkBuffer = NULL;
TaskHandle_t spkTaskHandle = NULL;

// ================= I2S 初始化 =================
bool initI2S() {
  // 1. 初始化 I2S 输入（麦克风）
  i2s_config_t i2s_in_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sample_rate,
    .bits_per_sample = i2s_bits_per_sample_t(bits_per_sample),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512
  };
  i2s_pin_config_t in_pins = {
    .bck_io_num = INMP441_SCK_PIN,
    .ws_io_num = INMP441_WS_PIN,
    .data_out_num = -1,
    .data_in_num = INMP441_SD_PIN
  };
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_in_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[语音] 麦克风 I2S 初始化失败 (err=%d)，请检查引脚是否被占用\n", err);
    return false;
  }
  i2s_set_pin(I2S_NUM_0, &in_pins);

  // 2. 初始化 I2S 输出（功放）
  i2s_config_t i2s_out_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = sample_rate,
    .bits_per_sample = i2s_bits_per_sample_t(bits_per_sample),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512
  };
  i2s_pin_config_t out_pins = {
    .bck_io_num = MAX98357_BCLK_PIN,
    .ws_io_num = MAX98357_LRC_PIN,
    .data_out_num = MAX98357_DIN_PIN,
    .data_in_num = -1
  };
  err = i2s_driver_install(I2S_NUM_1, &i2s_out_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[语音] 功放 I2S 初始化失败 (err=%d)，请检查引脚是否被占用\n", err);
    return false;
  }
  i2s_set_pin(I2S_NUM_1, &out_pins);
  return true;
}

// ================= 功放开/关（说/听 切换） =================
void setAudioMode(bool play) {
  isPlaying = play;
  // 注意：部分MAX98357模块高电平开启，部分低电平开启。如果反了，请交换 HIGH 和 LOW
  digitalWrite(MUTE_PIN, play ? HIGH : LOW);
  Serial.println(play ? "切换到【说】模式：功放开启" : "切换到【听】模式：麦克风录音");
}

// ================= 功放播放任务 =================
// 只做一件事：从缓冲里取数据写 I2S。与 Web 服务器任务彻底解耦，
// 因此 WebSocket 回调不会被 I2S 的阻塞写入卡住。
void spkTask(void *param) {
  static uint8_t buf[1024];   // 1024 字节 = 16ms 音频（4 字节为 L/R 一帧，必须是 4 的整数倍）
  for (;;) {
    if (spkBuffer == NULL) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // 不在【说】模式：把缓冲区里的残留数据取走丢掉，
    // 保证下次开口时从干净状态开始（不会先播出上一次的旧声音）
    if (!isPlaying) {
      uint8_t trash[256];
      xStreamBufferReceive(spkBuffer, trash, sizeof(trash), pdMS_TO_TICKS(100));
      continue;
    }

    size_t n = xStreamBufferReceive(spkBuffer, buf, sizeof(buf), portMAX_DELAY);
    if (n == 0) continue;
    size_t bytes_written = 0;
    i2s_write(I2S_NUM_1, buf, n, &bytes_written, portMAX_DELAY);
  }
}

// ================= WebSocket 事件回调 =================
// 协议约定（与 data/index.html 保持一致）：
//   文本帧 "start" -> 打开功放（网页按住说话）
//   文本帧 "stop"  -> 关闭功放，恢复麦克风
//   文本帧 "listen:0/1" -> 关闭/打开 ESP32 麦克风推流
//   二进制帧       -> 网页麦克风的 PCM 数据（16kHz/16bit/单声道/小端）
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
      case WS_EVT_CONNECT:
        wsAudioConnected = true;
        setAudioMode(false);   // 新页面接入，先回到【听】模式
        Serial.printf("[语音] 网页已连接 (id=%u, IP=%s)\n", client->id(),
                      client->remoteIP().toString().c_str());
        break;

      case WS_EVT_DISCONNECT:
        wsAudioConnected = false;
        setAudioMode(false);
        Serial.printf("[语音] 网页已断开 (id=%u)，等待重连...\n", client->id());
        break;

      case WS_EVT_ERROR:
        Serial.printf("[语音] WebSocket 出错 (id=%u)\n", client->id());
        break;

      case WS_EVT_DATA: {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        // 注意：不能再用 len < 5 之类的长度猜类型，
        // 否则 5 个字节的 "start" 会被当成音频数据（这就是之前功放不响的原因）。
        bool isText = (info->index == 0 && info->opcode == WS_TEXT);
        bool isBinary = (info->index == 0 && info->opcode == WS_BINARY) ||
                        (info->index > 0 && info->message_opcode == WS_BINARY);
        if (isText) {
          String cmd((char *)data, len);   // 按长度构造，不依赖 '\0' 结尾
          cmd.trim();
          if (cmd == "start") {
            setAudioMode(true);            // 网页按住说话，ESP32 打开功放播放
          } else if (cmd == "stop") {
            setAudioMode(false);           // 网页松开，ESP32 关闭功放，进入录音模式
          } else if (cmd.startsWith("listen:")) {
            micStreamEnabled = (cmd.substring(7).toInt() != 0);
            Serial.printf("[语音] 麦克风推流: %s\n", micStreamEnabled ? "开" : "关");
          }
        } else if (isBinary) {
          // 分片重组：TCP 可能把一个 WebSocket 消息拆成几段回调，
          // 按 info->index 偏移拼进重组缓冲，凑齐最后一个分片再送功放。
          // （不做重组的话，拆包时会把半截数据写进 I2S，出现“咔咔”杂音）
          static uint8_t accum[8192];
          if (info->index + len <= sizeof(accum)) {
            memcpy(accum + info->index, data, len);
            if (info->final) {                       // 本消息的最后一个分片
              size_t total = info->index + len;      // 该消息的总长度
              if (isPlaying && spkBuffer != NULL && total > 0 && (total % 4) == 0) {
                // 非阻塞写入（最多等 20ms）：写不下就丢掉这一包。
                // 丢包只会让声音卡一下，不会把 WebSocket 撑爆导致断连。
                xStreamBufferSend(spkBuffer, accum, total, pdMS_TO_TICKS(20));
              }
            }
          } else if (len > sizeof(accum)) {
            Serial.printf("[语音] 音频包过大(%u 字节)，已忽略（请刷新网页）\n", (unsigned)len);
          }
        }
        break;
      }

      default:
        break;
    }
}


// 打印上次复位原因，用于区分“手机访问导致断网”是单纯 WiFi 掉线还是芯片复位重启
void printResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  const char* reasonStr = "未知";
  switch (r) {
    case ESP_RST_POWERON:  reasonStr = "上电复位"; break;
    case ESP_RST_SW:       reasonStr = "软件复位(ESP.restart)"; break;
    case ESP_RST_PANIC:    reasonStr = "程序异常崩溃复位"; break;
    case ESP_RST_INT_WDT:  reasonStr = "中断看门狗复位"; break;
    case ESP_RST_TASK_WDT: reasonStr = "任务看门狗复位(任务卡死超时)"; break;
    case ESP_RST_WDT:      reasonStr = "其他看门狗复位"; break;
    case ESP_RST_BROWNOUT: reasonStr = "欠压复位(供电不足!)"; break;
    default: break;
  }
  Serial.printf("[启动] 上次复位原因=%d (%s)\n", (int)r, reasonStr);
}

void setup() {
  // 禁用欠压检测器（Brownout Detector），防止供电不足时芯片复位
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);

  // 打印上次复位原因（配合串口监视器判断“断网”是单纯掉线还是芯片复位重启）
  printResetReason();

  // 创建 TFT/SPI 访问互斥锁（Web 服务器任务与 loop 任务共享屏幕）
  tftMutex = xSemaphoreCreateMutex();
  tft.init();

  // JPEG 解码器配置（两行都不能少）：
  //   ① setCallback() 注册渲染回调 —— 库内部函数指针默认是 nullptr，
  //      不注册的话一解码就跳转到地址 0，直接 Guru Meditation 重启（点击列表崩溃的元凶）；
  //   ② setSwapBytes(true) —— 解码出来的是 RGB565，字节序和 SPI 送屏需要的相反，
  //      不换的话颜色会红蓝对调（库在解码时就把两个字节换好，不用再动 tft）。
  // 缩放倍数：默认 0 = 原尺寸 1:1（本屏 320x240 会显示照片左上角一块，最清晰）；
  // 想让一屏看到更多内容可以改成 TJpgDec.setJpgScale(1) → 1/2、2 → 1/4、3 → 1/8。
  TJpgDec.setCallback(tftOutput);
  TJpgDec.setSwapBytes(true);

  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);

  // 初始化触摸屏
  // 说明：坐标读取现在统一走 TFT_eSPI（见 readScreenTouch()），ts.begin() 只负责
  //       初始化 SPI 并把 TOUCH_CS 拉高，所以不再需要 ts.setRotation()。
  ts.begin();
  // 关键一步：装入触摸校准参数（NVS 优先），修正“触摸位置与显示位置 X/Y 对调”
  initTouch();

  // 初始化 LittleFS 文件系统（存放图片）
  if (!LittleFS.begin()) {
    Serial.println("LittleFS 初始化失败！");
  } else {
    Serial.println("LittleFS 初始化成功");
  }

  // 2. 初始化 SD 卡
  sdCardReady = initSDCard();
  if (sdCardReady) {
    // 测试基础读写（可选择注释掉）
    writeFile("/test.txt", "Hello ESP32 SD!\n");
    appendFile("/test.txt", "Appended line.\n");

    // 3. 扫描 SD 卡里的图片，把文件名读进内存。
    //    注意这里【不】画列表：下面连接 WiFi 时会 fillScreen(TFT_BLACK)，
    //    提前画出来的列表会被配网/连接提示擦掉，屏幕上最后只剩个时间。
    //    列表统一放在 setup() 的最后一步画（见函数末尾）。
    scanSDCard();
  }
  if (!sdCardReady) Serial.println("[SD] 挂载失败，无法显示图片列表");

  // 1. 设置屏幕配网提示
  // ==========================================
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("正在尝试连接 WiFi...", 10, 10, 2);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("若失败将自动创建", 10, 35, 2);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("热点: ESP32_Photo", 10, 60, 2);

  // ==========================================
  // 2. 核心：自动连接/配网逻辑
  // ==========================================
  // 使用 ESPAsync_WiFiManager 库（khoih-prog）
  // 它会先尝试连接已保存的 WiFi，失败则自动创建配网热点
  
  // 在 setup() 中创建 WiFiManager 实例（不能在全局构造阶段创建，
  // 因为其构造函数会调用 WiFi.mode()，此时 WiFi 子系统尚未初始化）
  wifiManager = new ESPAsync_WiFiManager(&server, &dnsServer);
  
  // 设置配网超时（秒），防止长时间阻塞触发看门狗复位
  wifiManager->setConfigPortalTimeout(120);
  
  // 给配网界面设置一个名字 (ESP32_Photo) 和一个密码 (可选，设为空就是无密码)
  bool res = wifiManager->autoConnect("ESP32_Photo", "12345678"); 

  
  if (!res) {
    // 连不上 WiFi、配网也超时了。
    // 这里保留原来的“重启重试”逻辑（重启后又会重新开放 2 分钟配网热点，是唯一的重配途径），
    // 但重启前【先把相册列表画出来】：没有网络时屏幕上也应该能列出 SD 卡里的照片，
    // 否则相册功能在断网时完全不可见，看起来就像“SD 挂载成功但屏幕没有列表”。
    Serial.println("[WiFi] 连接/配网失败：先显示相册列表，10 秒后重启重试");
    if (sdCardReady) {
      drawImageList();
      xSemaphoreTake(tftMutex, portMAX_DELAY);
      tft.fillRect(0, tft.height() - LIST_FOOTER_H, tft.width(), LIST_FOOTER_H, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.drawString("WiFi failed - rebooting in 10s", 10, tft.height() - LIST_FOOTER_H + 3);
      xSemaphoreGive(tftMutex);
      // 这 10 秒里照样处理触摸，方便没网的时候也能点开照片看看
      unsigned long t0 = millis();
      while (millis() - t0 < 10000) {
        handleTouch();
        delay(20);
      }
    } else {
      drawSdErrorScreen();
      delay(3000);
    }
    ESP.restart();
  }

  // 连上 WiFi 了！
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN);
  tft.drawString("Wi-Fi 连接成功!", 10, 10, 2);
  tft.drawString("IP: " + WiFi.localIP().toString(), 10, 40, 2);
  delay(1500);

  // ---- WiFi 稳定性优化：解决“手机访问页面导致 ESP32 断网”的问题 ----
  // ① 关闭 modem sleep（ESP32 默认开启）：让 WiFi 射频保持常开，
  //    避免省电休眠时漏收路由器 beacon 被判“AP 丢失”而主动断开。
  //    —— 手机靠近 ESP32 时发射功率大会压制其接收，这一步是首选修复
  WiFi.setSleep(false);

  // ② 断线自动重连，并在串口打印断开原因码，方便排查
  //    reason 含义：2=AUTH_EXPIRE  4=ASSOC_EXPIRE  15=握手超时(路由器侧)
  //                 200=BEACON_TIMEOUT  201=NO_AP_FOUND  203=ASSOC_FAIL(收不到beacon/信号弱)
  //                 205=AUTH_FAIL(密码错误)
  WiFi.setAutoReconnect(true);
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      uint8_t reason = info.wifi_sta_disconnected.reason;
      const char* desc = "其他";
      if (reason == 2)        desc = "AUTH_EXPIRE";
      else if (reason == 4)   desc = "ASSOC_EXPIRE";
      else if (reason == 15)  desc = "4WAY_HANDSHAKE_TIMEOUT(路由器问题)";
      else if (reason == 200) desc = "BEACON_TIMEOUT(收不到beacon)";
      else if (reason == 201) desc = "NO_AP_FOUND(找不到路由器)";
      else if (reason == 203) desc = "ASSOC_FAIL(信号弱/被手机压制)";
      else if (reason == 205) desc = "AUTH_FAIL(密码错误)";
      Serial.printf("[WiFi] 连接断开! reason=%u (%s)\n", (unsigned)reason, desc);
      WiFi.reconnect(); // 保险起见主动重连（setAutoReconnect 也会自动处理）
    }
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // ③ 降低 WiFi 发射功率（默认最大 20.5dBm → 11dBm）：
  //    减小发射瞬间的峰值电流（缓解供电不足/欠压），并减轻对同频手机信号的压制；
  //    局域网内访问控制网页，11dBm 覆盖完全足够
  WiFi.setTxPower(WIFI_POWER_11dBm);
  Serial.println("[WiFi] 稳定性优化已启用: setSleep(false)/autoReconnect/TX=11dBm");
  // ==========================================



  // 2. 初始化 NTP 时间
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime(); // 显示一次当前时间

  // 3. 配置 Web 服务器路由
  // 控制网页
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", getControlPage());
  });
  // 浏览器(尤其是手机浏览器)会自动请求 /favicon.ico，直接返回 204 No Content，
  // 减少额外的并发请求处理，降低服务器与 WiFi 负担
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(204);
  });
  server.on("/resetWifi", HTTP_GET, [](AsyncWebServerRequest *request){
    if (wifiManager) {
      wifiManager->resetSettings(); // 擦除保存的 WiFi 密码
    }
    ESP.restart();               // 重启，让设备重新进入配网模式
    request->send(200, "text/plain", "已重置，设备将重启");
  });

  // ---------- 图片相关路由（数据源统一换成 SD 卡） ----------
  // 说明：原来这几条路由是给 LittleFS 用的，改到 SD 卡的那次改动把它们删掉了，
  //       于是网页上的“上传/列表/显示/删除”四个按钮全部 404。这里按原协议补回来。

  // 显示指定图片：GET /display?img=1.jpg
  server.on("/display", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("img")) {
      request->send(400, "text/plain", "缺少 img 参数");
      return;
    }
    String img = request->getParam("img")->value();
    if (!img.startsWith("/")) img = "/" + img;   // SD.open() 必须用 "/xxx.jpg"

    // 先在启动时扫描出来的列表里找
    int idx = -1;
    for (int i = 0; i < imageCount; i++) {
      if (imageList[i] == img) { idx = i; break; }
    }
    // 不在列表里（比如刚上传、还没重新扫描）：确认文件存在后补进列表
    if (idx < 0) {
      if (!SD.exists(img)) {
        request->send(404, "text/plain", "图片不存在: " + img);
        return;
      }
      if (imageCount >= MAX_IMAGES) {
        request->send(507, "text/plain", "图片数量已达上限(" + String(MAX_IMAGES) + ")");
        return;
      }
      imageList[imageCount] = img;
      idx = imageCount;
      imageCount++;
    }

    currentSelectedIndex = idx;
    currentView = MODE_IMAGE;      // 屏幕切到图片模式，和触摸操作保持一致
    showSelectedImage(idx);
    request->send(200, "text/plain", "已显示: " + fileNameOf(img));
  });

  // 列出 SD 卡里的图片：GET /list
  // 直接返回启动时扫描好的内存列表，避免在 Web 任务里再翻一次 SD 目录（少一次 SPI 争用）
  server.on("/list", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "[";
    for (int i = 0; i < imageCount; i++) {
      if (i) json += ",";
      json += "\"";
      json += fileNameOf(imageList[i]);   // 返回文件名，和网页上传时用的名字对得上
      json += "\"";
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  // 删除图片：POST /delete?img=1.jpg
  server.on("/delete", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("img")) {
      request->send(400, "text/plain", "缺少 img 参数");
      return;
    }
    String img = request->getParam("img")->value();
    if (!img.startsWith("/")) img = "/" + img;

    // 只允许删图片，防止误删 index.html 之类的重要文件
    String lower = img; lower.toLowerCase();
    if (!lower.endsWith(".jpg") && !lower.endsWith(".jpeg")) {
      request->send(400, "text/plain", "只允许删除 JPG 图片");
      return;
    }

    // SD 和 TFT 共用一条 SPI 总线，而烧写/擦除会占用较长时间，加锁串行
    xSemaphoreTake(tftMutex, portMAX_DELAY);
    bool ok = SD.exists(img) && SD.remove(img);
    xSemaphoreGive(tftMutex);

    if (!ok) {
      request->send(404, "text/plain", "删除失败（文件不存在？）: " + fileNameOf(img));
      return;
    }

    // 同步移除内存列表里的这一项，保证屏幕列表和 SD 卡一致
    for (int i = 0; i < imageCount; i++) {
      if (imageList[i] == img) {
        for (int j = i; j < imageCount - 1; j++) imageList[j] = imageList[j + 1];
        imageCount--;
        if (currentSelectedIndex >= imageCount) currentSelectedIndex = imageCount - 1;
        break;
      }
    }
    if (currentView == MODE_LIST) drawImageList();   // 正停在列表界面就立刻刷新
    Serial.println("[SD] 已删除图片: " + img);
    request->send(200, "text/plain", "已删除: " + fileNameOf(img));
  });

  // 上传图片：POST /upload (multipart/form-data)，字段名 img
  server.on("/upload", HTTP_POST,
    [](AsyncWebServerRequest* request) {
      request->send(200, "text/plain", "上传完成");
    },
    [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
      static File uploadFile;
      static String uploadPath;

      if (!index) {                               // 收到新文件的第一个数据块
        // 只保留文件名，挡掉 "../" 之类的路径穿越
        int slash = filename.lastIndexOf('/');
        if (slash >= 0) filename = filename.substring(slash + 1);
        uploadPath = "/" + filename;

        String lower = uploadPath; lower.toLowerCase();
        if (!lower.endsWith(".jpg") && !lower.endsWith(".jpeg")) {
          Serial.println("[上传] 只支持 JPG 图片，已忽略: " + filename);
          return;                                 // uploadFile 保持无效，后面的块会被丢掉
        }
        uploadFile = SD.open(uploadPath, FILE_WRITE);
        if (!uploadFile) {
          Serial.println("[上传] 无法创建文件: " + uploadPath);
          return;
        }
        Serial.println("[上传] 开始: " + uploadPath);
      }

      if (uploadFile) uploadFile.write(data, len);

      if (final && uploadFile) {
        uploadFile.close();
        Serial.println("[上传] 完成: " + uploadPath);

        // 补进内存列表（已存在就复用原索引），然后立刻显示在屏幕上
        int idx = -1;
        for (int i = 0; i < imageCount; i++) {
          if (imageList[i] == uploadPath) { idx = i; break; }
        }
        if (idx < 0 && imageCount < MAX_IMAGES) {
          imageList[imageCount] = uploadPath;
          idx = imageCount;
          imageCount++;
        }
        if (idx >= 0) {
          currentSelectedIndex = idx;
          currentView = MODE_IMAGE;
          showSelectedImage(idx);
        }
      }
    });




  // 1. 初始化功放静音引脚（先静音，避免上电“砰”的一声）
  pinMode(MUTE_PIN, OUTPUT);
  isPlaying = false;
  digitalWrite(MUTE_PIN, LOW);

  // 2. 初始化 I2S（麦克风 + 功放）
  bool i2sOK = initI2S();
  Serial.printf("[语音] I2S 初始化: %s\n", i2sOK ? "成功" : "失败");

  // 3. 创建功放播放缓冲 + 播放任务
  //    （把 I2S 写入从 Web 服务器任务里挪出来，避免阻塞网页服务/WebSocket）
  spkBuffer = xStreamBufferCreate(AUDIO_SPK_BUFFER_BYTES, 1);
  if (spkBuffer != NULL) {
    xTaskCreatePinnedToCore(spkTask, "spk_task", 4096, NULL, 1, &spkTaskHandle, 0);
  } else {
    Serial.println("[语音] 播放缓冲创建失败，网页说话功能不可用");
  }

  // 启动服务器
  server.begin();
  Serial.print("Web 服务器已启动，访问 http://");
  Serial.println(WiFi.localIP());

  // 4. 挂载 WebSocket 和 WebServer
  wsAudio.onEvent(onWsEvent);
  server.addHandler(&wsAudio);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();
  Serial.println("服务器已启动");

  // 5. 所有初始化都做完了，最后才把相册列表画到屏幕上。
  //    必须放在最后：前面配网/连接 WiFi 的提示都会 fillScreen(TFT_BLACK)，
  //    提前画列表会被这些提示直接擦掉——这就是“SD 挂载成功但屏幕上没有照片列表”的原因。
  if (sdCardReady) {
    drawImageList();
  } else {
    drawSdErrorScreen();
  }
}

void loop() {
  
    static uint32_t lastPrint = 0;
    // 串口命令：实时微调触摸方向 / 触发四点校准（'c' 'x' 'h' 'v' 'p' 'r'，见文件上部说明）
    handleTouchSerial();
    // 处理触摸事件，状态机驱动
    handleTouch();
    // 触摸统一交给 TFT_eSPI 的 getTouch()（见 handleTouch()）。
    // 这里原本还有一段 XPT2046_Touchscreen(ts) 的读取代码，但它算出来的 x/y
    // 没有任何人使用（只剩一个 delay(100)），却会绕开互斥锁去操作 TFT/SD
    // 共用的那条 SPI 总线，属于纯粹的风险点，已删除。
  
  // 用 millis() 替代 delay()，避免阻塞WiFi（避免断线重连）
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) { // 每 1 秒更新一次
    printLocalTime();
    lastUpdate = millis();
  }
  
  // 定期清理已断开/超时的 WebSocket 客户端
  // （库要求定期调用，否则列表里的“僵尸客户端”会越积越多，广播一次要遍历很久）
  static unsigned long lastWsCleanup = 0;
  if (millis() - lastWsCleanup > 5000) {
    wsAudio.cleanupClients();
    lastWsCleanup = millis();
  }

  // 【听】模式下把本地麦克风(INMP441)的声音推给网页
  // 说明：
  //  1) i2s_read 自己按“有多少数据就读多少”的节奏返回，这里**不要**再额外 delay，
  //     否则读取速度慢于 16kHz 的采样速度，DMA 会溢出丢样本（声音断续）。
  //  2) 发送前必须用 availableForWriteAll() 看发送队列是否还有空间：
  //     队列满时无条件 binaryAll() 会让库打印
  //     “[E][AsyncWebSocket.cpp] Too many messages queued: closing connection”
  //     并直接断开 WebSocket（这就是之前终端里的报错）。
  //     队列满就丢掉这一帧，只影响一点音质，不会断线。
  if (!isPlaying && micStreamEnabled && wsAudioConnected && wsAudio.count() > 0) {
    static int16_t micBuf[AUDIO_CHUNK_BYTES / 2];   // 1024 采样点 = 2048 字节 ≈ 64ms
    size_t bytesRead = 0;
    if (i2s_read(I2S_NUM_0, micBuf, sizeof(micBuf), &bytesRead, pdMS_TO_TICKS(100)) == ESP_OK && bytesRead > 0) {
      bytesRead &= ~1u;   // 只发整数个 16bit 采样点，避免网页端解码错位
      if (bytesRead > 0 && wsAudio.availableForWriteAll()) {
        wsAudio.binaryAll((const char *)micBuf, bytesRead);
      }
    }
  }
}
