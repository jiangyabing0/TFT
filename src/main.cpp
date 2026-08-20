#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <LittleFS.h>
#include <TJpg_Decoder.h>
#include <WiFi.h>         // 核心WiFi库
#include <time.h>          // 时间库
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "soc/soc.h"       // 禁用欠压检测器
#include "soc/rtc_cntl_reg.h" // 禁用欠压检测器


// 触摸引脚（确保与 User_Setup.h 或实际接线一致）
#define TOUCH_CS  21
#define TOUCH_IRQ -1   // 不用可设为 -1

// 你家的 Wi-Fi 账号密码
const char* ssid = "CMCC-402";
const char* password = "jiangyabing";
// 设置 NTP 服务器（国内建议用阿里云或腾讯云）
const char* ntpServer = "ntp.aliyun.com";
const long  gmtOffset_sec = 8 * 3600; // 中国时区 UTC+8
const int   daylightOffset_sec = 0;   // 无夏令时

TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// 异步 Web 服务器（端口 80）
AsyncWebServer server(80);

// --- 2. 图片配置 ---
#define MAX_IMAGES 5 // 根据你 data 里的图片数量修改
const char* imageFiles[MAX_IMAGES] = {"/1.jpg", "/2.jpg", "/3.jpg", "/4.jpg", "/5.jpg"};
int currentImageIndex = 0;
// 当前显示的图片文件名
String currentImage = "/1.jpg";

// --- 缩放配置 ---
// 缩放倍率列表（0.5 倍 ~ 3 倍）
const float zoomLevels[] = {0.5, 0.75, 1.0, 1.5, 2.0, 3.0};
const int zoomLevelCount = sizeof(zoomLevels) / sizeof(zoomLevels[0]);
int currentZoomIndex = 2; // 默认 1.0 倍

// --- 底部按钮区域配置 ---
// 屏幕为横屏 320x240（rotation 1），底部 40 像素留给按钮
#define BUTTON_BAR_Y 200
#define BUTTON_BAR_H 40
#define BUTTON_COUNT 4
int buttonW = 0;  // 每个按钮的宽度，在 setup 中根据实际屏幕宽度计算

// 按钮文字
const char* buttonLabels[BUTTON_COUNT] = {"上一页", "下一页", "放大", "缩小"};


// 拍照显示函数 (和之前一样)
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if ( y >= tft.height() ) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

// --- 缩放用内存缓冲 ---
uint16_t* imgBuffer = NULL;   // 存放解码后的完整图片
uint16_t imgBufW = 0, imgBufH = 0;

// 用于把解码结果捕获到内存缓冲的回调
bool capture_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      int px = x + i;
      int py = y + j;
      if (px < imgBufW && py < imgBufH) {
        imgBuffer[py * imgBufW + px] = bitmap[j * w + i];
      }
    }
  }
  return 1;
}

// 绘制底部按钮栏
void drawButtons() {
  for (int i = 0; i < BUTTON_COUNT; i++) {
    int x = i * buttonW;
    // 绘制按钮背景
    tft.fillRect(x, BUTTON_BAR_Y, buttonW, BUTTON_BAR_H, TFT_NAVY);
    // 绘制按钮边框
    tft.drawRect(x, BUTTON_BAR_Y, buttonW, BUTTON_BAR_H, TFT_WHITE);
    // 绘制按钮文字（居中）
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setTextSize(1);
    // 计算文字居中位置
    int textW = tft.textWidth(buttonLabels[i], 2);
    int textX = x + (buttonW - textW) / 2;
    int textY = BUTTON_BAR_Y + (BUTTON_BAR_H - 16) / 2;
    tft.drawString(buttonLabels[i], textX, textY, 2);
  }
}


// 显示图片（支持缩放）
void displayImage(String filename) {
  tft.fillScreen(TFT_BLACK);
  if (LittleFS.exists(filename)) {
    // 获取图片原始尺寸
    uint16_t imgW, imgH;
    TJpgDec.getFsJpgSize(&imgW, &imgH, filename, LittleFS);

    // 计算缩放后的尺寸
    float zf = zoomLevels[currentZoomIndex];
    int scaledW = (int)(imgW * zf);
    int scaledH = (int)(imgH * zf);

    // 图片显示区域（底部按钮栏以上），使用实际屏幕宽度
    int displayW = tft.width();
    int displayH = BUTTON_BAR_Y;


    // 如果缩放后超出显示区域，则限制为显示区域大小（保持比例）
    if (scaledW > displayW || scaledH > displayH) {
      float ratio = min((float)displayW / scaledW, (float)displayH / scaledH);
      scaledW = (int)(scaledW * ratio);
      scaledH = (int)(scaledH * ratio);
    }

    // 居中显示
    int x = (displayW - scaledW) / 2;
    int y = (displayH - scaledH) / 2;

    // 释放上一次的缓冲
    if (imgBuffer) { free(imgBuffer); imgBuffer = NULL; }

    // 分配完整图片缓冲（16位色，每像素2字节）
    size_t bufSize = (size_t)imgW * imgH * 2;
    imgBuffer = (uint16_t*)malloc(bufSize);
    if (imgBuffer) {
      imgBufW = imgW;
      imgBufH = imgH;
      // 用捕获回调把图片完整解码到内存
      TJpgDec.setCallback(capture_output);
      TJpgDec.drawFsJpg(0, 0, filename, LittleFS);
      TJpgDec.setCallback(tft_output);

      // 从缓冲缩放并绘制到屏幕（逐行处理，节省内存）
      uint16_t* rowBuf = (uint16_t*)malloc(scaledW * 2);
      if (rowBuf) {
        for (int ty = 0; ty < scaledH; ty++) {
          int sy = ty * imgH / scaledH;
          for (int tx = 0; tx < scaledW; tx++) {
            int sx = tx * imgW / scaledW;
            rowBuf[tx] = imgBuffer[sy * imgW + sx];
          }
          tft.pushImage(x, y + ty, scaledW, 1, rowBuf);
        }
        free(rowBuf);
      }
      free(imgBuffer);
      imgBuffer = NULL;
    } else {
      // 内存不足时退回原始尺寸绘制
      TJpgDec.drawFsJpg(0, 0, filename, LittleFS);
    }

    currentImage = filename;
    Serial.printf("已切换到图片: %s (缩放 %.2f 倍, %dx%d)\n", filename.c_str(), zf, scaledW, scaledH);
  } else {
    tft.setTextColor(TFT_WHITE);
    tft.drawString("图片丢失", 10, 10, 2);
  }
  // 重绘按钮栏
  drawButtons();
}


// 切换图片（上一页/下一页）
void changePage(int delta) {
  currentImageIndex += delta;
  if (currentImageIndex < 0) currentImageIndex = MAX_IMAGES - 1;
  if (currentImageIndex >= MAX_IMAGES) currentImageIndex = 0;
  displayImage(imageFiles[currentImageIndex]);
}

// 缩放图片
void changeZoom(int delta) {
  currentZoomIndex += delta;
  if (currentZoomIndex < 0) currentZoomIndex = 0;
  if (currentZoomIndex >= zoomLevelCount) currentZoomIndex = zoomLevelCount - 1;
  displayImage(currentImage);
}

// 时间显示区域（避免闪烁的关键：只更新变化的区域）
#define DATE_X 10
#define DATE_Y 10
#define TIME_X 10
#define TIME_Y 40


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


void setup() {
  // 禁用欠压检测器（Brownout Detector），防止供电不足时芯片复位
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  tft.init();

  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);

  // 根据实际屏幕宽度计算每个按钮的宽度（横屏时 tft.width()=320）
  buttonW = tft.width() / BUTTON_COUNT;

  // 初始化触摸屏
  ts.begin();
  ts.setRotation(2); // 与屏幕旋转方向一致

/*
      // 2. 初始化触摸
    bool touchOk = ts.begin();
    if (!touchOk) {
        Serial.println("触摸芯片初始化失败！请检查 SPI 接线和 CS 引脚。");
        tft.fillScreen(TFT_RED);
        tft.setTextColor(TFT_WHITE, TFT_RED);
        tft.drawString("TOUCH ERROR", 10, 10, 4);
        while (1) delay(100);  // 死循环，提示错误
    }
    // 如果触摸芯片支持，设置旋转
    ts.setRotation(1);
    Serial.println("触摸芯片初始化成功！");
*/
  // 初始化 LittleFS 文件系统（存放图片）
  if (!LittleFS.begin()) {
    Serial.println("LittleFS 初始化失败！");
  } else {
    Serial.println("LittleFS 初始化成功");
  }

  // 配置 TJpg_Decoder
  tft.setSwapBytes(true); // 交换颜色字节序（大小端）
  TJpgDec.setJpgScale(1); // 缩放比例 1/2/4/8
  TJpgDec.setCallback(tft_output); // 指定渲染回调函数

  // 1. 连 WiFi（带超时机制，防止卡死）
  WiFi.begin(ssid, password);
  tft.drawString("Connecting WiFi...", 10, 10, 2);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) { // 最多等10秒
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  tft.fillScreen(TFT_BLACK);

  // 显示默认图片（如果存在）
  if (LittleFS.exists("/1.jpg")) {
    displayImage("/1.jpg");
  }

  // 2. 初始化 NTP 时间
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime(); // 显示一次当前时间

  // 3. 配置 Web 服务器路由
  // 控制网页
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", getControlPage());
  });

  // 显示指定图片：/display?img=xxx.jpg
  server.on("/display", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("img")) {
      String img = request->getParam("img")->value();
      // 确保文件名以 "/" 开头
      if (!img.startsWith("/")) img = "/" + img;
      if (LittleFS.exists(img)) {
        displayImage(img);
        request->send(200, "text/plain", "已显示: " + img);
      } else {
        request->send(404, "text/plain", "图片不存在: " + img);
      }
    } else {
      request->send(400, "text/plain", "缺少 img 参数");
    }
  });

  // 列出 LittleFS 中的图片
  server.on("/list", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "[";
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    bool first = true;
    while (file) {
      String name = file.name();
      if (name.endsWith(".jpg") || name.endsWith(".jpeg")) {
        if (!first) json += ",";
        json += "\"" + name + "\"";
        first = false;
      }
      file = root.openNextFile();
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  // 删除图片：POST /delete?img=xxx.jpg
  server.on("/delete", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("img")) {
      String img = request->getParam("img")->value();
      // 确保文件名以 "/" 开头
      if (!img.startsWith("/")) img = "/" + img;
      // 防止删除 index.html 等非图片文件
      if (!img.endsWith(".jpg") && !img.endsWith(".jpeg")) {
        request->send(400, "text/plain", "只允许删除 JPG 图片");
        return;
      }
      if (LittleFS.exists(img)) {
        if (LittleFS.remove(img)) {
          Serial.println("已删除图片: " + img);
          // 如果删除的是当前显示的图片，清屏
          if (img == currentImage) {
            tft.fillScreen(TFT_BLACK);
            currentImage = "";
          }
          request->send(200, "text/plain", "已删除: " + img);
        } else {
          request->send(500, "text/plain", "删除失败: " + img);
        }
      } else {
        request->send(404, "text/plain", "图片不存在: " + img);
      }
    } else {
      request->send(400, "text/plain", "缺少 img 参数");
    }
  });

  // 上传图片：POST /upload (multipart/form-data)
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest* request) {

    request->send(200, "text/plain", "上传完成");
  }, [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    // 处理文件上传数据
    static File uploadFile;
    if (!index) {
      // 开始新文件
      String path = "/" + filename;
      // 只允许 jpg/jpeg 文件
      if (!path.endsWith(".jpg") && !path.endsWith(".jpeg")) {
        request->send(400, "text/plain", "只支持 JPG 图片");
        return;
      }
      uploadFile = LittleFS.open(path, "w");
      if (!uploadFile) {
        request->send(500, "text/plain", "无法创建文件");
        return;
      }
      Serial.println("开始上传: " + path);
    }
    if (uploadFile) {
      uploadFile.write(data, len);
    }
    if (final) {
      if (uploadFile) {
        uploadFile.close();
        Serial.println("上传完成: " + filename);
        // 上传完成后自动显示
        String path = "/" + filename;
        displayImage(path);
      }
    }
  });

  // 启动服务器
  server.begin();
  Serial.print("Web 服务器已启动，访问 http://");
  Serial.println(WiFi.localIP());
}

void loop() {
  
    static uint32_t lastPrint = 0;
    // 检测触摸
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        // 压力值过滤（防止悬空误触）
        if (p.z > 100) {  // 根据实际调整阈值
            // 将 ADC 值映射到屏幕像素
            // x 对应屏幕横向(0~320)，y 对应屏幕纵向(0~240)
            uint16_t x = map(p.x, 0, 4095, 0, TFT_HEIGHT);
            uint16_t y = map(p.y, 0, 4095, 0, TFT_WIDTH);


            // 判断是否点击了底部按钮
            if (y >= BUTTON_BAR_Y) {
              int btnIndex = x / buttonW;

              if (btnIndex >= 0 && btnIndex < BUTTON_COUNT) {
                // 按钮按下反馈（高亮）
                int bx = btnIndex * buttonW;
                tft.fillRect(bx, BUTTON_BAR_Y, buttonW, BUTTON_BAR_H, TFT_BLUE);
                tft.drawRect(bx, BUTTON_BAR_Y, buttonW, BUTTON_BAR_H, TFT_WHITE);
                tft.setTextColor(TFT_WHITE, TFT_BLUE);
                int textW = tft.textWidth(buttonLabels[btnIndex], 2);
                tft.drawString(buttonLabels[btnIndex], bx + (buttonW - textW) / 2, BUTTON_BAR_Y + (BUTTON_BAR_H - 16) / 2, 2);


                switch (btnIndex) {
                  case 0: changePage(-1); break;  // 上一页
                  case 1: changePage(1);  break;  // 下一页
                  case 2: changeZoom(1);  break;  // 放大
                  case 3: changeZoom(-1); break;  // 缩小
                }
                Serial.printf("按钮 %d (%s) 被按下\n", btnIndex, buttonLabels[btnIndex]);
              }
            } else {
              // 在图片区域触摸，显示坐标（调试用）
              tft.setTextColor(TFT_WHITE, TFT_BLACK);
              tft.setTextSize(1);
              tft.setCursor(10, TFT_HEIGHT - 20);
              tft.printf("X:%3d Y:%3d  Z:%4d  ", x, y, p.z);
              Serial.printf("Touch: X=%d, Y=%d, Pressure=%d\n", x, y, p.z);
            }


            // 防抖
            delay(100);
        }
    }
  
  // 用 millis() 替代 delay()，避免阻塞WiFi（避免断线重连）
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) { // 每 1 秒更新一次
    printLocalTime();
    lastUpdate = millis();
  }
  

}
