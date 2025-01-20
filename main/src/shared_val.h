#ifndef SHARED_VAL_H
#define SHARED_VAL_H



enum  {
    COLORMAP_CLASSIC,
    COLORMAP_TURBO,
    COLORMAP_HOT,
    COLORMAP_VIRIDIS,
    COLORMAP_INFERNO,
    COLORMAP_GRAYSR, // 白热
    COLORMAP_GRAYS   // 黑热
}; // 颜色映射表类型

enum  {
    PROB_CONNECTING,
    PROB_INITIALIZING,
    PROB_PREPARING,
    PROB_READY
}; // 探头目前的启动状态码

// 线程锁
bool prob_lock = true;
bool pix_cp_lock = false;
bool flag_sensor_ok = false;
bool cmap_loading_lock = false; // 颜色映射表加载锁
// 共享变量
int brightness = 100;
unsigned short T_max, T_min;// 温度
unsigned long  T_avg; // 温度平均值

uint16_t test_point[2] = {140, 120};  // 测温点的位置
bool flag_use_kalman = true;  // 是否使用卡尔曼滤波器
bool use_upsample = true;  // 是否使用双线性插值
bool flag_trace_max = true;  // 是否使用最热点追踪
bool flag_in_photo_mode = false;  // 是否正处于照相模式(画面暂停)
bool flag_show_cursor = true; // 是否显示温度采集指针
bool flag_clear_cursor = false; // 是否拍照模式下清除温度采集指针
uint8_t cmap_now_choose = COLORMAP_CLASSIC;  // 当前所使用的颜色映射表
uint8_t vbat_percent = 100;  // 电池电量百分比
uint8_t prob_status = PROB_CONNECTING;  // 探头当前状态


#endif