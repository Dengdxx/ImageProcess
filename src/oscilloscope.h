#ifndef OSCILLOSCOPE_H
#define OSCILLOSCOPE_H

#include <gtk/gtk.h>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include "csv_reader.h"
#include "dynamic_log.h"  // 添加动态日志支持

// 通道数据结构
struct ChannelData {
    std::string name;           // 通道名称
    std::string variable_name;  // CSV或动态日志中的变量名
    GdkRGBA color;              // 通道颜色
    std::deque<double> values;  // 数据点（Y值）
    std::deque<double> times;   // 时间戳（X值，相对时间，单位：秒）
    bool visible;               // 是否显示
    bool is_dynamic;            // 是否为动态日志变量
    double min_value;           // 最小值（用于自动缩放）
    double max_value;           // 最大值（用于自动缩放）
};

// 示波器窗口类
class OscilloscopeWindow {
public:
    OscilloscopeWindow();
    ~OscilloscopeWindow();
    
    // 显示窗口
    void show();
    
    // 加载CSV数据
    bool loadCSV(const std::string& filename);
    
    // 更新显示（当视频帧变化时调用）
    void updateDisplay(int frame_index);
    
    // 获取窗口指针
        GtkWidget* getWindow() { return window; }
    
        void clearAllChannels();
        
    private:
    // UI 组件
    GtkWidget *window;
    GtkWidget *drawing_area;        // 绘图区域
    GtkWidget *channel_list;        // 通道列表（TreeView）
    GtkWidget *add_channel_combo;   // 添加通道下拉框
    GtkWidget *time_window_spin;    // 时间窗口设置
    GtkWidget *auto_scale_check;    // 自动缩放选项
    
    // 数据
    CSVReader csv_reader;                       // CSV读取器
    std::vector<ChannelData> channels;          // 所有通道
    std::vector<std::string> available_vars;    // 可用变量列表
    int current_frame_index;                    // 当前帧索引
    
    // 显示设置
    double time_window;             // 时间窗口（秒）
    bool auto_scale;                // 是否自动缩放
    double fixed_y_min;             // 固定Y轴最小值
    double fixed_y_max;             // 固定Y轴最大值
    
    // 颜色预设
    std::vector<GdkRGBA> color_palette;
    int next_color_index;
    
    // UI 构建
    void buildUI();
    GtkWidget* buildControlPanel();
    GtkWidget* buildDrawingArea();
    GtkWidget* buildChannelPanel();
    
    // 事件处理
    static gboolean onDraw(GtkWidget *widget, cairo_t *cr, gpointer user_data);
    static void onAddChannel(GtkWidget *widget, gpointer user_data);
    static void onRemoveChannel(GtkWidget *widget, gpointer user_data);
    static void onClearChannels(GtkWidget *widget, gpointer user_data);
    static void onTimeWindowChanged(GtkSpinButton *spin, gpointer user_data);
    static void onAutoScaleToggled(GtkToggleButton *toggle, gpointer user_data);
    static void onChannelVisibilityToggled(GtkCellRendererToggle *renderer, gchar *path_str, gpointer user_data);
    static gboolean onWindowDelete(GtkWidget *widget, GdkEvent *event, gpointer user_data);
    
    // 数据处理
    void addChannel(const std::string& variable_name);
    void removeChannel(int channel_index);
    void updateChannelData(int frame_index);
    void refreshDrawing();
    
    // 绘图
    void drawGrid(cairo_t *cr, int width, int height);
    void drawChannels(cairo_t *cr, int width, int height);
    void drawLegend(cairo_t *cr, int width, int height);
    void drawAxisLabels(cairo_t *cr, int width, int height);
    
    // 辅助函数
    void calculateYAxisRange(double& y_min, double& y_max) const;
    GdkRGBA getNextColor();
    void parseValueFromString(const std::string& str, double& value);
    std::string formatValue(double value);
};

#endif // OSCILLOSCOPE_H
