#include "oscilloscope.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <set>

// 定义 M_PI（某些编译器可能没有定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 构造函数
OscilloscopeWindow::OscilloscopeWindow() 
    : window(nullptr)
    , drawing_area(nullptr)
    , channel_list(nullptr)
    , add_channel_combo(nullptr)
    , time_window_spin(nullptr)
    , auto_scale_check(nullptr)
    , current_frame_index(0)
    , time_window(10.0)
    , auto_scale(true)
    , fixed_y_min(0.0)
    , fixed_y_max(100.0)
    , next_color_index(0)
{
    // 设置UTF-8编码（修复中文乱码）
    g_setenv("LANG", "zh_CN.UTF-8", TRUE);
    
    // 初始化颜色预设（8种不同颜色）
    color_palette = {
        {1.0, 0.2, 0.2, 1.0},  // 红色
        {0.2, 0.8, 1.0, 1.0},  // 青色
        {1.0, 0.8, 0.2, 1.0},  // 黄色
        {0.6, 0.4, 1.0, 1.0},  // 紫色
        {0.4, 0.9, 0.5, 1.0},  // 绿色
        {1.0, 0.6, 0.3, 1.0},  // 橙色
        {0.3, 0.6, 0.9, 1.0},  // 蓝色
        {1.0, 0.4, 0.6, 1.0}   // 粉色
    };
}

// 析构函数
OscilloscopeWindow::~OscilloscopeWindow() {
    // 不在这里销毁window，因为GTK会自动管理
    // 只需要清理数据
    channels.clear();
    available_vars.clear();
}

// 显示窗口
void OscilloscopeWindow::show() {
    if (!window) {
        buildUI();
        // 连接窗口关闭信号，只是隐藏窗口而不是销毁
        g_signal_connect(window, "delete-event", G_CALLBACK(onWindowDelete), this);
    }
    gtk_widget_show_all(window);
}

// 构建UI
void OscilloscopeWindow::buildUI() {
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "数据示波器");
    gtk_window_set_default_size(GTK_WINDOW(window), 1000, 600);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    
    // 设置全局CSS样式，使用支持中文的字体
    GtkCssProvider *css_provider = gtk_css_provider_new();
    const char *css_data = 
        "window, label, button, entry, combobox, treeview {"
        "  font-family: 'Microsoft YaHei', 'SimHei', 'Sans';"
        "  font-size: 10pt;"
        "}"
        "label {"
        "  font-family: 'Microsoft YaHei', 'SimHei', 'Sans';"
        "}";
    gtk_css_provider_load_from_data(css_provider, css_data, -1, NULL);
    gtk_style_context_add_provider_for_screen(
        gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(css_provider),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(css_provider);
    
    // 主容器（水平分隔）
    GtkWidget *main_paned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_container_add(GTK_CONTAINER(window), main_paned);
    
    // 左侧：绘图区域
    GtkWidget *left_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_paned_pack1(GTK_PANED(main_paned), left_box, TRUE, TRUE);
    
    // 控制面板
    GtkWidget *control_panel = buildControlPanel();
    gtk_box_pack_start(GTK_BOX(left_box), control_panel, FALSE, FALSE, 0);
    
    // 绘图区域
    GtkWidget *drawing_frame = gtk_frame_new("波形显示");
    gtk_frame_set_shadow_type(GTK_FRAME(drawing_frame), GTK_SHADOW_IN);
    drawing_area = buildDrawingArea();
    gtk_container_add(GTK_CONTAINER(drawing_frame), drawing_area);
    gtk_box_pack_start(GTK_BOX(left_box), drawing_frame, TRUE, TRUE, 0);
    
    // 右侧：通道面板
    GtkWidget *channel_panel = buildChannelPanel();
    gtk_paned_pack2(GTK_PANED(main_paned), channel_panel, FALSE, TRUE);
    
    // 设置初始分隔比例
    gtk_paned_set_position(GTK_PANED(main_paned), 700);
}

// 构建控制面板
GtkWidget* OscilloscopeWindow::buildControlPanel() {
    GtkWidget *frame = gtk_frame_new("显示设置");
    gtk_frame_set_shadow_type(GTK_FRAME(frame), GTK_SHADOW_ETCHED_IN);
    
    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_container_set_border_width(GTK_CONTAINER(hbox), 10);
    gtk_container_add(GTK_CONTAINER(frame), hbox);
    
    // 时间窗口设置
    GtkWidget *time_label = gtk_label_new("时间窗口:");
    gtk_box_pack_start(GTK_BOX(hbox), time_label, FALSE, FALSE, 0);
    
    time_window_spin = gtk_spin_button_new_with_range(1.0, 60.0, 1.0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(time_window_spin), time_window);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(time_window_spin), 1);
    g_signal_connect(time_window_spin, "value-changed", 
                     G_CALLBACK(onTimeWindowChanged), this);
    gtk_box_pack_start(GTK_BOX(hbox), time_window_spin, FALSE, FALSE, 0);
    
    GtkWidget *seconds_label = gtk_label_new("秒");
    gtk_box_pack_start(GTK_BOX(hbox), seconds_label, FALSE, FALSE, 0);
    
    // 分隔符
    GtkWidget *separator = gtk_separator_new(GTK_ORIENTATION_VERTICAL);
    gtk_box_pack_start(GTK_BOX(hbox), separator, FALSE, FALSE, 10);
    
    // 自动缩放选项
    auto_scale_check = gtk_check_button_new_with_label("自动缩放Y轴");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(auto_scale_check), auto_scale);
    g_signal_connect(auto_scale_check, "toggled", 
                     G_CALLBACK(onAutoScaleToggled), this);
    gtk_box_pack_start(GTK_BOX(hbox), auto_scale_check, FALSE, FALSE, 0);
    
    return frame;
}

// 构建绘图区域
GtkWidget* OscilloscopeWindow::buildDrawingArea() {
    GtkWidget *area = gtk_drawing_area_new();
    gtk_widget_set_size_request(area, 600, 400);
    g_signal_connect(area, "draw", G_CALLBACK(onDraw), this);
    return area;
}

// 构建通道面板
GtkWidget* OscilloscopeWindow::buildChannelPanel() {
    GtkWidget *frame = gtk_frame_new("通道管理");
    gtk_frame_set_shadow_type(GTK_FRAME(frame), GTK_SHADOW_ETCHED_IN);
    
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
    gtk_container_add(GTK_CONTAINER(frame), vbox);
    
    // 添加通道区域
    GtkWidget *add_label = gtk_label_new("选择变量添加通道:");
    gtk_box_pack_start(GTK_BOX(vbox), add_label, FALSE, FALSE, 0);
    
    add_channel_combo = gtk_combo_box_text_new();
    // 添加默认提示项
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_channel_combo), "(请先加载CSV)");
    gtk_combo_box_set_active(GTK_COMBO_BOX(add_channel_combo), 0);
    gtk_box_pack_start(GTK_BOX(vbox), add_channel_combo, FALSE, FALSE, 0);
    
    GtkWidget *add_button = gtk_button_new_with_label("添加通道");
    g_signal_connect(add_button, "clicked", G_CALLBACK(onAddChannel), this);
    gtk_box_pack_start(GTK_BOX(vbox), add_button, FALSE, FALSE, 0);
    
    // 分隔线
    GtkWidget *separator = gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_box_pack_start(GTK_BOX(vbox), separator, FALSE, FALSE, 5);
    
    // 通道列表
    GtkWidget *list_label = gtk_label_new("当前通道:");
    gtk_box_pack_start(GTK_BOX(vbox), list_label, FALSE, FALSE, 0);
    
    // 创建TreeView
    GtkWidget *scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), 
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_size_request(scroll, 250, -1);
    
    GtkListStore *store = gtk_list_store_new(3, G_TYPE_BOOLEAN, G_TYPE_STRING, GDK_TYPE_RGBA);
    channel_list = gtk_tree_view_new_with_model(GTK_TREE_MODEL(store));
    g_object_unref(store);
    
    // 可见性列（复选框）
    GtkCellRenderer *toggle_renderer = gtk_cell_renderer_toggle_new();
    g_signal_connect(toggle_renderer, "toggled", 
                     G_CALLBACK(onChannelVisibilityToggled), this);
    GtkTreeViewColumn *toggle_col = gtk_tree_view_column_new_with_attributes(
        "显示", toggle_renderer, "active", 0, NULL);
    gtk_tree_view_append_column(GTK_TREE_VIEW(channel_list), toggle_col);
    
    // 名称列
    GtkCellRenderer *text_renderer = gtk_cell_renderer_text_new();
    GtkTreeViewColumn *name_col = gtk_tree_view_column_new_with_attributes(
        "通道名称", text_renderer, "text", 1, NULL);
    gtk_tree_view_append_column(GTK_TREE_VIEW(channel_list), name_col);
    
    gtk_container_add(GTK_CONTAINER(scroll), channel_list);
    gtk_box_pack_start(GTK_BOX(vbox), scroll, TRUE, TRUE, 0);
    
    // 操作按钮
    GtkWidget *button_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(vbox), button_box, FALSE, FALSE, 5);
    
    GtkWidget *remove_button = gtk_button_new_with_label("删除选中");
    g_signal_connect(remove_button, "clicked", G_CALLBACK(onRemoveChannel), this);
    gtk_box_pack_start(GTK_BOX(button_box), remove_button, TRUE, TRUE, 0);
    
    GtkWidget *clear_button = gtk_button_new_with_label("清空全部");
    g_signal_connect(clear_button, "clicked", G_CALLBACK(onClearChannels), this);
    gtk_box_pack_start(GTK_BOX(button_box), clear_button, TRUE, TRUE, 0);
    
    return frame;
}

// 加载CSV数据
bool OscilloscopeWindow::loadCSV(const std::string& filename) {
    bool csv_loaded = csv_reader.loadCSV(filename);

    // 清空并重新构建可用变量列表，优先使用动态日志
    available_vars.clear();
    std::set<std::string> added_vars; // 使用set来跟踪已添加的纯变量名，以处理重名

    // 1. 优先添加所有动态日志变量
    std::vector<std::string> dynamic_vars = DynamicLogManager::getInstance().getAllVariableNames();
    for (const auto& var : dynamic_vars) {
        if (added_vars.find(var) == added_vars.end()) {
            available_vars.push_back(var + " [动态]");
            added_vars.insert(var);
        }
    }

    // 2. 如果CSV加载成功，添加不重名的CSV变量
    std::vector<std::string> csv_vars;
    if (csv_loaded) {
        csv_vars = csv_reader.getVariableNames();
        for (const auto& var : csv_vars) {
            if (added_vars.find(var) == added_vars.end()) {
                available_vars.push_back(var);
                added_vars.insert(var);
            }
        }
    }
    
    // 更新下拉框
    if (add_channel_combo) {
        // 清空现有选项
        gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(add_channel_combo));
        
        if (available_vars.empty()) {
            // 如果没有变量，显示提示
            gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_channel_combo), "(无可用变量)");
            gtk_combo_box_set_active(GTK_COMBO_BOX(add_channel_combo), 0);
            
            if (!csv_loaded) {
                g_print("[示波器] 未加载CSV，也没有动态日志变量\n");
            } else {
                g_print("[示波器] CSV中没有找到数值变量\n");
            }
        } else {
            // 添加所有可用变量
            for (const auto& var : available_vars) {
                gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_channel_combo), var.c_str());
            }
            
            // 默认选择第一个
            gtk_combo_box_set_active(GTK_COMBO_BOX(add_channel_combo), 0);
            
            if (csv_loaded) {
                g_print("[示波器] 成功加载CSV，找到 %zu 个可用变量 (%zu 动态, %zu CSV)\n", 
                        available_vars.size(), dynamic_vars.size(), csv_vars.size());
            } else {
                g_print("[示波器] 临时模式，找到 %zu 个动态日志变量\n", available_vars.size());
            }
        }
    }
    
    return !available_vars.empty();
}

// 添加通道
void OscilloscopeWindow::addChannel(const std::string& variable_name) {
    // 检查是否为提示文本
    if (variable_name.empty() || 
        variable_name.find("请先加载") != std::string::npos ||
        variable_name.find("没有数值变量") != std::string::npos) {
        g_print("[示波器] 请先加载包含数值变量的CSV文件\n");
        return;
    }
    
    // 检查是否为动态日志变量
    bool is_dynamic = (variable_name.find(" [动态]") != std::string::npos);
    std::string clean_name = variable_name;
    if (is_dynamic) {
        // 移除 " [动态]" 后缀
        size_t pos = clean_name.find(" [动态]");
        if (pos != std::string::npos) {
            clean_name = clean_name.substr(0, pos);
        }
    }
    
    // 检查是否已存在
    for (const auto& ch : channels) {
        if (ch.variable_name == clean_name) {
            g_print("[示波器] 通道 '%s' 已存在\n", clean_name.c_str());
            return;
        }
    }
    
    // 创建新通道
    ChannelData new_channel;
    new_channel.name = is_dynamic ? clean_name + " [动态]" : clean_name;
    new_channel.variable_name = clean_name;
    new_channel.is_dynamic = is_dynamic;
    new_channel.color = getNextColor();
    new_channel.visible = true;
    new_channel.min_value = 0.0;
    new_channel.max_value = 0.0;
    
    channels.push_back(new_channel);
    
    g_print("[示波器] 已添加%s通道: %s\n", is_dynamic ? "动态日志" : "CSV", clean_name.c_str());
    
    // 更新TreeView
    if (channel_list) {
        GtkListStore *store = GTK_LIST_STORE(gtk_tree_view_get_model(GTK_TREE_VIEW(channel_list)));
        GtkTreeIter iter;
        gtk_list_store_append(store, &iter);
        gtk_list_store_set(store, &iter,
                          0, new_channel.visible,
                          1, new_channel.name.c_str(),
                          2, &new_channel.color,
                          -1);
    }
    
    // 更新当前帧的数据
    updateChannelData(current_frame_index);
}

// 删除通道
void OscilloscopeWindow::removeChannel(int channel_index) {
    if (channel_index >= 0 && channel_index < (int)channels.size()) {
        channels.erase(channels.begin() + channel_index);
        refreshDrawing();
    }
}

// 清空所有通道
void OscilloscopeWindow::clearAllChannels() {
    channels.clear();
    
    if (channel_list) {
        GtkListStore *store = GTK_LIST_STORE(gtk_tree_view_get_model(GTK_TREE_VIEW(channel_list)));
        gtk_list_store_clear(store);
    }
    
    refreshDrawing();
}

// 更新通道数据
void OscilloscopeWindow::updateChannelData(int frame_index) {
    current_frame_index = frame_index;
    
    // 计算时间（使用帧索引作为时间，单位：秒，假设30fps）
    double current_time = frame_index / 30.0;
    
    // 更新每个通道的数据
    for (auto& channel : channels) {
        double value = 0.0;
        bool found = false;
        
        if (channel.is_dynamic) {
            // 从动态日志读取
            std::vector<DynamicLogVariable> dynamic_logs = 
                DynamicLogManager::getInstance().getFrameLogs(frame_index);
            
            for (const auto& log_var : dynamic_logs) {
                if (log_var.name == channel.variable_name) {
                    parseValueFromString(log_var.value_str, value);
                    found = true;
                    break;
                }
            }
        } else {
            // 从CSV读取
            if (csv_reader.getRecordCount() > 0) {
                // 限制索引范围
                int log_index = frame_index - 1;
                if (log_index < 0) log_index = 0;
                if (log_index >= csv_reader.getRecordCount()) {
                    log_index = csv_reader.getRecordCount() - 1;
                }
                
                LogRecord record = csv_reader.getLogByIndex(log_index);
                auto it = record.variables.find(channel.variable_name);
                if (it != record.variables.end()) {
                    parseValueFromString(it->second, value);
                    found = true;
                }
            }
        }
        
        if (!found) {
            continue;  // 该帧没有此变量的数据
        }
        
        // 检测时间倒退（用户往回拖动进度条）
        if (!channel.times.empty() && current_time < channel.times.back()) {
            // 清除所有当前时间之后的数据
            while (!channel.times.empty() && channel.times.back() > current_time) {
                channel.times.pop_back();
                channel.values.pop_back();
            }
        }
        
        // 添加数据点
        channel.values.push_back(value);
        channel.times.push_back(current_time);
        
        // 更新最小最大值
        if (channel.values.size() == 1) {
            channel.min_value = value;
            channel.max_value = value;
        } else {
            if (value < channel.min_value) channel.min_value = value;
            if (value > channel.max_value) channel.max_value = value;
        }
        
        // 移除超出时间窗口的旧数据
        while (!channel.times.empty() && 
               (current_time - channel.times.front()) > time_window) {
            channel.times.pop_front();
            channel.values.pop_front();
        }
        
        // 重新计算最小最大值
        if (!channel.values.empty()) {
            channel.min_value = *std::min_element(channel.values.begin(), channel.values.end());
            channel.max_value = *std::max_element(channel.values.begin(), channel.values.end());
        }
    }
    
    refreshDrawing();
}

// 更新显示
void OscilloscopeWindow::updateDisplay(int frame_index) {
    updateChannelData(frame_index);
}

// 刷新绘图
void OscilloscopeWindow::refreshDrawing() {
    if (drawing_area) {
        gtk_widget_queue_draw(drawing_area);
    }
}

// 绘图回调
gboolean OscilloscopeWindow::onDraw(GtkWidget *widget, cairo_t *cr, gpointer user_data) {
    OscilloscopeWindow *self = static_cast<OscilloscopeWindow*>(user_data);
    
    int width = gtk_widget_get_allocated_width(widget);
    int height = gtk_widget_get_allocated_height(widget);
    
    // 绘制背景
    cairo_set_source_rgb(cr, 0.1, 0.1, 0.15);
    cairo_rectangle(cr, 0, 0, width, height);
    cairo_fill(cr);
    
    // 绘制网格
    self->drawGrid(cr, width, height);
    
    // 绘制通道数据
    self->drawChannels(cr, width, height);
    
    // 绘制图例
    self->drawLegend(cr, width, height);
    
    // 绘制坐标轴标签
    self->drawAxisLabels(cr, width, height);
    
    return FALSE;
}

// 绘制网格
void OscilloscopeWindow::drawGrid(cairo_t *cr, int width, int height) {
    const int margin_left = 60;
    const int margin_right = 20;
    const int margin_top = 20;
    const int margin_bottom = 40;
    
    int plot_width = width - margin_left - margin_right;
    int plot_height = height - margin_top - margin_bottom;
    
    // 绘制边框
    cairo_set_source_rgb(cr, 0.3, 0.3, 0.35);
    cairo_set_line_width(cr, 1.0);
    cairo_rectangle(cr, margin_left, margin_top, plot_width, plot_height);
    cairo_stroke(cr);
    
    // 绘制网格线（横向和纵向各10条）
    cairo_set_source_rgba(cr, 0.2, 0.2, 0.25, 0.5);
    cairo_set_line_width(cr, 0.5);
    
    // 横向网格线
    for (int i = 1; i < 10; i++) {
        double y = margin_top + (plot_height * i / 10.0);
        cairo_move_to(cr, margin_left, y);
        cairo_line_to(cr, margin_left + plot_width, y);
        cairo_stroke(cr);
    }
    
    // 纵向网格线
    for (int i = 1; i < 10; i++) {
        double x = margin_left + (plot_width * i / 10.0);
        cairo_move_to(cr, x, margin_top);
        cairo_line_to(cr, x, margin_top + plot_height);
        cairo_stroke(cr);
    }
}

// 计算Y轴范围的辅助函数
void OscilloscopeWindow::calculateYAxisRange(double& y_min, double& y_max) const {
    y_min = 0.0;
    y_max = 100.0;
    
    if (auto_scale && !channels.empty()) {
        // 自动缩放：找到所有可见通道的最小最大值
        bool first = true;
        for (const auto& ch : channels) {
            if (ch.visible && !ch.values.empty()) {
                if (first) {
                    y_min = ch.min_value;
                    y_max = ch.max_value;
                    first = false;
                } else {
                    y_min = std::min(y_min, ch.min_value);
                    y_max = std::max(y_max, ch.max_value);
                }
            }
        }
        
        // 添加边距，避免数据贴边
        double range = y_max - y_min;
        if (range < 0.001) {
            // 如果范围太小，使用固定范围
            double center = (y_max + y_min) / 2.0;
            y_min = center - 0.5;
            y_max = center + 0.5;
        } else {
            // 添加10%的边距
            y_min -= range * 0.1;
            y_max += range * 0.1;
        }
    } else {
        y_min = fixed_y_min;
        y_max = fixed_y_max;
    }
}

// 绘制通道数据
void OscilloscopeWindow::drawChannels(cairo_t *cr, int width, int height) {
    const int margin_left = 60;
    const int margin_right = 20;
    const int margin_top = 20;
    const int margin_bottom = 40;
    
    int plot_width = width - margin_left - margin_right;
    int plot_height = height - margin_top - margin_bottom;
    
    if (channels.empty()) {
        // 显示提示文字（使用Pango渲染中文）
        PangoLayout *layout = pango_cairo_create_layout(cr);
        PangoFontDescription *desc = pango_font_description_from_string("Microsoft YaHei 12");
        pango_layout_set_font_description(layout, desc);
        
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
        cairo_move_to(cr, width / 2 - 100, height / 2);
        
        pango_layout_set_text(layout, "请添加通道以显示波形", -1);
        pango_cairo_show_layout(cr, layout);
        
        pango_font_description_free(desc);
        g_object_unref(layout);
        return;
    }
    
    // 计算Y轴范围
    double y_min, y_max;
    calculateYAxisRange(y_min, y_max);
    
    // 确保Y轴范围有效
    double y_range = y_max - y_min;
    if (y_range < 0.001) {
        y_range = 1.0;
        y_max = y_min + y_range;
    }
    
    // 获取当前时间窗口
    double time_max = current_frame_index / 30.0;
    double time_min = time_max - time_window;
    double time_range = time_window;
    if (time_range < 0.001) time_range = 1.0;
    
    // 绘制每个通道
    for (const auto& channel : channels) {
        if (!channel.visible || channel.values.empty()) {
            continue;
        }
        
        cairo_set_source_rgba(cr, channel.color.red, channel.color.green, 
                              channel.color.blue, channel.color.alpha);
        cairo_set_line_width(cr, 2.0);
        
        bool first_point = true;
        for (size_t i = 0; i < channel.values.size(); i++) {
            double t = channel.times[i];
            double v = channel.values[i];
            
            // 映射到屏幕坐标（添加边界检查）
            double x = margin_left + plot_width * (t - time_min) / time_range;
            double y = margin_top + plot_height * (1.0 - (v - y_min) / y_range);
            
            // 限制坐标范围，避免绘制到屏幕外
            if (x < margin_left - 10 || x > margin_left + plot_width + 10) continue;
            if (y < margin_top - 10 || y > margin_top + plot_height + 10) continue;
            
            if (first_point) {
                cairo_move_to(cr, x, y);
                first_point = false;
            } else {
                cairo_line_to(cr, x, y);
            }
        }
        
        cairo_stroke(cr);
    }
}

// 绘制图例
void OscilloscopeWindow::drawLegend(cairo_t *cr, int width, int height) {
    if (channels.empty()) return;
    
    const int legend_x = width - 180;
    const int legend_y = 30;
    const int line_height = 25;
    
    // 使用Pango渲染文字以更好地支持中文
    PangoLayout *layout = pango_cairo_create_layout(cr);
    PangoFontDescription *desc = pango_font_description_from_string("Microsoft YaHei 10");
    pango_layout_set_font_description(layout, desc);
    
    int y_offset = 0;
    for (const auto& channel : channels) {
        if (!channel.visible) continue;
        
        // 绘制颜色方块
        cairo_set_source_rgba(cr, channel.color.red, channel.color.green, 
                              channel.color.blue, channel.color.alpha);
        cairo_rectangle(cr, legend_x, legend_y + y_offset, 15, 15);
        cairo_fill(cr);
        
        // 绘制通道名称（使用Pango）
        cairo_set_source_rgb(cr, 0.9, 0.9, 0.9);
        cairo_move_to(cr, legend_x + 20, legend_y + y_offset);
        
        // 添加当前值
        std::string label = channel.name;
        if (!channel.values.empty()) {
            label += ": " + formatValue(channel.values.back());
        }
        
        pango_layout_set_text(layout, label.c_str(), -1);
        pango_cairo_show_layout(cr, layout);
        
        y_offset += line_height;
    }
    
    pango_font_description_free(desc);
    g_object_unref(layout);
}

// 绘制坐标轴标签
void OscilloscopeWindow::drawAxisLabels(cairo_t *cr, int width, int height) {
    const int margin_left = 60;
    const int margin_bottom = 40;
    
    // 使用Pango渲染中文文字
    PangoLayout *layout = pango_cairo_create_layout(cr);
    PangoFontDescription *desc = pango_font_description_from_string("Microsoft YaHei 9");
    pango_layout_set_font_description(layout, desc);
    
    cairo_set_source_rgb(cr, 0.7, 0.7, 0.7);
    
    // X轴标签（时间）
    cairo_move_to(cr, width / 2 - 30, height - 10);
    pango_layout_set_text(layout, "时间 (秒)", -1);
    pango_cairo_show_layout(cr, layout);
    
    // Y轴标签（旋转）
    cairo_save(cr);
    cairo_translate(cr, 10, height / 2);
    cairo_rotate(cr, -M_PI / 2);
    cairo_move_to(cr, -15, 0);
    pango_layout_set_text(layout, "数值", -1);
    pango_cairo_show_layout(cr, layout);
    cairo_restore(cr);
    
    // 绘制Y轴刻度值（使用与绘制通道相同的Y轴范围）
    double y_min, y_max;
    calculateYAxisRange(y_min, y_max);
    
    // 绘制刻度数字
    for (int i = 0; i <= 5; i++) {
        double value = y_min + (y_max - y_min) * i / 5.0;
        int y = height - margin_bottom - (height - 60) * i / 5;
        
        std::string label = formatValue(value);
        cairo_move_to(cr, margin_left - 50, y + 5);
        pango_layout_set_text(layout, label.c_str(), -1);
        pango_cairo_show_layout(cr, layout);
    }
    
    pango_font_description_free(desc);
    g_object_unref(layout);
}

// 事件处理：添加通道
void OscilloscopeWindow::onAddChannel(GtkWidget *widget, gpointer user_data) {
    OscilloscopeWindow *self = static_cast<OscilloscopeWindow*>(user_data);
    
    gchar *text = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(self->add_channel_combo));
    if (text) {
        self->addChannel(text);
        g_free(text);
    }
}

// 事件处理：删除通道
void OscilloscopeWindow::onRemoveChannel(GtkWidget *widget, gpointer user_data) {
    OscilloscopeWindow *self = static_cast<OscilloscopeWindow*>(user_data);
    
    GtkTreeSelection *selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(self->channel_list));
    GtkTreeIter iter;
    GtkTreeModel *model;
    
    if (gtk_tree_selection_get_selected(selection, &model, &iter)) {
        GtkTreePath *path = gtk_tree_model_get_path(model, &iter);
        gint *indices = gtk_tree_path_get_indices(path);
        int index = indices[0];
        gtk_tree_path_free(path);
        
        self->removeChannel(index);
        gtk_list_store_remove(GTK_LIST_STORE(model), &iter);
    }
}

// 事件处理：清空所有通道
void OscilloscopeWindow::onClearChannels(GtkWidget *widget, gpointer user_data) {
    OscilloscopeWindow *self = static_cast<OscilloscopeWindow*>(user_data);
    self->clearAllChannels();
}

// 事件处理：时间窗口变化
void OscilloscopeWindow::onTimeWindowChanged(GtkSpinButton *spin, gpointer user_data) {
    OscilloscopeWindow *self = static_cast<OscilloscopeWindow*>(user_data);
    self->time_window = gtk_spin_button_get_value(spin);
    self->refreshDrawing();
}

// 事件处理：自动缩放切换
void OscilloscopeWindow::onAutoScaleToggled(GtkToggleButton *toggle, gpointer user_data) {
    OscilloscopeWindow *self = static_cast<OscilloscopeWindow*>(user_data);
    self->auto_scale = gtk_toggle_button_get_active(toggle);
    self->refreshDrawing();
}

// 事件处理：通道可见性切换
void OscilloscopeWindow::onChannelVisibilityToggled(GtkCellRendererToggle *renderer, 
                                                     gchar *path_str, gpointer user_data) {
    OscilloscopeWindow *self = static_cast<OscilloscopeWindow*>(user_data);
    
    GtkTreeModel *model = gtk_tree_view_get_model(GTK_TREE_VIEW(self->channel_list));
    GtkTreePath *path = gtk_tree_path_new_from_string(path_str);
    GtkTreeIter iter;
    
    if (gtk_tree_model_get_iter(model, &iter, path)) {
        gint *indices = gtk_tree_path_get_indices(path);
        int index = indices[0];
        
        if (index >= 0 && index < (int)self->channels.size()) {
            self->channels[index].visible = !self->channels[index].visible;
            
            gtk_list_store_set(GTK_LIST_STORE(model), &iter, 
                              0, self->channels[index].visible, -1);
            
            self->refreshDrawing();
        }
    }
    
    gtk_tree_path_free(path);
}

// 事件处理：窗口关闭（隐藏而不是销毁）
gboolean OscilloscopeWindow::onWindowDelete(GtkWidget *widget, GdkEvent *event, gpointer user_data) {
    // 隐藏窗口而不是销毁，这样可以再次打开
    gtk_widget_hide(widget);
    return TRUE;  // 返回TRUE阻止默认的销毁行为
}

// 获取下一个颜色
GdkRGBA OscilloscopeWindow::getNextColor() {
    GdkRGBA color = color_palette[next_color_index % color_palette.size()];
    next_color_index++;
    return color;
}

// 从字符串解析数值
void OscilloscopeWindow::parseValueFromString(const std::string& str, double& value) {
    try {
        // 尝试直接转换
        value = std::stod(str);
    } catch (...) {
        // 如果失败，尝试提取数字部分
        std::string num_str;
        bool has_dot = false;
        bool has_minus = false;
        
        for (char c : str) {
            if (c == '-' && num_str.empty() && !has_minus) {
                num_str += c;
                has_minus = true;
            } else if (c == '.' && !has_dot) {
                num_str += c;
                has_dot = true;
            } else if (c >= '0' && c <= '9') {
                num_str += c;
            } else if (!num_str.empty()) {
                // 遇到非数字字符，停止
                break;
            }
        }
        
        if (!num_str.empty()) {
            try {
                value = std::stod(num_str);
            } catch (...) {
                value = 0.0;
            }
        } else {
            value = 0.0;
        }
    }
}

// 格式化数值为字符串
std::string OscilloscopeWindow::formatValue(double value) {
    std::ostringstream oss;
    
    if (std::abs(value) >= 1000.0) {
        oss << std::fixed << std::setprecision(0) << value;
    } else if (std::abs(value) >= 100.0) {
        oss << std::fixed << std::setprecision(1) << value;
    } else if (std::abs(value) >= 10.0) {
        oss << std::fixed << std::setprecision(2) << value;
    } else {
        oss << std::fixed << std::setprecision(3) << value;
    }
    
    return oss.str();
}

