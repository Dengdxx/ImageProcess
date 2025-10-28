#include <gtk/gtk.h>
#include <png.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "processor.h"
#include "global_image_buffer.h"
#include "csv_reader.h"
#include "oscilloscope.h"
#include "dynamic_log.h"

#if defined(HAVE_OPENCV) && (defined(__MINGW32__) || defined(__MINGW64__))
// 兼容 MSYS2 mingw64 (msvcrt) 缺少 quick_exit/timespec_get 的环境：
// 仅做前置声明以满足 <cstdlib>/<ctime> 中的 using 声明，代码中不实际调用这些符号。
extern "C" {
    void quick_exit(int);
    int at_quick_exit(void (*)(void));
    struct timespec; int timespec_get(struct timespec*, int);
}
#elif defined(HAVE_OPENCV)
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#endif

// 复用原有全局
static GtkWidget *window;
static GtkWidget *image_area;
static GtkWidget *original_array_area;
static GtkWidget *imo_image_view;
static GdkPixbuf *current_image = NULL;
static gchar *g_application_path = NULL; // 全局变量，存储程序路径

// 全局二维数组已在 global_image_buffer.h/c 定义
static int image_width = IMAGE_W;
static int image_height = IMAGE_H;
static const int TARGET_WIDTH = IMAGE_W;
static const int TARGET_HEIGHT = IMAGE_H;

#ifdef HAVE_OPENCV
// 视频相关状态
static cv::VideoCapture g_cap;
static int g_frame_index = 0;
static std::string g_video_path;
static guint g_playback_timer = 0;      // 播放定时器ID
static gboolean g_is_playing = FALSE;   // 是否正在播放
static double g_playback_speed = 1.0;   // 播放倍速
static GtkWidget *g_play_button = NULL; // 播放/暂停按钮
static GtkWidget *g_progress_scale = NULL; // 进度条
static gboolean g_updating_progress = FALSE; // 是否正在更新进度条（避免递归）

// 日志显示相关
static CSVReader g_csv_reader;
static GtkWidget *g_log_text_view = NULL;  // 日志显示文本框
static GtkTextBuffer *g_log_buffer = NULL; // 日志文本缓冲区
static std::string g_csv_file_path;        // 保存CSV文件路径

// 示波器
static OscilloscopeWindow *g_oscilloscope = NULL;
#endif

// 前置声明
static void activate(GtkApplication *app, gpointer user_data);
static void reset_image_views();
static void free_binary_image_data();
static void allocate_imo_array();
static void refresh_all_views();
static void on_size_allocate(GtkWidget *widget, GdkRectangle *allocation, gpointer user_data);
static GdkPixbuf* scale_pixbuf_to_target(GdkPixbuf *pixbuf);
static GdkPixbuf* scale_pixbuf_to_fit(GdkPixbuf *pixbuf, int max_width, int max_height);

// 从 pixbuf 填充 original_bi_image
static gboolean pixbuf_to_binary_array(GdkPixbuf *pixbuf);

// 媒体文件处理
static void open_media_clicked(GtkWidget *widget, gpointer data);
static gboolean load_png_image(const gchar *filename);
static gboolean load_jpeg_image(const gchar *filename);
static GdkPixbuf* render_bw_array_pixbuf(uint8_t **arr, int w, int h, int pixel_size);
static GdkPixbuf* render_imo_array_pixbuf(uint8_t **arr, int w, int h, int pixel_size);

#ifdef HAVE_OPENCV
// 视频导入相关回调
// open_video_clicked is now merged into open_media_clicked
static void next_frame_clicked(GtkWidget *widget, gpointer data);
static void prev_frame_clicked(GtkWidget *widget, gpointer data);
static void show_cv_frame(const cv::Mat &frame);
static void open_tiled_selector_clicked(GtkWidget *widget, gpointer data);
static void play_pause_clicked(GtkWidget *widget, gpointer data);
static void speed_changed(GtkWidget *widget, gpointer data);
static gboolean playback_timer_callback(gpointer user_data);
static void stop_playback();
static void start_playback();
static void progress_scale_changed(GtkRange *range, gpointer user_data);
static void update_progress_bar();
static void load_log_csv_clicked(GtkWidget *widget, gpointer data);
static void update_log_display(int frame_index);
static void open_oscilloscope_clicked(GtkWidget *widget, gpointer data);
static void reset_button_clicked(GtkWidget *widget, gpointer data);

// Thumbnail generation functions
static void on_thumb_clicked(GtkWidget *widget, gpointer user_data);
#endif

// 全局重置函数 - 通过重启进程实现
static void reset_button_clicked(GtkWidget *widget, gpointer data) {
    if (g_application_path) {
        // 在Windows上，使用 "start" 命令可以在新的命令窗口中启动程序，
        // 并且不会阻塞当前进程。第一个空的双引号是标题。
        std::string command = "start \"\" \"" + std::string(g_application_path) + "\"";
        
        // 执行命令
        system(command.c_str());
        
        // 退出当前应用程序实例
        GtkApplication *app = gtk_window_get_application(GTK_WINDOW(window));
        if (G_IS_APPLICATION(app)) {
            g_application_quit(G_APPLICATION(app));
        }
    } else {
        // Fallback: 如果获取路径失败，则显示错误
        GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), 
                                                 GTK_DIALOG_DESTROY_WITH_PARENT,
                                                 GTK_MESSAGE_ERROR, 
                                                 GTK_BUTTONS_CLOSE,
                                                 "无法获取程序路径，无法重启");
        gtk_dialog_run(GTK_DIALOG(err));
        gtk_widget_destroy(err);
    }
}

int main(int argc, char **argv) {
    // 保存程序路径，用于重启
    if (argc > 0 && argv[0]) {
        g_application_path = g_strdup(argv[0]);
    }

    GtkApplication *app;
    int status;
    app = gtk_application_new("com.example.binaryimage", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    status = g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);

    free_binary_image_data();
    g_free(g_application_path); // 释放路径内存
    return status;
}

static void activate(GtkApplication *app, gpointer user_data) {
    // 初始化全局缓冲，默认白底，避免初始全黑影响观感
    init_global_image_buffers_default();
    window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(window), "二值图处理器（含视频）");
    gtk_window_set_default_size(GTK_WINDOW(window), 1100, 720);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 0);

    GtkWidget *open_media_btn = gtk_button_new_with_label("打开文件");
    g_signal_connect(open_media_btn, "clicked", G_CALLBACK(open_media_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), open_media_btn, FALSE, FALSE, 0);

    GtkWidget *reset_btn = gtk_button_new_with_label("重置");
    g_signal_connect(reset_btn, "clicked", G_CALLBACK(reset_button_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), reset_btn, FALSE, FALSE, 0);

#ifdef HAVE_OPENCV
    GtkWidget *load_log_btn = gtk_button_new_with_label("加载日志CSV");
    gtk_box_pack_start(GTK_BOX(hbox), load_log_btn, FALSE, FALSE, 0);
    g_signal_connect(load_log_btn, "clicked", G_CALLBACK(load_log_csv_clicked), NULL);

    GtkWidget *oscilloscope_btn = gtk_button_new_with_label("📊 示波器");
    gtk_box_pack_start(GTK_BOX(hbox), oscilloscope_btn, FALSE, FALSE, 0);
    g_signal_connect(oscilloscope_btn, "clicked", G_CALLBACK(open_oscilloscope_clicked), NULL);

    GtkWidget *prev_btn = gtk_button_new_with_label("上一帧");
    gtk_box_pack_start(GTK_BOX(hbox), prev_btn, FALSE, FALSE, 0);
    g_signal_connect(prev_btn, "clicked", G_CALLBACK(prev_frame_clicked), NULL);

    g_play_button = gtk_button_new_with_label("▶ 播放");
    gtk_box_pack_start(GTK_BOX(hbox), g_play_button, FALSE, FALSE, 0);
    g_signal_connect(g_play_button, "clicked", G_CALLBACK(play_pause_clicked), NULL);

    GtkWidget *next_btn = gtk_button_new_with_label("下一帧");
    gtk_box_pack_start(GTK_BOX(hbox), next_btn, FALSE, FALSE, 0);
    g_signal_connect(next_btn, "clicked", G_CALLBACK(next_frame_clicked), NULL);

    // 倍速选择下拉框
    GtkWidget *speed_label = gtk_label_new("倍速:");
    gtk_box_pack_start(GTK_BOX(hbox), speed_label, FALSE, FALSE, 0);
    
    GtkWidget *speed_combo = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(speed_combo), "0.25x");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(speed_combo), "0.5x");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(speed_combo), "1x");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(speed_combo), "2x");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(speed_combo), "4x");
    gtk_combo_box_set_active(GTK_COMBO_BOX(speed_combo), 2); // 默认 1x
    gtk_box_pack_start(GTK_BOX(hbox), speed_combo, FALSE, FALSE, 0);
    g_signal_connect(speed_combo, "changed", G_CALLBACK(speed_changed), NULL);

    GtkWidget *tiled_btn = gtk_button_new_with_label("平铺选帧");
    gtk_box_pack_start(GTK_BOX(hbox), tiled_btn, FALSE, FALSE, 0);
    g_signal_connect(tiled_btn, "clicked", G_CALLBACK(open_tiled_selector_clicked), NULL);
#endif

#ifdef HAVE_OPENCV
    // 视频进度条
    GtkWidget *progress_hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(vbox), progress_hbox, FALSE, FALSE, 0);
    
    GtkWidget *progress_label = gtk_label_new("进度:");
    gtk_box_pack_start(GTK_BOX(progress_hbox), progress_label, FALSE, FALSE, 0);
    
    g_progress_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(g_progress_scale), TRUE);
    gtk_scale_set_value_pos(GTK_SCALE(g_progress_scale), GTK_POS_RIGHT);
    gtk_box_pack_start(GTK_BOX(progress_hbox), g_progress_scale, TRUE, TRUE, 0);
    g_signal_connect(g_progress_scale, "value-changed", G_CALLBACK(progress_scale_changed), NULL);
#endif

    // 使用水平分隔面板代替固定布局,支持拖动调整
    GtkWidget *main_hpaned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_box_pack_start(GTK_BOX(vbox), main_hpaned, TRUE, TRUE, 0);

    // 左侧垂直分隔面板：原始图 + Original数组图
    GtkWidget *left_vpaned = gtk_paned_new(GTK_ORIENTATION_VERTICAL);
    gtk_paned_pack1(GTK_PANED(main_hpaned), left_vpaned, TRUE, TRUE);

    // 原始输入图（带滚动窗口,图像自适应）
    GtkWidget *current_frame = gtk_frame_new("原始输入图");
    gtk_frame_set_shadow_type(GTK_FRAME(current_frame), GTK_SHADOW_ETCHED_IN);
    GtkWidget *current_scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(current_scroll), 
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    image_area = gtk_image_new();
    gtk_container_add(GTK_CONTAINER(current_scroll), image_area);
    gtk_container_add(GTK_CONTAINER(current_frame), current_scroll);
    gtk_paned_pack1(GTK_PANED(left_vpaned), current_frame, TRUE, TRUE);

    // Original数组图（带滚动窗口）
    GtkWidget *orig_frame = gtk_frame_new("Original 数组图 (0/255)");
    gtk_frame_set_shadow_type(GTK_FRAME(orig_frame), GTK_SHADOW_ETCHED_IN);
    GtkWidget *orig_scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(orig_scroll), 
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    original_array_area = gtk_image_new();
    gtk_container_add(GTK_CONTAINER(orig_scroll), original_array_area);
    gtk_container_add(GTK_CONTAINER(orig_frame), orig_scroll);
    gtk_paned_pack2(GTK_PANED(left_vpaned), orig_frame, TRUE, TRUE);

    // 右侧垂直分隔面板：IMO 数组图 + 日志显示（可拖动调整大小）
#ifdef HAVE_OPENCV
    GtkWidget *right_vpaned = gtk_paned_new(GTK_ORIENTATION_VERTICAL);
    gtk_paned_pack2(GTK_PANED(main_hpaned), right_vpaned, TRUE, TRUE);
#else
    GtkWidget *right_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_paned_pack2(GTK_PANED(main_hpaned), right_vbox, TRUE, TRUE);
#endif

    // IMO数组图（带滚动窗口）
    GtkWidget *imo_frame = gtk_frame_new("IMO 数组图（最终输出）");
    gtk_frame_set_shadow_type(GTK_FRAME(imo_frame), GTK_SHADOW_ETCHED_IN);
    GtkWidget *imo_scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(imo_scroll), 
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    imo_image_view = gtk_image_new();
    gtk_container_add(GTK_CONTAINER(imo_scroll), imo_image_view);
    gtk_container_add(GTK_CONTAINER(imo_frame), imo_scroll);
#ifdef HAVE_OPENCV
    gtk_paned_pack1(GTK_PANED(right_vpaned), imo_frame, TRUE, TRUE);
#else
    gtk_box_pack_start(GTK_BOX(right_vbox), imo_frame, TRUE, TRUE, 0);
#endif

#ifdef HAVE_OPENCV
    // 添加日志显示区域（在 IMO 数组图下方，可拖动调整大小）
    GtkWidget *log_frame = gtk_frame_new("实时日志");
    gtk_frame_set_shadow_type(GTK_FRAME(log_frame), GTK_SHADOW_ETCHED_IN);
    
    // 创建滚动窗口
    GtkWidget *log_scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(log_scroll), 
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    // 移除固定高度限制，改为最小高度
    gtk_widget_set_size_request(log_scroll, -1, 100); // 最小高度100像素
    
    // 创建文本视图
    g_log_text_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(g_log_text_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(g_log_text_view), GTK_WRAP_WORD_CHAR);
    
    // 设置等宽字体（使用CSS Provider代替弃用的override_font）
    GtkCssProvider *provider = gtk_css_provider_new();
    gtk_css_provider_load_from_data(provider, 
        "textview { font-family: Monospace; font-size: 10pt; }", -1, NULL);
    gtk_style_context_add_provider(
        gtk_widget_get_style_context(g_log_text_view),
        GTK_STYLE_PROVIDER(provider),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(provider);
    
    g_log_buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(g_log_text_view));
    
    gtk_container_add(GTK_CONTAINER(log_scroll), g_log_text_view);
    gtk_container_add(GTK_CONTAINER(log_frame), log_scroll);
    // 使用 GtkPaned 添加日志区域，使其可拖动调整大小
    gtk_paned_pack2(GTK_PANED(right_vpaned), log_frame, FALSE, TRUE);
#endif

    g_signal_connect(current_frame, "size-allocate", G_CALLBACK(on_size_allocate), NULL);
    g_signal_connect(orig_frame, "size-allocate", G_CALLBACK(on_size_allocate), NULL);
    g_signal_connect(imo_frame, "size-allocate", G_CALLBACK(on_size_allocate), NULL);

    reset_image_views();
    gtk_widget_show_all(window);
}

// 全局数组无需释放
static void free_imo_array() {}
static void free_binary_image_data() {}

static void reset_image_views() {
    if (original_array_area)
        gtk_image_set_from_icon_name(GTK_IMAGE(original_array_area), "image-x-generic", GTK_ICON_SIZE_DIALOG);
    if (imo_image_view)
        gtk_image_set_from_icon_name(GTK_IMAGE(imo_image_view), "image-x-generic", GTK_ICON_SIZE_DIALOG);
}

static GdkPixbuf* scale_pixbuf_to_target(GdkPixbuf *pixbuf) {
    if (!pixbuf) return NULL;
    int width = gdk_pixbuf_get_width(pixbuf);
    int height = gdk_pixbuf_get_height(pixbuf);
    if (width == TARGET_WIDTH && height == TARGET_HEIGHT) return g_object_ref(pixbuf);
    return gdk_pixbuf_scale_simple(pixbuf, TARGET_WIDTH, TARGET_HEIGHT, GDK_INTERP_BILINEAR);
}

// 新增：按比例缩放图像以适应指定区域（保持宽高比）
static GdkPixbuf* scale_pixbuf_to_fit(GdkPixbuf *pixbuf, int max_width, int max_height) {
    if (!pixbuf || max_width <= 0 || max_height <= 0) return NULL;
    
    int orig_width = gdk_pixbuf_get_width(pixbuf);
    int orig_height = gdk_pixbuf_get_height(pixbuf);
    
    // 计算缩放比例（保持宽高比）
    double scale_w = (double)max_width / orig_width;
    double scale_h = (double)max_height / orig_height;
    double scale = (scale_w < scale_h) ? scale_w : scale_h;
    
    // 限制最小缩放比例，避免图像过小
    if (scale < 0.1) scale = 0.1;
    
    int new_width = (int)(orig_width * scale);
    int new_height = (int)(orig_height * scale);
    
    if (new_width == orig_width && new_height == orig_height) {
        return g_object_ref(pixbuf);
    }
    
    return gdk_pixbuf_scale_simple(pixbuf, new_width, new_height, GDK_INTERP_BILINEAR);
}

static gboolean pixbuf_to_binary_array(GdkPixbuf *pixbuf) {
    if (!pixbuf) return FALSE;
    int rowstride = gdk_pixbuf_get_rowstride(pixbuf);
    guchar *pixels = gdk_pixbuf_get_pixels(pixbuf);
    int channels = gdk_pixbuf_get_n_channels(pixbuf);
    for (int y = 0; y < IMAGE_H; y++) {
        guchar *row = pixels + y * rowstride;
        for (int x = 0; x < IMAGE_W; x++) {
            int gray;
            if (channels == 1) gray = row[x];
            else if (channels == 2) gray = row[x * 2];
            else if (channels == 3) {
                int r = row[x * 3]; int g = row[x * 3 + 1]; int b = row[x * 3 + 2];
                gray = (int)(0.299 * r + 0.587 * g + 0.114 * b);
            } else {
                int r = row[x * 4]; int g = row[x * 4 + 1]; int b = row[x * 4 + 2]; int a = row[x * 4 + 3];
                if (a == 0) { original_bi_image[y][x] = 255; continue; }
                gray = (int)(0.299 * r + 0.587 * g + 0.114 * b);
            }
            original_bi_image[y][x] = (gray > 128) ? 255 : 0;
        }
    }
    return TRUE;
}

#include <cctype>
#include <algorithm>

// ... (other includes)

// Helper function to get lowercased file extension
static std::string get_file_extension(const std::string& filename) {
    size_t dot_pos = filename.find_last_of(".");
    if (dot_pos == std::string::npos) return "";
    std::string ext = filename.substr(dot_pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return ext;
}

static gboolean load_png_image(const gchar *filename) {
    GdkPixbuf *pixbuf = gdk_pixbuf_new_from_file(filename, NULL);
    if (!pixbuf) return FALSE;
    
    // 获取image_area的分配大小，自适应缩放显示
    GtkAllocation alloc;
    gtk_widget_get_allocation(image_area, &alloc);
    int max_w = alloc.width > 50 ? alloc.width - 20 : 800;
    int max_h = alloc.height > 50 ? alloc.height - 20 : 600;
    
    GdkPixbuf *display_pb = scale_pixbuf_to_fit(pixbuf, max_w, max_h);
    if (display_pb) {
        gtk_image_set_from_pixbuf(GTK_IMAGE(image_area), display_pb);
        g_object_unref(display_pb);
    }
    
    // 转为 188x120 用于处理
    GdkPixbuf *scaled = scale_pixbuf_to_target(pixbuf);
    g_object_unref(pixbuf);
    if (!scaled) return FALSE;
    gboolean ok = pixbuf_to_binary_array(scaled);
    g_object_unref(scaled);
    if (ok) {
        // 设置当前帧为0（单张图片）
        log_set_current_frame(0);
        
        // 自动处理图像
        allocate_imo_array();
        process_original_to_imo(&original_bi_image[0][0], &imo[0][0], IMAGE_W, IMAGE_H);
        refresh_all_views();
    }
    return ok;
}

static gboolean load_jpeg_image(const gchar *filename) {
    GdkPixbuf *pixbuf = gdk_pixbuf_new_from_file(filename, NULL);
    if (!pixbuf) return FALSE;
    
    // 获取image_area的分配大小，自适应缩放显示
    GtkAllocation alloc;
    gtk_widget_get_allocation(image_area, &alloc);
    int max_w = alloc.width > 50 ? alloc.width - 20 : 800;
    int max_h = alloc.height > 50 ? alloc.height - 20 : 600;
    
    GdkPixbuf *display_pb = scale_pixbuf_to_fit(pixbuf, max_w, max_h);
    if (display_pb) {
        gtk_image_set_from_pixbuf(GTK_IMAGE(image_area), display_pb);
        g_object_unref(display_pb);
    }
    
    // 转为 188x120 用于处理
    GdkPixbuf *scaled = scale_pixbuf_to_target(pixbuf);
    g_object_unref(pixbuf);
    if (!scaled) return FALSE;
    gboolean ok = pixbuf_to_binary_array(scaled);
    g_object_unref(scaled);
    if (ok) {
        // 设置当前帧为0（单张图片）
        log_set_current_frame(0);
        
        // 自动处理图像
        allocate_imo_array();
        process_original_to_imo(&original_bi_image[0][0], &imo[0][0], IMAGE_W, IMAGE_H);
        refresh_all_views();
    }
    return ok;
}

static void open_media_clicked(GtkWidget *widget, gpointer data) {
    GtkWidget *dialog;
    dialog = gtk_file_chooser_dialog_new("打开媒体文件", GTK_WINDOW(window), GTK_FILE_CHOOSER_ACTION_OPEN,
                                         "_取消", GTK_RESPONSE_CANCEL, "_打开", GTK_RESPONSE_ACCEPT, NULL);

    // Filter for all supported types (default)
    GtkFileFilter *filter_supported = gtk_file_filter_new();
    gtk_file_filter_set_name(filter_supported, "支持的媒体文件 (*.png, *.jpg, *.mp4)");
    gtk_file_filter_add_pattern(filter_supported, "*.png");
    gtk_file_filter_add_pattern(filter_supported, "*.jpg");
    gtk_file_filter_add_pattern(filter_supported, "*.jpeg");
    #ifdef HAVE_OPENCV
    gtk_file_filter_add_pattern(filter_supported, "*.mp4");
    #endif
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter_supported);

    // Filter for images
    GtkFileFilter *filter_img = gtk_file_filter_new();
    gtk_file_filter_set_name(filter_img, "图像文件 (*.png, *.jpg, *.jpeg)");
    gtk_file_filter_add_pattern(filter_img, "*.png");
    gtk_file_filter_add_pattern(filter_img, "*.jpg");
    gtk_file_filter_add_pattern(filter_img, "*.jpeg");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter_img);

    #ifdef HAVE_OPENCV
    // Filter for videos
    GtkFileFilter *filter_vid = gtk_file_filter_new();
    gtk_file_filter_set_name(filter_vid, "视频文件 (*.mp4)");
    gtk_file_filter_add_pattern(filter_vid, "*.mp4");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter_vid);
    #endif

    // Filter for all files
    GtkFileFilter *filter_all = gtk_file_filter_new();
    gtk_file_filter_set_name(filter_all, "所有文件 (*.*)");
    gtk_file_filter_add_pattern(filter_all, "*");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter_all);

    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char *filename_c = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        std::string filename(filename_c ? filename_c : "");
        g_free(filename_c);

        if (!filename.empty()) {
            std::string ext = get_file_extension(filename);
            
            if (ext == "png" || ext == "jpg" || ext == "jpeg") {
                gboolean loaded = FALSE;
                if (ext == "png") {
                    loaded = load_png_image(filename.c_str());
                } else {
                    loaded = load_jpeg_image(filename.c_str());
                }

                if (loaded) {
                    reset_image_views();
                    refresh_all_views();
                } else {
                     GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), GTK_DIALOG_DESTROY_WITH_PARENT,
                                                        GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE,
                                                        "无法加载图像文件。");
                    gtk_dialog_run(GTK_DIALOG(err));
                    gtk_widget_destroy(err);
                }
            #ifdef HAVE_OPENCV
            } else if (ext == "mp4") {
                // 当加载新视频时，重置所有与旧日志/CSV相关的状态
                g_csv_reader.clear();
                DynamicLogManager::getInstance().clearAll();
                if (g_oscilloscope) {
                    g_oscilloscope->clearAllChannels();
                }
                if (g_log_buffer) {
                    gtk_text_buffer_set_text(g_log_buffer, "(新视频已加载, 请加载对应CSV)", -1);
                }
                g_csv_file_path.clear();

                g_video_path = filename;
                stop_playback();
                if (g_cap.isOpened()) g_cap.release();
                g_cap.open(g_video_path);
                g_frame_index = 0;
                if (g_cap.isOpened()) {
                    cv::Mat frame; 
                    if (g_cap.read(frame)) { 
                        g_frame_index = 1; 
                        show_cv_frame(frame);
                        update_progress_bar();
                    } else {
                        GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), GTK_DIALOG_DESTROY_WITH_PARENT,
                                                                GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "无法从此视频读取首帧。");
                        gtk_dialog_run(GTK_DIALOG(err));
                        gtk_widget_destroy(err);
                    }
                } else {
                    GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), GTK_DIALOG_DESTROY_WITH_PARENT,
                                                            GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "无法打开视频文件。");
                    gtk_dialog_run(GTK_DIALOG(err));
                    gtk_widget_destroy(err);
                }
            #endif
            } else {
                GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), GTK_DIALOG_DESTROY_WITH_PARENT,
                                                    GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE,
                                                    "不支持的文件类型。请选择 PNG, JPG, 或 MP4 文件。");
                gtk_dialog_run(GTK_DIALOG(err));
                gtk_widget_destroy(err);
            }
        }
    }
    gtk_widget_destroy(dialog);
}


static void allocate_imo_array() {
    for (int i = 0; i < IMAGE_H; i++) {
        for (int j = 0; j < IMAGE_W; j++) {
            imo[i][j] = 255;
        }
    }
}

static GdkPixbuf* render_bw_array_pixbuf(uint8_t arr[IMAGE_H][IMAGE_W], int w, int h, int pixel_size) {
    if (!arr || w <= 0 || h <= 0) return NULL;
    int W = w * pixel_size, H = h * pixel_size;
    GdkPixbuf *pb = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, W, H);
    if (!pb) return NULL;
    gdk_pixbuf_fill(pb, 0xffffffff);
    int rowstride = gdk_pixbuf_get_rowstride(pb);
    guchar *pixels = gdk_pixbuf_get_pixels(pb);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            guint32 color = (arr[y][x] == 0) ? 0xff000000 : 0xffffffff;
            for (int dy = 0; dy < pixel_size; ++dy)
                for (int dx = 0; dx < pixel_size; ++dx) {
                    guchar *p = pixels + (y * pixel_size + dy) * rowstride + (x * pixel_size + dx) * 3;
                    p[0] = (color >> 16) & 0xff; p[1] = (color >> 8) & 0xff; p[2] = color & 0xff;
                }
        }
    }
    return pb;
}

static GdkPixbuf* render_imo_array_pixbuf(uint8_t arr[IMAGE_H][IMAGE_W], int w, int h, int pixel_size) {
    if (!arr || w <= 0 || h <= 0) return NULL;
    int W = w * pixel_size, H = h * pixel_size;
    GdkPixbuf *pb = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, W, H);
    if (!pb) return NULL;
    gdk_pixbuf_fill(pb, 0xffffffff);
    int rowstride = gdk_pixbuf_get_rowstride(pb);
    guchar *pixels = gdk_pixbuf_get_pixels(pb);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            guint32 color;
            switch (arr[y][x]) {
                case 0: color = 0xff000000; break;
                case 1: color = 0xffff0000; break;
                case 2: color = 0xffffa500; break;
                case 3: color = 0xffffff00; break;
                case 4: color = 0xff00ff00; break;
                case 5: color = 0xff00ffff; break; // 青色
                case 6: color = 0xffff00ff; break; // 洋红色
                case 7: color = 0xff0000ff; break; // 蓝色
                case 8: color = 0xff800080; break; // 紫色
                case 9: color = 0xffffc0cb; break; // 粉色
                case 255: default: color = 0xffffffff; break; // 白色
            }
            for (int dy = 0; dy < pixel_size; ++dy)
                for (int dx = 0; dx < pixel_size; ++dx) {
                    guchar *p = pixels + (y * pixel_size + dy) * rowstride + (x * pixel_size + dx) * 3;
                    p[0] = (color >> 16) & 0xff; p[1] = (color >> 8) & 0xff; p[2] = color & 0xff;
                }
        }
    }
    return pb;
}

static void refresh_all_views() {
    // 获取各个图像区域的分配大小
    GtkAllocation alloc_orig, alloc_imo;
    
    if (original_array_area) {
        gtk_widget_get_allocation(original_array_area, &alloc_orig);
        int max_w = alloc_orig.width > 50 ? alloc_orig.width - 20 : 400;
        int max_h = alloc_orig.height > 50 ? alloc_orig.height - 20 : 300;
        
        GdkPixbuf *base = render_bw_array_pixbuf(original_bi_image, IMAGE_W, IMAGE_H, 2);
        if (base) {
            GdkPixbuf *scaled = scale_pixbuf_to_fit(base, max_w, max_h);
            if (scaled) {
                gtk_image_set_from_pixbuf(GTK_IMAGE(original_array_area), scaled);
                g_object_unref(scaled);
            }
            g_object_unref(base);
        }
    }
    
    if (imo_image_view) {
        gtk_widget_get_allocation(imo_image_view, &alloc_imo);
        int max_w = alloc_imo.width > 50 ? alloc_imo.width - 20 : 400;
        int max_h = alloc_imo.height > 50 ? alloc_imo.height - 20 : 300;
        
        GdkPixbuf *base = render_imo_array_pixbuf(imo, IMAGE_W, IMAGE_H, 2);
        if (base) {
            GdkPixbuf *scaled = scale_pixbuf_to_fit(base, max_w, max_h);
            if (scaled) {
                gtk_image_set_from_pixbuf(GTK_IMAGE(imo_image_view), scaled);
                g_object_unref(scaled);
            }
            g_object_unref(base);
        }
    }
}

// 全局timeout_id，避免多次触发
static guint g_refresh_timeout_id = 0;

static gboolean refresh_all_views_callback(gpointer user_data) {
    refresh_all_views();
    g_refresh_timeout_id = 0;  // 清除ID，表示已完成
    return G_SOURCE_REMOVE; // 只执行一次
}

static void on_size_allocate(GtkWidget *widget, GdkRectangle *allocation, gpointer user_data) {
    (void)widget; (void)allocation; (void)user_data; 
    // 延迟刷新，避免在size-allocate期间频繁重绘
    if (g_refresh_timeout_id != 0) {
        g_source_remove(g_refresh_timeout_id);
    }
    g_refresh_timeout_id = g_timeout_add(150, refresh_all_views_callback, NULL);
}

#ifdef HAVE_OPENCV
static GdkPixbuf* pixbuf_from_rgb_mat_copy(const cv::Mat &rgb)
{
    GdkPixbuf *pb = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, rgb.cols, rgb.rows);
    if (!pb) return NULL;
    int rowstride = gdk_pixbuf_get_rowstride(pb);
    guchar *pixels = gdk_pixbuf_get_pixels(pb);
    for (int y = 0; y < rgb.rows; ++y) {
        memcpy(pixels + y * rowstride, rgb.ptr(y), rgb.cols * 3);
    }
    return pb;
}

static void show_cv_frame(const cv::Mat &frame) {
    // 转换为RGB格式
    cv::Mat bgr; 
    if (frame.channels() == 3) 
        bgr = frame; 
    else 
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    
    cv::Mat rgb; 
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    
    // 显示原始图像（不限制大小，让滚动窗口和自适应缩放处理）
    GdkPixbuf *pb = pixbuf_from_rgb_mat_copy(rgb);
    if (pb) {
        // 获取image_area的分配大小，自适应缩放
        GtkAllocation alloc;
        gtk_widget_get_allocation(image_area, &alloc);
        int max_w = alloc.width > 50 ? alloc.width - 20 : 800;
        int max_h = alloc.height > 50 ? alloc.height - 20 : 600;
        
        GdkPixbuf *display_pb = scale_pixbuf_to_fit(pb, max_w, max_h);
        if (display_pb) {
            gtk_image_set_from_pixbuf(GTK_IMAGE(image_area), display_pb);
            g_object_unref(display_pb);
        }
        
        // 用于二值化的 pixbuf（固定为 TARGET_WIDTH x TARGET_HEIGHT）
        GdkPixbuf *scaled = gdk_pixbuf_scale_simple(pb, TARGET_WIDTH, TARGET_HEIGHT, GDK_INTERP_BILINEAR);
        if (scaled) {
            pixbuf_to_binary_array(scaled);
            g_object_unref(scaled);
        }
        
        g_object_unref(pb);
        
        // 设置当前帧索引（让动态日志知道当前帧号）
        log_set_current_frame(g_frame_index);
        
        // 自动处理图像：从 original 转换为 imo
        allocate_imo_array();
        process_original_to_imo(&original_bi_image[0][0], &imo[0][0], IMAGE_W, IMAGE_H);
        
        refresh_all_views();
    }
    
    // 更新日志显示
    update_log_display(g_frame_index);
    
    // 更新示波器
    if (g_oscilloscope && g_csv_reader.getRecordCount() > 0) {
        g_oscilloscope->updateDisplay(g_frame_index);
    }
}

static void load_log_csv_clicked(GtkWidget *widget, gpointer data) {
    GtkWidget *dialog = gtk_file_chooser_dialog_new("选择日志CSV文件", GTK_WINDOW(window), 
                                                      GTK_FILE_CHOOSER_ACTION_OPEN,
                                                      "_取消", GTK_RESPONSE_CANCEL, 
                                                      "_打开", GTK_RESPONSE_ACCEPT, NULL);
    
    // 添加CSV文件过滤器
    GtkFileFilter *csv_filter = gtk_file_filter_new();
    gtk_file_filter_set_name(csv_filter, "CSV 文件");
    gtk_file_filter_add_pattern(csv_filter, "*.csv");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), csv_filter);
    
    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char *filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        
        if (g_csv_reader.loadCSV(filename)) {
            // 保存CSV路径
            g_csv_file_path = filename;
            
            // 设置动态日志直接写入同一个CSV文件
            log_set_csv_path(g_csv_file_path.c_str());
            
            // 成功加载
            GtkWidget *info = gtk_message_dialog_new(GTK_WINDOW(window), 
                                                      GTK_DIALOG_DESTROY_WITH_PARENT,
                                                      GTK_MESSAGE_INFO, 
                                                      GTK_BUTTONS_CLOSE,
                                                      "成功加载 %d 条日志记录\n动态日志将写入同一文件", 
                                                      g_csv_reader.getRecordCount());
            gtk_dialog_run(GTK_DIALOG(info));
            gtk_widget_destroy(info);
            
            // 更新当前帧的日志显示
            update_log_display(g_frame_index);
            
            // 如果示波器已打开，也加载到示波器
            if (g_oscilloscope) {
                g_oscilloscope->loadCSV(filename);
                g_oscilloscope->updateDisplay(g_frame_index);
            }
        } else {
            GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), 
                                                     GTK_DIALOG_DESTROY_WITH_PARENT,
                                                     GTK_MESSAGE_ERROR, 
                                                     GTK_BUTTONS_CLOSE,
                                                     "无法加载CSV文件");
            gtk_dialog_run(GTK_DIALOG(err));
            gtk_widget_destroy(err);
        }
        
        g_free(filename);
    }
    
    gtk_widget_destroy(dialog);
}

static void update_log_display(int frame_index) {
    if (!g_log_buffer) return;
    
    // 更新动态日志管理器的当前帧
    log_set_current_frame(frame_index);
    
    // 清空当前内容
    gtk_text_buffer_set_text(g_log_buffer, "", -1);
    
    // 检查日志来源
    bool has_csv_logs = (g_csv_reader.getRecordCount() > 0);
    std::vector<DynamicLogVariable> dynamic_logs = DynamicLogManager::getInstance().getFrameLogs(frame_index);
    bool has_dynamic_logs = !dynamic_logs.empty();
    
    // 如果既没有CSV也没有动态日志
    if (!has_csv_logs && !has_dynamic_logs) {
        gtk_text_buffer_set_text(g_log_buffer, 
            "未加载日志文件\n"
            "━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            "• 点击\"加载日志CSV\"加载文件\n"
            "• 或在代码中使用 log_add_*() 函数添加日志", -1);
        return;
    }
    
    // 构建显示文本
    std::string display_text;
    display_text += "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    display_text += "  当前帧: " + std::to_string(frame_index);
    if (has_csv_logs) {
        display_text += " / " + std::to_string(g_csv_reader.getRecordCount());
    }
    display_text += "\n";
    display_text += "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";
    
    // ======== 显示动态日志（优先级更高） ========
    if (has_dynamic_logs) {
        display_text += "🔧 动态日志 (代码添加):\n";
        for (const auto& var : dynamic_logs) {
            display_text += "  • " + var.name + ": " + var.value_str + "\n";
        }
        display_text += "\n";
    }
    
    // ======== 显示CSV日志 ========
    if (has_csv_logs) {
        // 获取对应帧的日志（假设帧索引从1开始，CSV记录从0开始）
        int log_index = frame_index - 1;
        if (log_index < 0) log_index = 0;
        
        if (log_index >= g_csv_reader.getRecordCount()) {
            display_text += "⚠️  当前帧超出CSV日志范围\n\n";
        } else {
            LogRecord record = g_csv_reader.getLogByIndex(log_index);
            
            display_text += "⏰ 时间戳:\n  " + record.timestamp + "\n\n";
            
            // 显示自定义变量
            std::vector<std::string> varNames = g_csv_reader.getVariableNames();
            if (!varNames.empty()) {
                display_text += "📊 CSV日志变量:\n";
                for (const auto& varName : varNames) {
                    auto it = record.variables.find(varName);
                    if (it != record.variables.end()) {
                        display_text += "  • " + varName + ": " + it->second + "\n";
                    }
                }
                display_text += "\n";
            }
            
            // 显示原始数据
            if (!record.log_utf8.empty()) {
                display_text += "📝 UTF-8文本:\n  " + record.log_utf8 + "\n\n";
            }
    
            if (!record.log_hex.empty()) {
                display_text += "🔧 Hex数据:\n  ";
                // 格式化hex显示，每32个字符换行
                for (size_t i = 0; i < record.log_hex.length(); i += 32) {
                    if (i > 0) display_text += "  ";
                    display_text += record.log_hex.substr(i, 32) + "\n";
                }
            }
        }
    }
    
    gtk_text_buffer_set_text(g_log_buffer, display_text.c_str(), -1);
}

// 打开示波器窗口
static void open_oscilloscope_clicked(GtkWidget *widget, gpointer data) {
    if (!g_oscilloscope) {
        g_oscilloscope = new OscilloscopeWindow();
    }
    
    g_oscilloscope->show();
    
    // 如果已经加载了CSV，自动加载到示波器
    if (!g_csv_file_path.empty() && g_csv_reader.getRecordCount() > 0) {
        g_oscilloscope->loadCSV(g_csv_file_path);
        g_oscilloscope->updateDisplay(g_frame_index);
    } else {
        // 提示用户先加载CSV
        GtkWidget *info = gtk_message_dialog_new(GTK_WINDOW(window),
                                                  GTK_DIALOG_MODAL,
                                                  GTK_MESSAGE_INFO,
                                                  GTK_BUTTONS_OK,
                                                  "提示\n\n请先点击\"加载日志CSV\"按钮\n加载包含数值变量的CSV文件");
        gtk_dialog_run(GTK_DIALOG(info));
        gtk_widget_destroy(info);
    }
}

static void show_cv_frame_OLD(const cv::Mat &frame) {
    // 左侧显示原始帧（安全复制到 GdkPixbuf）
    cv::Mat bgr; if (frame.channels()==3) bgr = frame; else cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    GdkPixbuf *pb = pixbuf_from_rgb_mat_copy(rgb);
    if (pb) {
        gtk_image_set_from_pixbuf(GTK_IMAGE(image_area), pb);
        // 生成 188x120 的 pixbuf 用于二值化
        GdkPixbuf *scaled = gdk_pixbuf_scale_simple(pb, TARGET_WIDTH, TARGET_HEIGHT, GDK_INTERP_BILINEAR);
        g_object_unref(pb);
        if (scaled) {
            pixbuf_to_binary_array(scaled);
            g_object_unref(scaled);
            
            // 自动处理图像：从 original 转换为 imo
            allocate_imo_array();
            process_original_to_imo(&original_bi_image[0][0], &imo[0][0], IMAGE_W, IMAGE_H);
            
            refresh_all_views();
        }
    }
}

typedef struct ThumbData { int frame_index; GtkWidget *dialog; } ThumbData;



static void on_thumb_clicked(GtkWidget *widget, gpointer user_data)
{
    ThumbData *td = (ThumbData*)user_data;
    if (!td) return;
    if (g_cap.isOpened()) {
        stop_playback(); // 停止播放
        g_cap.set(cv::CAP_PROP_POS_FRAMES, td->frame_index);
        cv::Mat frame; 
        if (g_cap.read(frame)) { 
            g_frame_index = (int)g_cap.get(cv::CAP_PROP_POS_FRAMES); 
            show_cv_frame(frame);
            update_progress_bar();
        }
    }
    if (td->dialog) gtk_widget_destroy(td->dialog);
    free(td);
}

static void open_tiled_selector_clicked(GtkWidget *widget, gpointer data)
{
    if (!g_cap.isOpened()) {
        GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), GTK_DIALOG_DESTROY_WITH_PARENT,
                                                GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "请先导入视频");
        gtk_dialog_run(GTK_DIALOG(err)); gtk_widget_destroy(err); return;
    }
    
    stop_playback(); // 停止播放

    // 获取总帧数与采样步长
    int total = (int)g_cap.get(cv::CAP_PROP_FRAME_COUNT);
    if (total <= 0) total = 3000; // 某些容器不返回帧数
    const int maxThumbs = 200; // 限制最大缩略图数，避免内存与等待时间过大
    int step = total / maxThumbs; if (step < 1) step = 1;

    GtkWidget *dialog = gtk_dialog_new_with_buttons("选择帧", GTK_WINDOW(window), GTK_DIALOG_MODAL,
                                                    "关闭", GTK_RESPONSE_CLOSE, NULL);
    gtk_window_set_default_size(GTK_WINDOW(dialog), 1400, 900);  // 设置默认窗口大小
    gtk_window_set_position(GTK_WINDOW(dialog), GTK_WIN_POS_CENTER);  // 居中显示

    GtkWidget *content = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    GtkWidget *scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_box_pack_start(GTK_BOX(content), scroll, TRUE, TRUE, 0);

    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 6);
    gtk_container_add(GTK_CONTAINER(scroll), grid);

    // 生成缩略图
    const int cols = 6; // 每行几列
    int r = 0, c = 0;
    double saved_pos = g_cap.get(cv::CAP_PROP_POS_FRAMES);
    for (int i = 0; i < total; i += step) {
        g_cap.set(cv::CAP_PROP_POS_FRAMES, i);
        cv::Mat frame; if (!g_cap.read(frame)) break;
        cv::Mat bgr = (frame.channels()==3) ? frame : (cv::Mat)cv::Mat();
        if (frame.channels()!=3) cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
        cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        cv::Mat thumb; cv::resize(rgb, thumb, cv::Size(TARGET_WIDTH, TARGET_HEIGHT), 0, 0, cv::INTER_AREA);
        GdkPixbuf *pb = pixbuf_from_rgb_mat_copy(thumb);
        if (!pb) continue;
        GtkWidget *img = gtk_image_new_from_pixbuf(pb);
        g_object_unref(pb);
        GtkWidget *btn = gtk_button_new();
        gtk_container_add(GTK_CONTAINER(btn), img);
        ThumbData *td = (ThumbData*)malloc(sizeof(ThumbData));
        td->frame_index = i; td->dialog = dialog;
        g_signal_connect(btn, "clicked", G_CALLBACK(on_thumb_clicked), td);
        gtk_grid_attach(GTK_GRID(grid), btn, c, r, 1, 1);
        if (++c >= cols) { c = 0; ++r; }
    }
    // 还原播放位置
    g_cap.set(cv::CAP_PROP_POS_FRAMES, saved_pos);

    gtk_widget_show_all(dialog);
    g_signal_connect_swapped(dialog, "response", G_CALLBACK(gtk_widget_destroy), dialog);
}

// open_video_clicked function was removed and its logic merged into open_media_clicked

static void next_frame_clicked(GtkWidget *widget, gpointer data) {
    if (!g_cap.isOpened()) return;
    stop_playback(); // 停止播放
    cv::Mat frame; 
    if (g_cap.read(frame)) { 
        ++g_frame_index; 
        show_cv_frame(frame);
        update_progress_bar();
    }
}

static void prev_frame_clicked(GtkWidget *widget, gpointer data) {
    if (!g_cap.isOpened()) return;
    stop_playback(); // 停止播放
    double pos = g_cap.get(cv::CAP_PROP_POS_FRAMES);
    double back = pos - 2; if (back < 0) back = 0;
    g_cap.set(cv::CAP_PROP_POS_FRAMES, back);
    cv::Mat frame; if (g_cap.read(frame)) { 
        g_frame_index = (int)g_cap.get(cv::CAP_PROP_POS_FRAMES); 
        show_cv_frame(frame);
        update_progress_bar();
    }
}

// 播放控制函数
static void stop_playback() {
    if (g_playback_timer != 0) {
        g_source_remove(g_playback_timer);
        g_playback_timer = 0;
    }
    g_is_playing = FALSE;
    if (g_play_button) {
        gtk_button_set_label(GTK_BUTTON(g_play_button), "▶ 播放");
    }
}

static void start_playback() {
    if (!g_cap.isOpened()) return;
    
    // 计算定时器间隔（毫秒）
    double fps = g_cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 25.0; // 默认25fps
    int interval_ms = (int)(1000.0 / (fps * g_playback_speed));
    if (interval_ms < 1) interval_ms = 1; // 至少1ms
    
    g_is_playing = TRUE;
    if (g_play_button) {
        gtk_button_set_label(GTK_BUTTON(g_play_button), "⏸ 暂停");
    }
    
    // 启动定时器
    if (g_playback_timer != 0) {
        g_source_remove(g_playback_timer);
    }
    g_playback_timer = g_timeout_add(interval_ms, playback_timer_callback, NULL);
}

static gboolean playback_timer_callback(gpointer user_data) {
    if (!g_is_playing || !g_cap.isOpened()) {
        stop_playback();
        return G_SOURCE_REMOVE;
    }
    
    // 读取下一帧
    cv::Mat frame;
    if (g_cap.read(frame)) {
        ++g_frame_index;
        show_cv_frame(frame);
        update_progress_bar();
        
        // 检查是否到达视频末尾
        int total_frames = (int)g_cap.get(cv::CAP_PROP_FRAME_COUNT);
        if (total_frames > 0 && g_frame_index >= total_frames) {
            stop_playback();
            return G_SOURCE_REMOVE;
        }
        
        // 根据倍速调整定时器间隔
        double fps = g_cap.get(cv::CAP_PROP_FPS);
        if (fps <= 0) fps = 25.0;
        int interval_ms = (int)(1000.0 / (fps * g_playback_speed));
        if (interval_ms < 1) interval_ms = 1;
        
        // 移除旧定时器并创建新的（以应用新的倍速）
        g_playback_timer = g_timeout_add(interval_ms, playback_timer_callback, NULL);
        return G_SOURCE_REMOVE;
    } else {
        // 到达视频末尾
        stop_playback();
        return G_SOURCE_REMOVE;
    }
}

static void play_pause_clicked(GtkWidget *widget, gpointer data) {
    if (!g_cap.isOpened()) {
        GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(window), GTK_DIALOG_DESTROY_WITH_PARENT,
                                                GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "请先导入视频");
        gtk_dialog_run(GTK_DIALOG(err));
        gtk_widget_destroy(err);
        return;
    }
    
    if (g_is_playing) {
        stop_playback();
    } else {
        start_playback();
    }
}

static void speed_changed(GtkWidget *widget, gpointer data) {
    GtkComboBox *combo = GTK_COMBO_BOX(widget);
    int active = gtk_combo_box_get_active(combo);
    
    switch (active) {
        case 0: g_playback_speed = 0.25; break;
        case 1: g_playback_speed = 0.5; break;
        case 2: g_playback_speed = 1.0; break;
        case 3: g_playback_speed = 2.0; break;
        case 4: g_playback_speed = 4.0; break;
        default: g_playback_speed = 1.0; break;
    }
    
    // 如果正在播放，重启定时器以应用新的倍速
    if (g_is_playing) {
        gboolean was_playing = g_is_playing;
        stop_playback();
        if (was_playing) {
            start_playback();
        }
    }
}

static void update_progress_bar() {
    if (!g_cap.isOpened() || !g_progress_scale) return;
    
    int total_frames = (int)g_cap.get(cv::CAP_PROP_FRAME_COUNT);
    if (total_frames <= 0) return;
    
    double progress = (double)g_frame_index / total_frames * 100.0;
    
    g_updating_progress = TRUE;
    gtk_range_set_value(GTK_RANGE(g_progress_scale), progress);
    g_updating_progress = FALSE;
}

static void progress_scale_changed(GtkRange *range, gpointer user_data) {
    if (g_updating_progress || !g_cap.isOpened()) return;
    
    double value = gtk_range_get_value(range);
    int total_frames = (int)g_cap.get(cv::CAP_PROP_FRAME_COUNT);
    if (total_frames <= 0) return;
    
    int target_frame = (int)(value / 100.0 * total_frames);
    if (target_frame < 0) target_frame = 0;
    if (target_frame >= total_frames) target_frame = total_frames - 1;
    
    // 停止播放并跳转到目标帧
    gboolean was_playing = g_is_playing;
    stop_playback();
    
    g_cap.set(cv::CAP_PROP_POS_FRAMES, target_frame);
    cv::Mat frame;
    if (g_cap.read(frame)) {
        g_frame_index = (int)g_cap.get(cv::CAP_PROP_POS_FRAMES);
        show_cv_frame(frame);
    }
    
    // 如果之前在播放，继续播放
    if (was_playing) {
        start_playback();
    }
}
#endif
