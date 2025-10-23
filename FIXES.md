# 🔧 示波器修复说明

## 已修复的问题

### 1. ✅ 中文乱码问题
- 在构造函数中添加了UTF-8编码设置
- 使用 `g_setenv("LANG", "zh_CN.UTF-8", TRUE)`

### 2. ✅ 无法添加变量问题
- 添加了CSV路径保存功能
- 改进了变量加载逻辑
- 添加了友好的提示信息

### 3. ✅ 用户体验改进
- 添加了"请先加载CSV"的提示
- 打开示波器时自动加载已有的CSV
- 添加了调试输出信息

## 📝 使用步骤（重要）

### 步骤1：关闭正在运行的程序
**必须先关闭imageprocessor.exe，否则无法重新编译！**

### 步骤2：重新编译
```batch
.\build.bat
```

### 步骤3：运行程序
```batch
.\run.bat
```

### 步骤4：使用示波器
1. **先加载CSV**：点击主窗口的"加载日志CSV"按钮
2. **打开示波器**：点击"📊 示波器"按钮
3. **添加通道**：从下拉框选择变量，点击"添加通道"
4. **播放视频**：波形会自动更新

## 🎯 测试CSV示例

创建一个测试CSV文件（test_data.csv）：

```csv
host_recv_iso,log_text_hex,log_text_utf8,温度,速度,电压
2025-10-24 10:00:00.001,48656C6C6F,Hello,25.5,120,3.3
2025-10-24 10:00:00.034,576F726C64,World,26.1,125,3.2
2025-10-24 10:00:00.067,54657374,Test,25.8,118,3.4
2025-10-24 10:00:00.101,44617461,Data,27.2,130,3.5
2025-10-24 10:00:00.134,496E666F,Info,26.5,122,3.3
```

## 🐛 如果还有问题

### 中文仍然乱码
尝试设置系统编码：
```cpp
// 在 oscilloscope.cpp 的构造函数中已添加
g_setenv("LANG", "zh_CN.UTF-8", TRUE);
```

### 下拉框仍然是空的
检查：
1. CSV文件是否正确加载？
2. CSV中是否有除了时间戳、hex、utf8之外的列？
3. 控制台是否输出"成功加载CSV"信息？

### 添加通道没反应
检查：
1. 是否选中了有效的变量（不是提示文本）？
2. 控制台是否输出"已添加通道"信息？

## 📊 调试信息

程序会在控制台输出以下信息：
```
[示波器] 成功加载CSV，找到 3 个变量
[示波器] 已添加通道: 温度
[示波器] 通道 '温度' 已存在
```

如果看不到这些信息，说明可能有问题。

## ⚙️ 技术细节

### 修改的文件
1. `src/oscilloscope.cpp`
   - 添加UTF-8编码设置
   - 改进loadCSV函数
   - 添加通道验证
   - 添加提示信息

2. `src/main.cpp`
   - 添加g_csv_file_path全局变量
   - 改进open_oscilloscope_clicked
   - 保存CSV路径

### 关键代码片段

#### 1. UTF-8编码设置
```cpp
// 在构造函数中
g_setenv("LANG", "zh_CN.UTF-8", TRUE);
```

#### 2. CSV路径保存
```cpp
// 加载CSV时保存路径
g_csv_file_path = filename;
```

#### 3. 自动加载到示波器
```cpp
if (!g_csv_file_path.empty() && g_csv_reader.getRecordCount() > 0) {
    g_oscilloscope->loadCSV(g_csv_file_path);
    g_oscilloscope->updateDisplay(g_frame_index);
}
```

---

**编译前请确保关闭所有正在运行的imageprocessor.exe实例！**
