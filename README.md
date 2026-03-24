# rmdev_control_algorithm

`rmdev` 控制算法模块，当前提供 PID 等常用控制器。

## 依赖

- `emdevif_core`
- `rmdev_math`

## 功能

- 提供可复用的控制算法实现（如 PID）
- 适配模块导入与头文件导入两种使用方式

## 使用

链接 `rmdev` 后，导入对应模块/头文件即可。

```cpp
// 示例（模块方式）
// import rmdev.control_algorithm.pid;
```

## 说明

该模块无独立 CMake 配置项；行为参数建议由调用侧（业务控制循环）管理。
