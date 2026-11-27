/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-04 16:34:00
 * @LastEditors: 於悦洋 yuyueyang2468@163.com
 * @LastEditTime: 2025-11-23 11:22:54
 * @FilePath: /ROBOCON2026_base/src/common_logger/include/common_logger/common_logger.hpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#ifndef COMMON_LOGGER_HPP
#define COMMON_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>
#include <string>
#include <filesystem>

namespace common_logger {

class Logger {
public:
  /**
   * @brief 获取日志单例
   * @param node_name 节点名称
   * @param log_dir 日志目录（默认 ~/robocon_logs）
   * @param max_file_size 单个日志文件最大大小（字节，默认 10MB）
   * @param max_files 最多保留日志文件数（默认 3）
   */
  static std::shared_ptr<Logger> getInstance(
    const std::string& node_name = "default",
    const std::string& log_dir = "",
    size_t max_file_size = 1024 * 1024 * 10,
    size_t max_files = 10
  );

  // 日志级别
  void debug(const std::string& msg);
  void info(const std::string& msg);
  void warn(const std::string& msg);
  void error(const std::string& msg);
  void critical(const std::string& msg);

  // 带格式化的日志
  template<typename... Args>
  void debug(const std::string& fmt, Args&&... args) {
    logger_->debug(fmt, std::forward<Args>(args)...);
  }

  template<typename... Args>
  void info(const std::string& fmt, Args&&... args) {
    logger_->info(fmt, std::forward<Args>(args)...);
  }

  template<typename... Args>
  void warn(const std::string& fmt, Args&&... args) {
    logger_->warn(fmt, std::forward<Args>(args)...);
  }

  template<typename... Args>
  void error(const std::string& fmt, Args&&... args) {
    logger_->error(fmt, std::forward<Args>(args)...);
  }

  template<typename... Args>
  void critical(const std::string& fmt, Args&&... args) {
    logger_->critical(fmt, std::forward<Args>(args)...);
  }

  // 设置日志级别
  void setLevel(spdlog::level::level_enum level);

  // 刷新日志到文件
  void flush();

private:
  Logger(const std::string& node_name,
         const std::string& log_dir,
         size_t max_file_size,
         size_t max_files);
  std::shared_ptr<spdlog::logger> logger_;
  static std::mutex mutex_;
  static std::unordered_map<std::string, std::shared_ptr<Logger>> instances_; // 多实例
};

// 带名称的宏
#define LOG_N_INFO(name, ...)  common_logger::Logger::getInstance(name)->info(__VA_ARGS__)
#define LOG_N_DEBUG(name, ...) common_logger::Logger::getInstance(name)->debug(__VA_ARGS__)
#define LOG_N_WARN(name, ...)  common_logger::Logger::getInstance(name)->warn(__VA_ARGS__)
#define LOG_N_ERROR(name, ...) common_logger::Logger::getInstance(name)->error(__VA_ARGS__)
#define LOG_N_CRITICAL(name, ...) common_logger::Logger::getInstance(name)->critical(__VA_ARGS__)

// 默认实例宏
#define LOG_INFO(...)          common_logger::Logger::getInstance()->info(__VA_ARGS__)
#define LOG_DEBUG(...)         common_logger::Logger::getInstance()->debug(__VA_ARGS__)
#define LOG_WARN(...) common_logger::Logger::getInstance()->warn(__VA_ARGS__)
#define LOG_ERROR(...) common_logger::Logger::getInstance()->error(__VA_ARGS__)
#define LOG_CRITICAL(...) common_logger::Logger::getInstance()->critical(__VA_ARGS__)

} // namespace common_logger

#endif