/*
 * @Author: 於悦洋 yuyueyang2468@163.com
 * @Date: 2025-11-04 16:31:39
 * @LastEditors: 於悦洋 yuyueyang2468@163.com
 * @LastEditTime: 2025-11-04 16:40:31
 * @FilePath: /ROBOCON2026_base/src/common_logger/src/logger_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by Action, All Rights Reserved. 
 */
#include "common_logger/common_logger.hpp"
#include <spdlog/pattern_formatter.h>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace common_logger {

std::unordered_map<std::string, std::shared_ptr<Logger>> Logger::instances_;
std::mutex Logger::mutex_;

std::shared_ptr<Logger> Logger::getInstance(
  const std::string& node_name,
  const std::string& log_dir,
  size_t max_file_size,
  size_t max_files)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = instances_.find(node_name);
  if (it != instances_.end()) {
    return it->second;
  }
  auto inst = std::shared_ptr<Logger>(
    new Logger(node_name, log_dir, max_file_size, max_files)
  );
  instances_[node_name] = inst;
  return inst;
}

Logger::Logger(
  const std::string& node_name,
  const std::string& log_dir,
  size_t max_file_size,
  size_t max_files)
{
  // 获取当前时间
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now;
  localtime_r(&time_t_now, &tm_now);

  // 格式化时间字符串: yyyy-MM-dd-hh-mm
  std::ostringstream time_stream;
  time_stream << std::setfill('0')
              << std::setw(4) << (tm_now.tm_year + 1900) << "-"
              << std::setw(2) << (tm_now.tm_mon + 1) << "-"
              << std::setw(2) << tm_now.tm_mday << "-"
              << std::setw(2) << tm_now.tm_hour << "-"
              << std::setw(2) << tm_now.tm_min;
  std::string timestamp = time_stream.str();

  // 确定日志目录: ./record/logs/<node_name>/
  std::string dir;
  if (!log_dir.empty()) {
    dir = log_dir;
  } else {
    dir = "./record/logs/" + node_name;
  }

  // 创建日志目录
  std::filesystem::create_directories(dir);

  // 日志文件路径: ./record/logs/<node_name>/yyyy-MM-dd-hh-mm.txt
  std::string log_file = dir + "/" + timestamp + ".txt";

  // 创建 sinks
  std::vector<spdlog::sink_ptr> sinks;
  
  // 1. 控制台输出（彩色）
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);
  sinks.push_back(console_sink);

  // 2. 文件输出（滚动）
  auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
    log_file, max_file_size, max_files
  );
  file_sink->set_level(spdlog::level::trace);
  sinks.push_back(file_sink);

  // 创建 logger
  logger_ = std::make_shared<spdlog::logger>(node_name, sinks.begin(), sinks.end());
  
  // 设置日志格式: [时间] [节点名] [级别] 消息
  logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%f] [%n] [%^%l%$] %v");
  
  // 设置默认级别
  logger_->set_level(spdlog::level::debug);
  logger_->flush_on(spdlog::level::info);      // info 即写
  spdlog::flush_every(std::chrono::milliseconds(100)); // 定期刷新
  logger_->info("Logger initialized, file: {}", log_file);
}

void Logger::debug(const std::string& msg) {
  logger_->debug(msg);
}

void Logger::info(const std::string& msg) {
  logger_->info(msg);
}

void Logger::warn(const std::string& msg) {
  logger_->warn(msg);
}

void Logger::error(const std::string& msg) {
  logger_->error(msg);
}

void Logger::critical(const std::string& msg) {
  logger_->critical(msg);
}

void Logger::setLevel(spdlog::level::level_enum level) {
  logger_->set_level(level);
}

void Logger::flush() {
  logger_->flush();
}

}  // namespace common_logger