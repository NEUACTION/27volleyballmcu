#ifndef _FRECORD_HPP_
#define _FRECORD_HPP_


#include <dirent.h>
#include <sys/stat.h>
#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <map>
#include <variant>
#include <chrono>
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

//输出文件路径
#define FILE_PATH "./record/"
#define RECORD(a,b) recorder1.PushToRecord(a,#b)
class frecord
{
public:
  explicit frecord(std::string foldername);
  ~frecord();
private:
  std::string Convert(int num);
  std::string GetTimeString();
public:
  std::fstream recordfile;
  std::string filename;
};

union dataUnion
{
  float floatType;
  bool boolType;
  int32_t intType;
  uint32_t uintType;

};
class Recorder
{
    public:
        std::shared_ptr<frecord> recordFile;                                            //指向输出文件类的指针
        std::map<std::string, std::variant<float, bool, int, uint32_t, std::string>> dataRecordMap;  //存储数据的map,std::variant<float, bool, int, uint32_t>是高级联合体
        std::shared_ptr<std::vector<std::string>> dataOrderVector;                      //存储数据的顺序
    public:
        Recorder(std::string dataName);
        ~Recorder();
        template <typename T> void PushToRecord(T data, std::string dataName);          //将数据存入map
        void PrintToFile();    
        private:
        void EnqueueLine(std::string line);
        void WriterLoop();

        std::mutex write_mutex_;
        std::condition_variable write_cv_;
        std::deque<std::string> write_queue_;
        std::thread writer_thread_;
        std::atomic<bool> writer_running_{false};                                                         //将数据写入文件
};



template <typename T>
/**
 * @brief 将数据推送到记录器中
 * @tparam T 数据类型，支持 float、double、bool、有符号和无符号整数类型
 * @param data 要记录的数据
 * @param dataName 数据的名称标识符
 * 
 * @details 该函数将输入数据转换为标准类型(float、bool、int、uint32_t)并存储在记录器中。
 * 如果已存在同名数据则更新其值，否则创建新条目并记录数据名称顺序。
 * 数据存储使用 variant 实现类型安全的多类型存储。
 */
void Recorder::PushToRecord(T data, std::string dataName)
{
  std::variant<float, bool, int, uint32_t, std::string> tempData;

  if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>)
  {
    tempData = (float)data;
  }
  else if constexpr (std::is_same_v<T, bool>)
  {
    tempData = (bool)data;
  }
  else if constexpr (std::is_integral_v<T>)
  {

    if constexpr (std::is_unsigned_v<T>)
    {
      tempData = (uint32_t)data;
    }
    else
    {
      tempData = (int)data;
    }
    
  }
  else if constexpr (std::is_same_v<T, std::string>)
  {
    tempData = (std::string)data;
  }
  else
  {
    return;
  }


  std::map<std::string, std::variant<float, bool, int, uint32_t, std::string>>::iterator it = dataRecordMap.find(dataName);

  //检查是否已经有同名的数据
  if(it != dataRecordMap.end())
  {
    {
      //相同就更新
      dataRecordMap[dataName] = std::move(tempData);
      return;
    }
  }
  else
  {
    //如果没有同名数据，就新建并插入
    dataRecordMap[dataName] = tempData;
    dataOrderVector->emplace_back(dataName);
  }
}
void RecordData();
extern Recorder recorder1;
extern Recorder l515;
extern Recorder recorder1;
#endif