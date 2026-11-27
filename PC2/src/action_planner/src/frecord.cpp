#include "action_planner/frecord.hpp"
#include "action_planner/leg.hpp"
#include "action_planner/action_planner_node.hpp"
#include "action_planner/sword.hpp"

Recorder recorder1("mydebug");
Recorder l515("l515_data");


/// @brief 创建输出文件
/// @param foldername 子文件夹名
frecord::frecord(std::string foldername)
{
  std::string stime = this->GetTimeString();

  //生成record文件夹
  if (opendir(FILE_PATH) == NULL) 
  {
    if (mkdir(FILE_PATH, 0777) != 0)
    {
      std::cout << "cannot mkdir " << FILE_PATH << std::endl;
    }
	}

  //生成子文件夹++
  filename = FILE_PATH + foldername + "/";

  if (opendir(filename.c_str()) == NULL) 
  {
    //0777 是权限标志，表示该目录对所有用户（文件所有者、组用户和其他用户）都有读、写、执行权限。
    if (mkdir(filename.c_str(), 0777) != 0)
    {
      std::cout << "cannot mkdir " << filename << std::endl;
    }
	}
  
  //获取debug文件名
  filename = filename + stime + " " + foldername + ".txt";
  
  //以只写模式打开，没有文件就创建
  recordfile.open(filename,std::fstream::out);
}

//关闭文件
frecord::~frecord()
{
    if (recordfile.is_open()) {
        recordfile.close();
    }
}
 

/// @brief 将int类型转化为string类型 并补0
/// @param num int类型数据
/// @param std::stringstream 是 C++ 标准库中的一个类，用于在内存中构建字符串。
/// @param iss 是一个 std::stringstream 类型的对象，用于将整数转换为字符串
/// @return string类型数据
std::string frecord::Convert(int num)
{
  std::stringstream iss;
  if (num < 10)
  {
    //如果 num 小于 10，将数字 0 插入到字符串流 iss 中。这样做的目的是在单个位数的数字前添加一个前导零。
    //因而在文件名称中才会对称
    iss << 0;
  }
  iss << num;
  return iss.str();
}



/// @brief 获取y-m-d h-min 的时间字符串
/// @return 时间字符串
std::string frecord::GetTimeString()
{
    auto now = std::chrono::system_clock::now();
    std::time_t timeReal = std::chrono::system_clock::to_time_t(now);
    std::tm* t = std::gmtime(&timeReal);

    std::ostringstream oss;
    oss << Convert(t->tm_year + 1900) << "-"
        << Convert(t->tm_mon + 1) << "-"
        << Convert(t->tm_mday) << " "
        << Convert(t->tm_hour + 8) << "-"
        << Convert(t->tm_min);

    return oss.str();
}

Recorder::Recorder(std::string dataName)
{
    recordFile = std::make_shared<frecord>(dataName);
    dataOrderVector = std::make_shared<std::vector<std::string>>();
    writer_running_.store(true, std::memory_order_release);
    writer_thread_ = std::thread(&Recorder::WriterLoop, this);
}

Recorder::~Recorder()
{
    writer_running_.store(false, std::memory_order_release);
    write_cv_.notify_one();
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    if (this->recordFile && this->recordFile->recordfile.is_open()) {
        this->recordFile->recordfile.flush();
        this->recordFile->recordfile.close();
    }
}

void Recorder::EnqueueLine(std::string line)
{
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_queue_.push_back(std::move(line));
    }
    write_cv_.notify_one();
}

void Recorder::WriterLoop()
{
    std::deque<std::string> local_queue;

    while (writer_running_.load(std::memory_order_acquire)) {
        {
            std::unique_lock<std::mutex> lock(write_mutex_);
            write_cv_.wait(lock, [this]() {
                return !writer_running_.load(std::memory_order_acquire) || !write_queue_.empty();
            });
            local_queue.swap(write_queue_);
        }

        while (!local_queue.empty()) {
            this->recordFile->recordfile << local_queue.front();
            local_queue.pop_front();
        }
        this->recordFile->recordfile.flush();
    }

    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        local_queue.swap(write_queue_);
    }

    while (!local_queue.empty()) {
        this->recordFile->recordfile << local_queue.front();
        local_queue.pop_front();
    }
    this->recordFile->recordfile.flush();
}

void Recorder::PrintToFile()
{
    std::ostringstream buffer; // 创建一个字符串流作为缓存

    for (auto it = dataOrderVector->begin(); it != dataOrderVector->end(); it++) 
    {
        const std::string &key = *it;
        const auto &record = dataRecordMap[key];

        if(it != dataOrderVector->begin())
        {
          // 访问dataValue，并根据类型输出到缓存
          buffer << "\t" << key << "\t" ;
        }
        else
        {
          buffer << "\t" << key << "\t" ;
        }

        std::visit(
            [&buffer](auto&& arg) 
            {
                buffer <<  std::setw(12) << arg;
            },
            record);
    }

    buffer << std::endl;

    EnqueueLine(buffer.str());
}

void RecordData(){
  auto time = rclcpp::Clock().now();
  string time_str = std::to_string((int)time.seconds()) + "." + std::to_string((int)((time.seconds() - (int)time.seconds()) * 1e6));
  RECORD(time_str, time);
  RECORD(plannerNode->cnt, cnt);
  // RECORD(plannerNode->myFlags.RB_flag, RB_flag);


  recorder1.PrintToFile();
}