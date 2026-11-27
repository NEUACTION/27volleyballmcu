/*** 
 * @Author: hunheyufu yuyueyang2468@163.com
 * @Date: 2026-06-22 00:47:38
 * @LastEditors: hunheyufu yuyueyang2468@163.com
 * @LastEditTime: 2026-06-22 05:14:02
 * @FilePath: /kfs_ultra/inc/kfs.hpp
 * @Description: 
 * @
 * @Copyright (c) 2026 by ACTION, All Rights Reserved. 
 */
#ifndef RCAPP_KFS_PLANNER_H
#define RCAPP_KFS_PLANNER_H

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

using Point = std::pair<int, int>;

constexpr int TYPE_EMPTY = 0;
constexpr int TYPE_R1 = 1;
constexpr int TYPE_R2 = 2;
constexpr int TYPE_FAKE = 3;

// 路径时间参数，单位：ms。
inline constexpr int R1_STEP_TIME = 550;
inline constexpr int R1_TURN_90_TIME = 300;
inline constexpr int R1_TAKE_TIME = 500;

inline constexpr int R2_STEP_TIME = 350;
inline constexpr int R2_THROW_TIME = 250;
inline constexpr int R2_TURN_TIME = 3600;
inline constexpr int R2_TAKE_TIME = 500;
inline constexpr int R2_CLIMB_TIME = 1800;

struct FindPathResult {
    std::vector<Point> pathPoints;
    std::vector<Point> r1CollectBlocks;
    std::vector<Point> r2CollectBlocks;
    int r2ColumnCount = 0;
};

struct R1Result {
    int r1WayNum = 0;
    int firstBlockIdx = -1;
    int secondBlockIdx = -1;
};

void buildMap(std::vector<std::vector<int>>& map, const std::vector<int>& blockTypes);
int search_path_1(std::vector<std::vector<int>> kfs, std::deque<std::pair<int, int>> r1_kfs, std::vector<std::pair<int, int>> &path);
int search_path_2(std::vector<std::vector<int>> kfs, std::deque<std::pair<int, int>> r1_kfs, std::vector<std::pair<int, int>> &path);
std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, std::deque<std::pair<int, int>>>, int>> find_R1_path(std::vector<std::vector<int>> kfs, int r2_path);
std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, int>, int>> find_R2_path(std::vector<std::vector<int>> kfs);
// 辅助函数：判断R1是否为角落
bool is_R1_turn90(std::pair<int, int> coor_R1);
void get_taken_R1(std::vector<Point>& r1_target);
std::vector<std::pair<Point, int>> cal_R2_wait_time(std::vector<std::vector<int>> kfs, const FindPathResult path, const int R2_WAIT_R1_TIME);

/**
 * 根据 R1 路径和 R1 要取的块，返回 R1 路径编号和左右手取块号。
 * r1Path / r1CollectBlocks 坐标格式统一为：{row, col}。
 * firstBlockIdx / secondBlockIdx 使用 R1 取块顺序编号 0~9，-1 表示未设置。
 */
R1Result get_R1_result(const std::vector<Point>& r1Path, const std::vector<Point>& r1CollectBlocks);

/**
 * 根据 R2 路径列和 R2 要取的块坐标，返回 R2 结果。
 * r2CollectBlocks 坐标格式统一为：{row, col}。
 * 返回值格式：{action1, action2, action3, action4, FirstR2KFS}。
 * action1~action4 分别对应 row=1~4：0=无动作，1=本列取，2=左侧取，3=右侧取。
 * FirstR2KFS 使用 1~12 块号，0 表示未设置。
 */
std::vector<int> get_R2_result(int r2PathColumn, const std::vector<Point>& r2CollectBlocks, const std::vector<int>& blockTypes);

/**
 * 检查 12 个块是否符合 KFS 规则。
 * blockTypes: 0=未选择, 1=R1, 2=R2, 3=假。
 */
bool checkKfs(const std::vector<int>& blockTypes);

/**
 * 传入 12 个块的类型，返回路径、R1取块、R2取块和 R2 走的列数。
 * blockTypes: 0=未选择, 1=R1, 2=R2, 3=假。
 */
FindPathResult findPath(const std::vector<int>& blockTypes);

int convertR2Result(
    FindPathResult result
);

void extractBlockTypes(const std::vector<std::vector<int>>& map, std::vector<int>& blockTypes);

#endif // RCAPP_KFS_PLANNER_H

