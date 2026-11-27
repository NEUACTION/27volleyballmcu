/**
 * @Author: hunheyufu yuyueyang2468@163.com
 * @Date: 2026-06-20 02:16:10
 * @LastEditors: hunheyufu yuyueyang2468@163.com
 * @LastEditTime: 2026-06-21 02:08:14
 * @FilePath: app/src/main/cpp/kfs_planner.cpp
 * @Description: 
 * Copyright (c) 2026 by neu_action@163.com, All Rights Reserved. 
 */
#include "action_planner/kfs.hpp"

bool is_inner_kfs(int row, int col)
{
    return row >= 1 && row <= 4 && col >= 1 && col <= 3;
}

bool is_same_kfs(const std::pair<int, int> &lhs, const std::pair<int, int> &rhs)
{
    return lhs.first == rhs.first && lhs.second == rhs.second;
}

bool contains_kfs(const std::deque<std::pair<int, int>> &kfs_list, const std::pair<int, int> &target)
{
    for (const auto &kfs_pos : kfs_list)
    {
        if (is_same_kfs(kfs_pos, target))
        {
            return true;
        }
    }
    return false;
}

bool contains_point(const std::vector<std::pair<int, int>> &points, const std::pair<int, int> &target)
{
    return std::find(points.begin(), points.end(), target) != points.end();
}

void addR2BlocksFromColumn(const std::vector<std::vector<int>> &kfs,
                           std::vector<std::pair<int, int>> &r2Blocks,
                           int col,
                           int expectedCount)
{
    if (col < 1 || col > 3 || static_cast<int>(r2Blocks.size()) >= expectedCount)
    {
        return;
    }

    for (int row = 1; row <= 4 && static_cast<int>(r2Blocks.size()) < expectedCount; row++)
    {
        std::pair<int, int> point = {row, col};
        if (kfs[row][col] == TYPE_R2 && !contains_point(r2Blocks, point))
        {
            r2Blocks.push_back(point);
        }
    }
}

void ensureR2BlockCount(const std::vector<std::vector<int>> &kfs,
                        std::vector<std::pair<int, int>> &r2Blocks,
                        int preferredCol,
                        int expectedCount)
{
    addR2BlocksFromColumn(kfs, r2Blocks, preferredCol, expectedCount);

    for (int offset = 1; static_cast<int>(r2Blocks.size()) < expectedCount && offset <= 2; offset++)
    {
        addR2BlocksFromColumn(kfs, r2Blocks, preferredCol - offset, expectedCount);
        addR2BlocksFromColumn(kfs, r2Blocks, preferredCol + offset, expectedCount);
    }
}

void buildMap(std::vector<std::vector<int>>& map, const std::vector<int>& blockTypes) {
    map.clear();
    for (int i = 0; i < 6; i++) {
        std::vector<int> row;
        for (int j = 0; j < 5; j++) {
            if (i >=1 && i <= 4 && j >= 1 && j <= 3) 
            {
                // 中间 4x3 块区域
                int blockIndex = (i - 1) * 3 + (j - 1);
                row.push_back(blockTypes[blockIndex]);
            }
            else if((i == 0 && j == 0) || (i == 0 && j == 4) || (i == 5 && j == 0) || (i == 5 && j == 4))
            {
                row.push_back(2); // cost = 2
            }
            else
            {
                row.push_back(1); // cost = 1
            }
        }
        map.push_back(row);
    }
}

/**
 * @brief 向右搜索一圈
 * 
 * @param kfs 地图
 * @param r1_kfs 需要取的R1块
 * @param path 路径
 * @return int 取块的代价，如果绕一圈取不全返回-1
 */
int search_path_1(std::vector<std::vector<int>> kfs, std::deque<std::pair<int, int>> r1_kfs, std::vector<std::pair<int, int>> &path)
{
    int cost = 0;
    std::pair<int, int> current = path[0];
    path.clear();

    int collect_kfs = 0;
    while (collect_kfs != 2)
    {
        path.push_back(current);
        cost += kfs[current.first][current.second];
        // 往右走
        if (current.first == 5 && current.second >= 0 && current.second < 4)
        {
            int block_row = current.first - 1;
            int block_col = current.second;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.second++;
        }
            // 往上走
        else if (current.second == 4 && current.first > 0 && current.first <= 5)
        {
            int block_row = current.first;
            int block_col = current.second - 1;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.first--;
        }
            // 往左走
        else if (current.first == 0 && current.second > 0 && current.second <= 4)
        {
            int block_row = current.first + 1;
            int block_col = current.second;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.second--;
        }
            // 往下走
        else if (current.second == 0 && current.first >= 0 && current.first < 5)
        {
            int block_row = current.first;
            int block_col = current.second + 1;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.first++;
        }
        if (current == path[0])
        {
            return -1;
        }
    }

    return cost;
}

/**
 * @brief 向左搜索一圈
 * 
 * @param kfs 地图
 * @param r1_kfs 需要取的R1块
 * @param path 路径
 * @return int 取块的代价，如果绕一圈取不全返回-1
 */
int search_path_2(std::vector<std::vector<int>> kfs, std::deque<std::pair<int, int>> r1_kfs, std::vector<std::pair<int, int>> &path)
{
    int cost = 0;
    std::pair<int, int> current = path[0];
    path.clear();

    int collect_kfs = 0;
    while (collect_kfs != 2)
    {
        path.push_back(current);
        cost += kfs[current.first][current.second];
        // 往左走
        if (current.first == 5 && current.second > 0 && current.second <= 4)
        {
            int block_row = current.first - 1;
            int block_col = current.second;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.second--;
        }
            // 往上走
        else if (current.second == 0 && current.first > 0 && current.first <= 5)
        {
            int block_row = current.first;
            int block_col = current.second + 1;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.first--;
        }
            // 往左走
        else if (current.first == 0 && current.second >= 0 && current.second < 4)
        {
            int block_row = current.first + 1;
            int block_col = current.second;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.second++;
        }
            // 往下走
        else if (current.second == 4 && current.first >= 0 && current.first < 5)
        {
            int block_row = current.first;
            int block_col = current.second - 1;
            // 如果还有需要取的R2路径上的R1块，先取掉
            if (r1_kfs.size() > 0)
            {
                for (int i = 0; i < r1_kfs.size(); i++)
                {
                    if (is_inner_kfs(block_row, block_col) && r1_kfs[i].first == block_row && r1_kfs[i].second == block_col)
                    {
                        collect_kfs++;
                        kfs[block_row][block_col] = 0;
                        r1_kfs.erase(r1_kfs.begin() + i);
                        break;
                    }
                }
            }
            current.first++;
        }
        if (current == path[0])
        {
            return -1;
        }
    }

    return cost;
}

/**
 * @brief 找到R1的路径
 * 
 * @param kfs 地图
 * @param r2_path R2路径 1, 2, 3分别代表第一列、第二列、第三列
 * @return std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, std::deque<std::pair<int, int>>>, int>> 所有R1路径、对应的代价、取的块
 */
std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, std::deque<std::pair<int, int>>>, int>> find_R1_path(std::vector<std::vector<int>> kfs, int r2_path)
{
    // 找到所有R1块的位置
    std::deque<std::pair<int, int>> r1_kfs;
    for (int i = 1; i <= 4; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            if (kfs[i][j] == 1)
            {
                r1_kfs.push_back(std::make_pair(i, j));
            }
        }
    }
    // 找所有r2路径上的R1块
    if (r2_path < 1 || r2_path > 3)
    {
        return {};
    }
    std::deque<std::pair<int, int>> r1_kfs_on_r2_path;
    for (int i = 1; i <= 4; i++)
    {
        if (kfs[i][r2_path] == 1)
        {
            r1_kfs_on_r2_path.push_back(std::make_pair(i, r2_path));
        }
    }
    // 取两个块，其中必须包含全部r1_kfs_on_r2_path
    std::vector<std::deque<std::pair<int, int>>> all_r1_kfs;

    if (r1_kfs_on_r2_path.size() == 2)
    {
        all_r1_kfs.push_back(r1_kfs_on_r2_path);
    }
    else if (r1_kfs_on_r2_path.size() == 1)
    {
        for (const auto &r1_kfs_pos : r1_kfs)
        {
            if (contains_kfs(r1_kfs_on_r2_path, r1_kfs_pos))
            {
                continue;
            }

            std::deque<std::pair<int, int>> selected_r1_kfs = r1_kfs_on_r2_path;
            selected_r1_kfs.push_back(r1_kfs_pos);
            all_r1_kfs.push_back(selected_r1_kfs);
        }
    }
    else if (r1_kfs_on_r2_path.size() == 0)
    {
        for (size_t i = 0; i < r1_kfs.size(); i++)
        {
            for (size_t j = i + 1; j < r1_kfs.size(); j++)
            {
                std::deque<std::pair<int, int>> selected_r1_kfs;
                selected_r1_kfs.push_back(r1_kfs[i]);
                selected_r1_kfs.push_back(r1_kfs[j]);
                all_r1_kfs.push_back(selected_r1_kfs);
            }
        }
    }
    else return {};

    std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, std::deque<std::pair<int, int>>>, int>> result;

    // 对每种R1块组合，分别向右和向左搜索一圈
    for (size_t combo_idx = 0; combo_idx < all_r1_kfs.size(); combo_idx++)
    {
        const auto &r1_kfs_pos = all_r1_kfs[combo_idx];
        
        // 如果需要取左下两个，取不了
        if (contains_kfs(r1_kfs_pos, std::make_pair(4, 1)) && contains_kfs(r1_kfs_pos, std::make_pair(4, 2)))
        {
            continue;
        }

        // 单独处理左右两种走法
        std::pair<int, int> start;
        start = std::make_pair(5, 1);

        // 向右走正常走就行
        std::vector<std::pair<int, int>> right_path{start};
        int cost_right = search_path_1(kfs, r1_kfs_pos, right_path);
        if (cost_right != -1)
        {
            // 接终点到(0, 4)
            std::pair<int, int> current = right_path.back();
            int loop_cnt = 0;
            while(current != std::make_pair(0, 4))
            {
                loop_cnt++;
                if (current.first == 5 && current.second >= 0 && current.second < 4)
                {
                    current.second++;
                }
                else if (current.second == 4 && current.first > 0 && current.first <= 5)
                {
                    current.first--;
                }
                else if (current.first == 0 && current.second >= 0 && current.second < 4)
                {
                    current.second++;
                }
                else if (current.second == 0 && current.first > 0 && current.first <= 5)
                {
                    current.first--;
                }
                else
                {
                    break;
                }
                right_path.push_back(current);
                cost_right += kfs[current.first][current.second];
                if (loop_cnt > 100)
                {
                    break;
                }
            }
            cost_right -= kfs[current.first][current.second];
            result.push_back(std::make_pair(std::make_pair(right_path, r1_kfs_pos), cost_right));
        }

        // 向左需要判断起点
        int add_cost = 0;
        // 需要先取(4, 3)
        if (contains_kfs(r1_kfs_pos, std::make_pair(4, 3)))
        {
            start = std::make_pair(5, 3);
            add_cost = 4;
        }
            // 需要先取(4, 2)
        else if (contains_kfs(r1_kfs_pos, std::make_pair(4, 2)))
        {
            start = std::make_pair(5, 2);
            add_cost = 2;
        }
        else
        {
            start = std::make_pair(5, 1);
        }
        std::vector<std::pair<int, int>> left_path{start};
        int cost_left = search_path_2(kfs, r1_kfs_pos, left_path);

        if (cost_left != -1)
        {
            cost_left += add_cost;
            // 接终点到(0, 4)
            std::pair<int, int> current = left_path.back();
            int loop_cnt = 0;
            while(current != std::make_pair(0, 4))
            {
                loop_cnt++;
                if (current.first == 5 && current.second >= 0 && current.second < 4)
                {
                    current.second++;
                }
                else if (current.second == 4 && current.first > 0 && current.first <= 5)
                {
                    current.first--;
                }
                else if (current.first == 0 && current.second >= 0 && current.second < 4)
                {
                    current.second++;
                }
                else if (current.second == 0 && current.first > 0 && current.first <= 5)
                {
                    current.first--;
                }
                else
                {
                    break;
                }
                left_path.push_back(current);
                cost_left += kfs[current.first][current.second];
                if (loop_cnt > 100)
                {
                    break;
                }
            }
            cost_left -= kfs[current.first][current.second];
            result.push_back(std::make_pair(std::make_pair(left_path, r1_kfs_pos), cost_left));
        }
    }
    return result;
}

/**
 * @brief 找到R2的路径
 * 
 * @param kfs 地图
 * @return std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, int>, int>> 所有R2路径、对应的代价、取哪个块
 */
std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, int>, int>> find_R2_path(std::vector<std::vector<int>> kfs)
{
    int MOVE_COST = 2;
    int THROW_COST = 2;
    int TURN_COST = 7;
    int FIXED_COST = 15;
    int wrong_col = 0; // 3 所在列号
    std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, int>, int>> result; // 输出结果
    std::vector<std::pair<int, int>> take_kfs2;  // 存储 (col, row) 坐标
    int rows = kfs.size();
    int cols = kfs[0].size();
    std::vector<std::vector<int>> height = {
            {0, 0, 0, 0, 0},
            {0, 200, 400, 200, 0},
            {0, 400, 600, 400, 0},
            {0, 600, 400, 200, 0},
            {0, 400, 200, 400, 0},
            {0, 0, 0, 0, 0}
    };

    for (int col = 1; col < cols - 1; col++)
    {
        for (int row = 1; row < rows - 1; row++)
        {
            if (kfs[row][col] == 3)
            {
                wrong_col = col;
                break;
            }
        }
    }

    // 遍历第一行，看是否有2
    for (int col = 1; col < cols - 1; col++)
    {
        if (kfs[4][col] == 2)
        {
            take_kfs2.push_back({col, 4});
        }
    }
    int target_2_count = 2 - take_kfs2.size();

    for(int col = 1; col < cols - 1; col++)
    {
        // 跳过3所在的列
        if (col == wrong_col)
        {
            continue;
        }
        // 跳过3个1所在的列
        int ones_count = 0;
        for(int row = 1; row < rows - 1; row++)
        {
            if (kfs[row][col] == 1)
            {
                ones_count++;
            }
        }
        if(ones_count >= 3)
        {
            continue;
        }

        // 跳过2不足的列
        std::vector<int> adjacent_cols;
        for (int c = std::max(1, col-1); c <= std::min(cols-2, col+1); c++)
        {
            adjacent_cols.push_back(c);
        }
        int total_2s = 0;
        for (int adj_col : adjacent_cols)
        {
            for (int row = 1; row < rows - 2; row++)
            {
                if (kfs[row][adj_col] == 2) total_2s++;
            }
        }
        if (total_2s < target_2_count)
        {
            continue;
        }
        // 每个列迭代使用 take_kfs2 的副本，避免跨迭代污染
        auto working_kfs2 = take_kfs2;

        // 遍历当前列（不包括第一行）查看R2块的数量
        int col_2s = 0;
        for (int row = 1; row < rows - 2; row++)
        {
            if (kfs[row][col] == 2) col_2s++;
        }
        int cost = 0;

        if(col_2s > target_2_count)
        {
            // 第一行两个R2块
            if(target_2_count == 0)
            {
                // 第一排拿一个，路上扔一个
                if(col_2s == 2)
                {
                    if(kfs[4][col] == 2)
                    {
                        working_kfs2.clear();
                        working_kfs2.push_back({col, 4});
                        for(int i = 1; i < rows - 1; i++)
                        {
                            if(kfs[i][col] == 2)
                            {
                                working_kfs2.push_back({col, i});
                                break;
                            }
                        }
                    }
                    else
                    {
                        bool token_col3 = false;
                        for(int i = 0; i < working_kfs2.size(); i++)
                        {
                            if(working_kfs2[i].first == 3)
                            {
                                working_kfs2.erase(working_kfs2.begin() + i);
                                token_col3 = true;
                                break;
                            }
                        }
                        if(!token_col3)
                        {
                            for(int i = 0; i < working_kfs2.size(); i++)
                            {
                                if(working_kfs2[i].first == 1)
                                {
                                    working_kfs2.erase(working_kfs2.begin() + i);
                                    break;
                                }
                            }
                        }
                        for(int i = 1; i < rows - 1; i++)
                        {
                            if(kfs[i][col] == 2)
                            {
                                working_kfs2.push_back({col, i});
                                break;
                            }
                        }
                    }
                    cost += THROW_COST;
                }
                    // 第一排拿1个，路径上不扔
                else if(col_2s == 1)
                {
                    if(kfs[4][col] == 2)
                    {
                        working_kfs2.clear();
                        for(int i = 1; i < rows - 1; i++)
                        {
                            if(kfs[i][col] == 2)
                            {
                                working_kfs2.push_back({col, i});
                            }
                        }
                    }
                    else
                    {
                        // 两边的，删除距离大于1的2
                        if(col != 2)
                        {
                            for(int i = 0; i < working_kfs2.size(); i++)
                            {
                                int current_col = working_kfs2[i].first;
                                int dist = abs(current_col - col);
                                // 删除距离大于1的2
                                if(dist > 1)
                                {
                                    working_kfs2.erase(working_kfs2.begin() + i);
                                    break;
                                }
                            }
                        }
                            // 中间列，删除不顺路的2（第三列）
                        else
                        {
                            int erase_col = 3;
                            for(int i = 0; i < working_kfs2.size(); i++)
                            {
                                if(working_kfs2[i].first == erase_col)
                                {
                                    working_kfs2.erase(working_kfs2.begin() + i);
                                    break;
                                }
                            }
                        }
                        // 添加本列的2
                        for(int i = 1; i < rows - 1; i++)
                        {
                            if(kfs[i][col] == 2)
                            {
                                working_kfs2.push_back({col, i});
                                break;
                            }
                        }
                    }
                }
                // 如果第一排一个不扔，该排为空，一切不变
            }
                // 第一行三个R2块
            else if(target_2_count == -1)
            {
                working_kfs2.clear();
                // 刚好这列拿完
                if(col_2s == 1)
                {
                    for(int i = 1; i < rows - 1; i++)
                    {
                        if(kfs[i][col] == 2)
                        {
                            working_kfs2.push_back({col, i});
                        }
                    }
                }
                    // 扔一个
                else
                {
                    if(col == 1 || col == 2)
                    {
                        working_kfs2.push_back({1, 4});
                        working_kfs2.push_back({2, 4});
                    }
                    else
                    {
                        working_kfs2.push_back({2, 4});
                        working_kfs2.push_back({3, 4});
                    }
                }
            }
                // 第一行1个R2块或没有R2块
            else
            {
                for(int i = 1; i < rows - 1; i++)
                {
                    if(working_kfs2.size() < 2)
                    {
                        if(kfs[i][col] == 2)
                        {
                            working_kfs2.push_back({col, i});
                        }
                    }
                }
                int extra = col_2s - target_2_count;
                cost += extra * THROW_COST;
            }
        }
        else
        {
            std::vector<int> idx_2_in_col;
            for(int i = 1; i < rows - 2; i++)
            {
                if(working_kfs2.size() < 2)
                {
                    if(kfs[i][col] == 2)
                    {
                        working_kfs2.push_back({col, i});
                        idx_2_in_col.push_back(i);
                    }
                }
            }

            int need = 2 - working_kfs2.size();
            if(need > 0)
            {
                std::vector<int> adj_cols;
                for(int c = std::max(1, col - 1); c <= std::min(cols - 2, col + 1); c++)
                {
                    if(c != col) adj_cols.push_back(c);
                }

                auto try_take_best = [&](int avoid_next_row = -1) -> bool
                {
                    int best_row = -1, best_col = -1;
                    int best_score = -1;

                    for(int adj_c : adj_cols)
                    {
                        for(int r = 1; r < rows - 1; r++)
                        {
                            if(kfs[r][adj_c] != 2) continue;
                            if(avoid_next_row != -1 && r == avoid_next_row + 1) continue;

                            bool already_taken = false;
                            for(const auto& taken : working_kfs2)
                            {
                                if(taken.first == adj_c && taken.second == r)
                                {
                                    already_taken = true;
                                    break;
                                }
                            }
                            if(already_taken) continue;

                            int h = height[r][adj_c];
                            int ref_h = (avoid_next_row != -1) ? height[avoid_next_row][col] : 0;
                            int score = (h >= ref_h) ? 10000 + h : h;

                            if(score > best_score)
                            {
                                best_score = score;
                                best_row = r;
                                best_col = adj_c;
                            }
                        }
                    }

                    if(best_row != -1)
                    {
                        working_kfs2.push_back({best_col, best_row});
                        return true;
                    }
                    return false;
                };

                for(const auto& current_idx : idx_2_in_col)
                {
                    if(working_kfs2.size() >= 2) break;
                    if(try_take_best(current_idx)) need--;
                }

                while(working_kfs2.size() < 2)
                {
                    if(!try_take_best()) break;
                }
            }
            int borrowed = target_2_count - col_2s;
            cost += borrowed * TURN_COST;
        }

        if(target_2_count < 2)
        {
            int extra_move = 0;
            // 取第一行的2的移动成本
            for(int i = 0; i < working_kfs2.size(); i++)
            {
                if(working_kfs2[i].second == 4)
                {
                    int current_col = working_kfs2[i].first;
                    extra_move += std::max(0, current_col - col);
                }
            }
            cost += extra_move * MOVE_COST;
        }

        // 翻转working_kfs2的行列
        for(int i = 0; i < working_kfs2.size(); i++)
        {
            std::swap(working_kfs2[i].first, working_kfs2[i].second);
        }

        if(working_kfs2[0].first < working_kfs2[1].first)
        {
            std::swap(working_kfs2[0], working_kfs2[1]);
        }
        else if(working_kfs2[0].first == working_kfs2[1].first)
        {
            if(working_kfs2[0].second > working_kfs2[1].second)
            {
                std::swap(working_kfs2[0], working_kfs2[1]);
            }
        }

        result.push_back({{working_kfs2, col}, cost + FIXED_COST});
    }
    return result;
}



bool is_R1_turn90(std::pair<int, int> coor_R1)
{
    return coor_R1 == std::make_pair(5, 0) || 
           coor_R1 == std::make_pair(0, 4) || 
           coor_R1 == std::make_pair(5, 4) || 
           coor_R1 == std::make_pair(0, 0);
}
void get_taken_R1(std::vector<Point>& r1_target)
{
    for(auto& target : r1_target)
    {
        if(target.first == 1)
        {
            target.first = 0;
        }
        else if(target.first == 4)
        {
            target.first = 5;
        }
        if(target.second == 1)
        {
            target.second = 0;
        }
        else if(target.second == 3)
        {
            target.second = 4;
        }

        if(target.first == 5 && target.second == 4)
        {
            target.second = 3;
        }
        else if(target.first == 0 && target.second == 0)
        {
            target.second = 1;
        }
        else if(target.first == 5 && target.second == 0)
        {
            target.second = 1;
        }
        else if(target.first == 0 && target.second == 4)
        {
            target.second = 3;
        }
    }
}

std::vector<std::pair<Point, int>> cal_R2_wait_time(std::vector<std::vector<int>> kfs, const FindPathResult path, const int R2_WAIT_R1_TIME)
{
    std::vector<std::pair<Point, int>> result;
    int rows = kfs.size();
    int cols = kfs[0].size();
    int R2_col = path.r2ColumnCount;

    std::vector<Point> take_kfs1;

    for(int i = rows - 2; i > 0; i--)
    {
        if(kfs[i][R2_col] == 1)
        {
            take_kfs1.push_back({i, R2_col});
        }
    }
    get_taken_R1(take_kfs1);

    if(!take_kfs1.empty())
    {
        for(auto target : take_kfs1)
        {
            int R1_time = 0, R2_time = 0;
            // 计算R1时间
            for(const auto &R1_path : path.pathPoints)
            {
                if(R1_path != target)
                {
                    R1_time += R1_STEP_TIME;
                    if(is_R1_turn90(R1_path))
                    {
                        R1_time += R1_TURN_90_TIME;
                    }
                    if(R1_path == path.r1CollectBlocks[0])
                    {
                        R1_time += R1_TAKE_TIME;
                    }
                    else if(R1_path == path.r1CollectBlocks[1])
                    {
                        R1_time += R1_TAKE_TIME;
                    }
                }
                else
                {
                    R1_time += R1_TAKE_TIME;
                    break;
                }
            }
            // 计算R2时间
            R2_time += R2_WAIT_R1_TIME;
            std::vector<Point> R2_row4;
            //R2中目标在第一排的
            if(path.r2CollectBlocks[0].first == 4) R2_row4.push_back(path.r2CollectBlocks[0]);
            if(path.r2CollectBlocks[1].first == 4) R2_row4.push_back(path.r2CollectBlocks[1]);
            // 计算R2平移格数
            int r2_step = R2_col - 1;
            for(const auto r2_4 : R2_row4)
            {
                r2_step += 2 * std::max(0, r2_4.second - R2_col);
            }
            R2_time += r2_step * R2_STEP_TIME;
            // 计算R2到目标R1的时间
            for(int row = 4; row > target.first; row--)
            {
                R2_time += R2_CLIMB_TIME;
                // 是否取块
                if(std::make_pair(row, R2_col) == path.r2CollectBlocks[0] 
                || std::make_pair(row, R2_col) == path.r2CollectBlocks[1])
                {
                    R2_time += R2_TAKE_TIME;
                }
                // 是否转身取块
                if(std::make_pair(row, R2_col -1) == path.r2CollectBlocks[0] 
                || std::make_pair(row, R2_col -1) == path.r2CollectBlocks[1]
                || std::make_pair(row, R2_col +1) == path.r2CollectBlocks[0]
                || std::make_pair(row, R2_col +1) == path.r2CollectBlocks[1])
                {
                    R2_time += R2_TURN_TIME;
                    R2_time += R2_TAKE_TIME;
                }
                // 是否扔块
                if(kfs[row][R2_col] == 2 
                && std::make_pair(row, R2_col) != path.r2CollectBlocks[0]
                && std::make_pair(row, R2_col) != path.r2CollectBlocks[1])
                {
                    R2_time += R2_THROW_TIME;
                }
            }
            int wait_time = std::max(0, R1_time - R2_time);
            if(wait_time > 0)
            {
                std::cout << "R1 time: " << R1_time << ", R2 time: " << R2_time << ", wait time: " << wait_time << std::endl;
            }
            result.push_back({target, wait_time});
        }
    }
    return result;
}

R1Result get_R1_result(const std::vector<Point>& r1Path, const std::vector<Point>& r1CollectBlocks)
{
    // r1Path / r1CollectBlocks 坐标格式统一为 {row, col}。
    // 1. result.r1WayNum         -> R1 路径编号
    // 2. result.firstBlockIdx  -> 0-12
    // 3. result.secondBlockIdx -> 0-12
    (void) r1Path;
    (void) r1CollectBlocks;
    
    std::vector<Point> r1CollectBlocksCopy = r1CollectBlocks;
    
    if (r1Path.size() < 2)
    {
        R1Result result;
        return result;
    }
    
    std::vector<int> collected_kfs_idx;
    
    // 先看往哪走
    std::pair<int, int> first_step;
    first_step.first = r1Path[1].first - r1Path[0].first;
    first_step.second = r1Path[1].second - r1Path[0].second;
    R1Result result;
    
    // 往右走
    if (first_step == std::make_pair(0, 1))
    {
        int way_num = 1;
        // 遍历路径
        for (auto &point : r1Path) {
            if (point == std::make_pair(0, 3)) {
                way_num = 3;
            }
            if (point.first == 5 && point.second > 0 && point.second < 4) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(4, point.second)) {
                        collected_kfs_idx.push_back(point.second - 1); // 0 1 2
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
            if (point.second == 4 && point.first > 0 && point.first < 4) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(point.first, 3)) {
                        collected_kfs_idx.push_back(6 - point.first); // 3 4 5
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
            if (point.first == 0 && point.second > 0 && point.second < 3) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(1, point.second)) {
                        collected_kfs_idx.push_back(8 - point.second); // 6 7
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
            if (point.second == 0 && point.first > 1 && point.first < 4) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(point.first, 1)) {
                        collected_kfs_idx.push_back(6 + point.first); // 8 9
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
        }
        result.r1WayNum = way_num;
        result.firstBlockIdx = collected_kfs_idx.size() > 0 ? collected_kfs_idx[0] : -1;
        result.secondBlockIdx = collected_kfs_idx.size() > 1 ? collected_kfs_idx[1] : -1;
    }
    // 另外两条
    else
    {
        int way_num = 2;
        // 遍历路径
        for (auto &point : r1Path) {
            if (point == std::make_pair(1, 4)) {
                way_num = 4;
            }
            if (point.first == 5 && point.second > 0 && point.second < 4) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(4, point.second)) {
                        collected_kfs_idx.push_back(3 - point.second); // 0 1 2
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
            if (point.second == 0 && point.first > 0 && point.first < 4) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(point.first, 1)) {
                        collected_kfs_idx.push_back(6 - point.first); // 3 4 5
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
            if (point.first == 0 && point.second > 1 && point.second < 4) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(1, point.second)) {
                        collected_kfs_idx.push_back(4 + point.second); // 6 7
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
            if (point.second == 4 && point.first > 1 && point.first < 4) {
                for (int i = 0; i < r1CollectBlocksCopy.size(); i++) {
                    if (r1CollectBlocksCopy[i] == std::make_pair(point.first, 3)) {
                        collected_kfs_idx.push_back(6 + point.first); // 8 9
                        r1CollectBlocksCopy.erase(r1CollectBlocksCopy.begin() + i);
                        break;
                    }
                }
            }
        }
        
        result.r1WayNum = way_num;
        result.firstBlockIdx = collected_kfs_idx.size() > 0 ? collected_kfs_idx[0] : -1;
        result.secondBlockIdx = collected_kfs_idx.size() > 1 ? collected_kfs_idx[1] : -1;
        if (way_num == 4)
        {
            result.firstBlockIdx = result.firstBlockIdx < 0 ? -1 : (result.firstBlockIdx > 0 ? result.firstBlockIdx - 1 : 9);
            result.secondBlockIdx = result.secondBlockIdx < 0 ? -1 : (result.secondBlockIdx > 0 ? result.secondBlockIdx - 1 : 9);
        }
    }
    return result;
}

std::vector<int> get_R2_result(int r2PathColumn, const std::vector<Point>& r2CollectBlocks, const std::vector<int>& blockTypes)
{
    std::vector<std::vector<int>> kfs;
    buildMap(kfs, blockTypes);
    
    std::vector<int> result = {0, 0, 0, 0, 0};
    std::vector<Point> blocks = r2CollectBlocks;
    
    for (auto block : blocks)
    {
        // 先看最下面一行
        if (block.first == 4) {
            if (block.second == 1) {
                result[4] |= 1;
            } else if (block.second == 2) {
                result[4] |= 2;
            } else if (block.second == 3) {
                result[4] |= 4;
            }
        }
        // 这里面只取两边的
        else
        {
            if (block.second == r2PathColumn - 1) {
                result[4 - block.first] |= 2;
            }
            else if (block.second == r2PathColumn + 1) {
                result[4 - block.first] |= 4;
            }
        }
    }
    
    // 最后看路径上的
    int col = r2PathColumn;
    for (int i = 4; i >= 1; i--)
    {
        if (kfs[i][col] == 2)
        {
            result[3 - i] |= 1;
        }
    }

    return result;
}



bool checkKfs(const std::vector<int>& blockTypes) {
    // blockTypes 固定传入 12 个块：
    // 0 = 未选择，1 = R1，2 = R2，3 = 假。
    // 返回 true 表示通过检查，false 表示不通过。
    
    std::vector<std::vector<int>> kfs;
    buildMap(kfs, blockTypes);
    
    int fake_count = 0;
    int r1_count = 0;
    int r2_count = 0;

    for (int i = 1; i <= 4; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            if (kfs[i][j] == TYPE_FAKE)
            {
                fake_count++;
                if (i == 4)
                {
                    return false;
                }
            }
            else if (kfs[i][j] == TYPE_R1)
            {
                r1_count++;
                // 判断不在中心
                if ((i > 1 && i < 4) && (j > 1 && j < 3))
                {
                    return false;
                }
            }
            else if (kfs[i][j] == TYPE_R2)
            {
                r2_count++;
            }
            else if (kfs[i][j] != TYPE_EMPTY)
            {
                return false;
            }
        }
    }

    if (fake_count != 1 || r1_count != 3 || r2_count != 4) {
        return false;
    }
    return true;
}

void extractBlockTypes(const std::vector<std::vector<int>>& map, std::vector<int>& blockTypes) {
    // 1. 清空目标数组，准备接收新数据
    blockTypes.clear();

    // 2. 防御性检查：确保传入的 map 尺寸是 6x5
    if (map.size() != 6) {
        // 尺寸不合法，直接返回空数组或抛出异常
        return; 
    }

    // 3. 遍历中间的 4x3 区域
    for (int i = 1; i <= 4; ++i) {
        // 防御性检查：确保每一行的列数是 5
        if (map[i].size() != 5) {
            blockTypes.clear(); // 数据损坏，清空并返回
            return;
        }
        for (int j = 1; j <= 3; ++j) {
            // 使用与 buildMap 完全相同的索引计算公式
            int blockIndex = (i - 1) * 3 + (j - 1);
            
            // 提取中间区域的值，存入一维数组
            blockTypes.push_back(map[i][j]);
        }
    }
}

FindPathResult findPath(const std::vector<int>& blockTypes) {
    // blockTypes 固定传入 12 个块：
    // 0 = 未选择，1 = R1，2 = R2，3 = 假。
    //
    // 需要填充三组结果：
    // 1. result.pathPoints      -> 路径上的每个点，格式 {row, col}
    // 2. result.r1CollectBlocks -> 需要取的 R1 块坐标，格式 {row, col}
    // 3. result.r2CollectBlocks -> 需要取的 R2 块坐标，格式 {row, col}
    // 4. result.r2ColumnCount   -> R2 走的列数
    //
    // 中间 4x3 块坐标建议使用：row=1~4，col=1~3。
    // 四周路径坐标建议使用：6x5 地图坐标，row=0~5，col=0~4。
    std::vector<std::vector<int>> kfs;
    buildMap(kfs, blockTypes);

    std::vector<std::pair<std::pair<std::vector<std::pair<int, int>>, int>, int>> R2_result = find_R2_path(kfs);

    std::pair<std::pair<std::vector<std::pair<int, int>>, std::deque<std::pair<int, int>>>, int> R1_choice;
    std::pair<std::pair<std::vector<std::pair<int, int>>, int>, int> R2_choice;

    int bestR2Cost = 9999999;
    int bestR1Cost = 9999999;
    bool hasChoice = false;
    for (int i = 0; i < R2_result.size(); i++)
    {
        auto R1_result = find_R1_path(kfs, R2_result[i].first.second);
        for (int j = 0; j < R1_result.size(); j++)
        {
            int r2Cost = R2_result[i].second;
            if (!R1_result[j].first.first.empty() && R1_result[j].first.first.front() == std::make_pair(5, 3))
            {
                if (R2_result[i].first.second == 3)
                {
                    r2Cost += 2;
                }
                else if (R2_result[i].first.second == 2)
                {
                    r2Cost += 4;
                }
                else {
                    r2Cost += 6;
                }
            }
            else if (!R1_result[j].first.first.empty() && R1_result[j].first.first.front() == std::make_pair(5, 2))
            {
                if (R2_result[i].first.second == 2)
                {
                    r2Cost += 2;
                }
                else if (R2_result[i].first.second == 1)
                {
                    r2Cost += 4;
                }
            }

            int r1Cost = R1_result[j].second;
            if (r1Cost > r2Cost)
            {
                continue;
            }

            if (!hasChoice || r2Cost < bestR2Cost || (r2Cost == bestR2Cost && r1Cost < bestR1Cost))
            {
                bestR2Cost = r2Cost;
                bestR1Cost = r1Cost;
                R1_choice = R1_result[j];
                R2_choice = R2_result[i];
                hasChoice = true;
            }
        }
    }

    FindPathResult result;
    if (!hasChoice) {
        return result;
    }

    result.pathPoints = R1_choice.first.first;
    result.r1CollectBlocks = std::vector<std::pair<int, int>>(R1_choice.first.second.begin(), R1_choice.first.second.end());
    result.r2CollectBlocks = R2_choice.first.first;
    result.r2ColumnCount = R2_choice.first.second;

    return result;
}

int convertR2Result(FindPathResult result)
{
    int r2ColumnCount = 0;
    for(int i = 1; i <= result.r2CollectBlocks.size(); i++)
    {
        if(result.r2CollectBlocks[i].first == 4)
        {
            if(result.r2CollectBlocks[i].second == 1)
            {
                r2ColumnCount += 4;
            }
            else if(result.r2CollectBlocks[i].second == 2)
            {
                r2ColumnCount += 2;
            }
            else if(result.r2CollectBlocks[i].second == 3)
            {
                r2ColumnCount += 1;
            }
        }
    }
    return r2ColumnCount;
}