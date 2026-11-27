#include "action_planner/robot.hpp"
#include "action_planner/action_planner_node.hpp"
#include "action_planner/kfs.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
#include <utility>

//进行坐标转换（输入x,y得到编号)
int Lrz::ChangeToNum(int _x, int _y)
{
	return 3 * (_y - 1) + _x;
}

int Lrz::CheckKFSInR1Way(int way[6], int kfs)
{
	for(int i = 0;i < 6; ++i)
    {
        if(way[i] == kfs)
        {
            return i;
        }
    }

	return -1;
}

void Lrz::GoNext(R2PathPlan &plan)
{
    plan.nowPosY++;
}

void Lrz::FlatMove(R2PathPlan &plan, int m)//m给1或-1.横移方向
{
    plan.nowPosX += m;
    plan.nowcost += FLAT_COST;
}

void Lrz::takeKFS(R2PathPlan &plan,int Map[10][10],int m)//m为取块方向，0前1左2右
{    
    if (m == 0)
    {
        Map[plan.nowPosX][plan.nowPosY + 1] = 0;
        if(plan.nowPosY)
        {
            plan.Action[plan.nowPosY] = plan.Action[plan.nowPosY] | 0x01;
        }
        else 
        {
            plan.FirsrR2KFS = plan.FirsrR2KFS | (1 << (plan.nowPosX - 1));
        }
    }
    else if (m == 1)
    {
        Map[plan.nowPosX - 1][plan.nowPosY] = 0;
        plan.Action[plan.nowPosY] = plan.Action[plan.nowPosY] | 0x02;
    }
    else if (m == 2)
    {
        Map[plan.nowPosX + 1][plan.nowPosY] = 0;
        plan.Action[plan.nowPosY] = plan.Action[plan.nowPosY] | 0x04;
    }
    
    plan.now_R2_KFS_NUM++;
    
    if(plan.now_R2_KFS_NUM > 2)
    {
        plan.nowcost += THROW_COST;
    }

    if (m)
    {
        plan.nowcost += TURN_COST;
    }
}

void Lrz::RecordMap(int RMap[10][10], int OMap[10][10])
{
    for(int i = 0; i <= 3; ++i)
    {
        for(int j = 0; j <= 4; ++j)
        {
            RMap[i][j] = OMap[i][j];
        }
    }
}

void Lrz::FindPath(R2PathPlan nowPlan,int nowMap[10][10])
{
    if(nowPlan.nowcost > NowBestCost || nowPlan.now_R1_KFS_NUM > 2 || nowMap[nowPlan.nowPosX][nowPlan.nowPosY] == 3)//超过最大花费剪枝，R1最多取两个块,不能碰假块
    {
        return;
    }
    
    if(nowPlan.nowPosY && !nowPlan.FirsrR2KFS && nowPlan.FirstR2KFS_flag)//如果第一行有块而且没取，直接剪枝
    {
        return;
    }
        
    if(nowPlan.nowPosY == 5)
    {
        if(nowPlan.now_R2_KFS_NUM < 2)
        {
            return;
        }

        nowPlan.nowcost += std::abs(nowPlan.nowPosX - nowPlan.endPosX) * FLAT_COST;
        
        if(nowPlan.nowcost > NowBestCost)
        {
            return;
        }

        nowPlan.way_num = nowPlan.nowPosX;
        R2 = nowPlan;
        NowBestCost = R2.nowcost;
        
        return;
    }
    
    //记录当前地图
    int RecordedMap[10][10];
    int nowx = nowPlan.nowPosX;
    int nowy = nowPlan.nowPosY;
    R2PathPlan orgPlan = nowPlan;
    RecordMap(RecordedMap,nowMap);

    if(nowy == 0)
    {
        if (RecordedMap[nowx][1] == 2)
        {
            takeKFS(nowPlan,RecordedMap,0);
            //前左右移动都试一下
            R2PathPlan TestRBT = nowPlan;
            GoNext(nowPlan);
            FindPath(nowPlan,RecordedMap);
            nowPlan = TestRBT;
            
            if (nowx < 3)
            {
                FlatMove(nowPlan,1);
                FindPath(nowPlan,RecordedMap);
                nowPlan = TestRBT;
            }
            if (nowx > 1)
            {
                FlatMove(nowPlan,-1);
                FindPath(nowPlan,RecordedMap);
                nowPlan = orgPlan;
            }            
        }
        
        GoNext(nowPlan);
        FindPath(nowPlan,RecordedMap);
        nowPlan = orgPlan;
        
        if (nowx < 3)
        {
            FlatMove(nowPlan,1);
            FindPath(nowPlan,RecordedMap);
            nowPlan = orgPlan;
        }
        
        if(nowx > 1)
        {
            FlatMove(nowPlan,-1);
            FindPath(nowPlan,RecordedMap);
            nowPlan = orgPlan;
        }        
        
        return;
    }
    
    if(RecordedMap[nowx][nowy] == 1)
    {
        nowPlan.now_R1_KFS_NUM ++;
        
        if(nowPlan.Must_Take_R1_1 < 0)
        {
            nowPlan.Must_Take_R1_1 = ChangeToNum(nowx,nowy);
        }
        else 
        {
            nowPlan.Must_Take_R1_2 = ChangeToNum(nowx,nowy);
        }
    }
    
    if(nowMap[nowx][nowy + 1] == 2)
    {
        // 尝试向前取块
        takeKFS(nowPlan,RecordedMap,0);
        FindPath(nowPlan,RecordedMap);
        GoNext(nowPlan);
        FindPath(nowPlan,RecordedMap);//递推，枚举同一位置取多块的情况
        
        return;
    }

    //前面必须取
    R2PathPlan This_plan = nowPlan;
    int this_map[10][10];
    RecordMap(this_map,RecordedMap);//复位
    
    if(nowMap[nowx - 1][nowy] == 2)
    {
        // 尝试向左取块
        takeKFS(nowPlan,this_map,1);
        GoNext(nowPlan);
        FindPath(nowPlan,this_map);//递推，枚举同一位置取多块的情况
    }
    
    nowPlan = This_plan; 
    RecordMap(this_map,RecordedMap);//复位
    
    if(nowMap[nowx + 1][nowy] == 2)
    {
        // 尝试向右取块
        takeKFS(nowPlan,this_map,2);
        GoNext(nowPlan);
        FindPath(nowPlan,this_map);//递推，枚举同一位置取多块的情况
    }
    
    nowPlan = This_plan; 
    RecordMap(this_map,RecordedMap);//复位
    GoNext(nowPlan);
    FindPath(nowPlan,this_map);
    
    return;
}

void Lrz::OutPut()
{
    R2PathPlan root_plan;

    for(int i = 1;i <= 4;++i)
    {
        for(int j = 1;j <= 3;++j)
        {
            kfs_map_num[ChangeToNum(j,i)] = kfs_map[j][i];

            if(kfs_map[j][i] == 2 && i == 1)
            {
                root_plan.FirstR2KFS_flag = true;
            }
        }
    }

    if(camp_flag)
    {
        root_plan.nowPosX = 3;
        root_plan.endPosX = 1;
    }
    else 
    {
        root_plan.nowPosX = 1;
        root_plan.endPosX = 3;
    }
    
	for(int i = 1; i <= 4; ++i)
    {
		for(int j = 1; j <= 3; ++j)
        {
			printf("%d ",kfs_map[j][i]);
		}
	}	

    FindPath(root_plan, kfs_map);

    for(int i = 0; i <= 4; ++i)
    {
        plannerNode->logger()->info("ACTION {}", R2.Action[i]);
    }

}

void Lrz::InPut(int i11, int i12, int i13, int i14, int i21, int i22, int i23, int i24, int i31, int i32, int i33, int i34) // 0无,1R1,2R2,3假
{
    this->kfs_map[1][1] = i11;
    this->kfs_map[1][2] = i12;
    this->kfs_map[1][3] = i13;
    this->kfs_map[1][4] = i14;
    this->kfs_map[2][1] = i21;
    this->kfs_map[2][2] = i22;
    this->kfs_map[2][3] = i23;
    this->kfs_map[2][4] = i24;
    this->kfs_map[3][1] = i31;
    this->kfs_map[3][2] = i32;
    this->kfs_map[3][3] = i33;
    this->kfs_map[3][4] = i34;
}

void Lrz::InPutZtx(int i11, int i12, int i13, int i14, int i21, int i22, int i23, int i24, int i31, int i32, int i33, int i34) // 0无,1R1,2R2,3假
{
    this->kfs_map_ztx[0] = i11;
    this->kfs_map_ztx[1] = i12;
    this->kfs_map_ztx[2] = i13;
    this->kfs_map_ztx[3] = i14;
    this->kfs_map_ztx[4] = i21;
    this->kfs_map_ztx[5] = i22;
    this->kfs_map_ztx[6] = i23;
    this->kfs_map_ztx[7] = i24;
    this->kfs_map_ztx[8] = i31;
    this->kfs_map_ztx[9] = i32;
    this->kfs_map_ztx[10] = i33;
    this->kfs_map_ztx[11] = i34;
}

void Lrz::ReHandleData()
{
    // plannerNode->zhuData.way = 1;
    if (plannerNode->grid.data[0][2] != 0)
    {
        this->R2.FirsrR2KFS = 1;
    }
    else if (plannerNode->grid.data[0][1] != 0)
    {
        this->R2.FirsrR2KFS = 2;
    }
    else if (plannerNode->grid.data[0][0] != 0)
    {
        this->R2.FirsrR2KFS = 4;
    }

    if (plannerNode->zhuData.way == 1)
    {
        this->R2.way_num = 1;

        if (plannerNode->grid.data[1][2] != 0)
        {
            this->R2.Action[1] = 1;
        }
        if (plannerNode->grid.data[2][2] != 0)
        {
            this->R2.Action[2] = 1;
        }
        if (plannerNode->grid.data[3][2] != 0)
        {
            this->R2.Action[3] = 1;
        }

        if (plannerNode->grid.data[1][1] != 0)
        {
            if (this->R2.Action[2] == 1)
            {
                this->R2.Action[2] = 3;
            }
            else
            {
                this->R2.Action[2] = 2;
            } 
        }
        if (plannerNode->grid.data[2][1] != 0)
        {
            if (this->R2.Action[3] == 1)
            {
                this->R2.Action[3] = 3;
            }
            else
            {
                this->R2.Action[3] = 2;
            }
        }
        if (plannerNode->grid.data[3][1] != 0)
        {
            this->R2.Action[4] = 2;
        }
    }
    else if (plannerNode->zhuData.way == 2)
    {
        this->R2.way_num = 2;

        if (plannerNode->grid.data[1][1] != 0)
        {
            this->R2.Action[1] = 1;
        }
        if (plannerNode->grid.data[2][1] != 0)
        {
            this->R2.Action[2] = 1;
        }
        if (plannerNode->grid.data[3][1] != 0)
        {
            this->R2.Action[3] = 1;
        }

        if (plannerNode->grid.data[1][0] != 0)
        {
            if (R2.Action[2] == 1)
            {
                this->R2.Action[2] = 3;
            }
            else
            {
                this->R2.Action[2] = 2;
            }
        }
        if (plannerNode->grid.data[2][0] != 0)
        {
            if (R2.Action[3] == 1)
            {
                this->R2.Action[3] = 3;
            }
            else
            {
                this->R2.Action[3] = 2;
            }
        }
        if (plannerNode->grid.data[3][0] != 0)
        {
            this->R2.Action[4] = 2;
        }

        if (plannerNode->grid.data[1][2] != 0)
        {
            if (R2.Action[2] == 1)
            {
                this->R2.Action[2] = 5;
            }
            else
            {
                this->R2.Action[2] = 4;
            }
        }
        if (plannerNode->grid.data[2][2] != 0)
        {
            if (R2.Action[3] == 1)
            {
                this->R2.Action[3] = 5;
            }
            else
            {
                this->R2.Action[3] = 4;
            }
        }
        if (plannerNode->grid.data[3][2] != 0)
        {
            this->R2.Action[4] = 4;
        }
    }
    else if (plannerNode->zhuData.way == 3)
    {
        this->R2.way_num = 3;

        if (plannerNode->grid.data[1][0] != 0)
        {
            this->R2.Action[1] = 1;
        }
        if (plannerNode->grid.data[2][0] != 0)
        {
            this->R2.Action[2] = 1;
        }
        if (plannerNode->grid.data[3][0] != 0)
        {
            this->R2.Action[3] = 1;
        }

        if (plannerNode->grid.data[1][1] != 0)
        {
            if (this->R2.Action[2] == 1)
            {
                this->R2.Action[2] = 5;
            }
            else
            {
                this->R2.Action[2] = 4;
            } 
        }
        if (plannerNode->grid.data[2][1] != 0)
        {
            if (this->R2.Action[3] == 1)
            {
                this->R2.Action[3] = 5;
            }
            else
            {
                this->R2.Action[3] = 4;
            }
        }
        if (plannerNode->grid.data[3][1] != 0)
        {
            this->R2.Action[4] = 4;
        }
    }
}

void Lrz::HandleData()
{
    if ((this->R2.FirsrR2KFS >> 0) & 0x01)
    {
        this->trunk_first[0] = 1;
        
        if (this->R2.way_num == 1)
        {
            this->trunk_flag[0][0] = 2;
        }
        else
        {
            if (!((this->R2.FirsrR2KFS >> 1) & 0x01) && !((this->R2.FirsrR2KFS >> 2) & 0x01))
            {
                this->trunk_flag[0][0] = 2;
            }
            else
            {
                this->trunk_flag[0][0] = 2;
            }

            if ((this->R2.FirsrR2KFS >> 1) & 0x01)
            {
                this->trunk_flag[0][1] = 2;
                
            }
            else if ((this->R2.FirsrR2KFS >> 2) & 0x01)
            {
                this->trunk_flag[0][2] = 2;
            }
        }
        this->trunk_num++;
    }

    if ((this->R2.FirsrR2KFS >> 1) & 0x01)
    {
        this->trunk_first[1] = 1;
        if (this->R2.way_num == 2)
        {
            this->trunk_flag[0][1] = 2;
        }
        else
        {
            if (!((this->R2.FirsrR2KFS >> 0) & 0x01) && !((this->R2.FirsrR2KFS >> 2) & 0x01))
            {
                this->trunk_flag[0][1] = 2;
            }
            else
            {
                this->trunk_flag[0][1] = 2;
            }

            if ((this->R2.FirsrR2KFS >> 0) & 0x01)
            {
                this->trunk_flag[0][0] = 2;
            }
            else if ((this->R2.FirsrR2KFS >> 2) & 0x01)
            {
                this->trunk_flag[0][2] = 2;
            }
        }
        this->trunk_num++;
    }

    if ((this->R2.FirsrR2KFS >> 2) & 0x01)
    {
        this->trunk_first[2] = 1;
        if (this->R2.way_num == 3)
        {
            this->trunk_flag[0][2] = 2;
        }
        else
        {
            if (!((this->R2.FirsrR2KFS >> 0) & 0x01) && !((this->R2.FirsrR2KFS >> 1) & 0x01))
            {
                this->trunk_flag[0][2] = 2;     
            }
            else
            {
                this->trunk_flag[0][2] = 2;
            }

            if ((this->R2.FirsrR2KFS >> 0) & 0x01)
            {
                this->trunk_flag[0][0] = 2;
            }
            else if ((this->R2.FirsrR2KFS >> 1) & 0x01)
            {
                this->trunk_flag[0][1] = 2;
            }
        }
        this->trunk_num++;
    }

    if ((this->R2.way_num == 1 && this->trunk_first[1] == 1) || (this->R2.way_num == 3 && this->trunk_first[1] == 1))
    {
        this->trunk_leg = 1;
    }

    if (this->R2.way_num == 1)
    {
        if (this->R2.Action[1] & 1)
        {
            this->trunk_flag[1][3] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[1] & 2)
        {
            this->trunk_flag[1][1] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[1] & 4)//最右列不可能向右
        {
            this->trunk_flag[1][2] = 1;
            this->trunk_num++;
        }

        if (this->R2.Action[2] & 1)
        {
            this->trunk_flag[2][3] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[2] & 2)
        {
            this->trunk_flag[2][1] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[2] & 4)//最右列不可能向右
        {
            this->trunk_flag[2][2] = 2;
            this->trunk_num++;
        }

        if (this->R2.Action[3] & 1)
        {
            this->trunk_flag[3][3] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[3] & 2)
        {
            this->trunk_flag[3][1] = 1;
            this->trunk_num++;
        }
        if (this->R2.Action[3] & 4)//最右列不可能向右
        {
            this->trunk_flag[3][2] = 1;
            this->trunk_num++;
        }

        if (this->R2.Action[4] & 1)//最后排不可能向前
        {
            this->trunk_flag[4][3] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[4] & 2)
        {
            this->trunk_flag[4][1] = 1;
            this->trunk_num++;
        }
        if (this->R2.Action[4] & 4)//最右列不可能向右
        {
            this->trunk_flag[4][2] = 1;
            this->trunk_num++;
        }
    }

    if (this->R2.way_num == 2)
    {
        if (this->R2.Action[1] & 1)
        {
            this->trunk_flag[1][3] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[1] & 2)
        {
            this->trunk_flag[1][1] = 1;
            this->trunk_num++;
        }
        if (this->R2.Action[1] & 4)
        {
            this->trunk_flag[1][2] = 1;
            this->trunk_num++;
        }

        if (this->R2.Action[2] & 1)
        {
            this->trunk_flag[2][3] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[2] & 2)
        {
            this->trunk_flag[2][1] = 1;
            this->trunk_num++;
        }
        if (this->R2.Action[2] & 4)
        {
            this->trunk_flag[2][2] = 4;
            this->trunk_num++;
        }

        if (this->R2.Action[3] & 1)
        {
            this->trunk_flag[3][3] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[3] & 2)
        {
            this->trunk_flag[3][1] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[3] & 4)
        {
            this->trunk_flag[3][2] = 4;
            this->trunk_num++;
        }

        if (this->R2.Action[4] & 1)//最后排不可能向前
        {
            this->trunk_flag[4][3] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[4] & 2)
        {
            this->trunk_flag[4][1] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[4] & 4)
        {
            this->trunk_flag[4][2] = 4;
            this->trunk_num++;
        }
    }

    if (this->R2.way_num == 3)
    {
        if (this->R2.Action[1] & 1)
        {
            this->trunk_flag[1][3] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[1] & 2)//最左列不可能向左
        {
            this->trunk_flag[1][1] = 1;
            this->trunk_num++;
        }
        if (this->R2.Action[1] & 4)
        {
            this->trunk_flag[1][2] = 4;
            this->trunk_num++;
        }

        if (this->R2.Action[2] & 1)
        {
            this->trunk_flag[2][3] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[2] & 2)//最左列不可能向左
        {
            this->trunk_flag[2][1] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[2] & 4)
        {
            this->trunk_flag[2][2] = 4;
            this->trunk_num++;
        }

        if (this->R2.Action[3] & 1)
        {
            this->trunk_flag[3][3] = 4;
            this->trunk_num++;
        }
        if (this->R2.Action[3] & 2)//最左列不可能向左
        {
            this->trunk_flag[3][1] = 1;
            this->trunk_num++;
        }
        if (this->R2.Action[3] & 4)
        {
            this->trunk_flag[3][2] = 1;
            this->trunk_num++;
        }

        if (this->R2.Action[4] & 1)//最后排不可能向前
        {
            this->trunk_flag[4][3] = 2;
            this->trunk_num++;
        }
        if (this->R2.Action[4] & 2)//最左列不可能向左
        {
            this->trunk_flag[4][1] = 1;
            this->trunk_num++;
        }
        if (this->R2.Action[4] & 4)
        {
            this->trunk_flag[4][2] = 1;
            this->trunk_num++;
        }
    }
}

std::vector<std::vector<int>> get_R2_map(int map[12])
{
    std::vector<std::vector<int>> r2_map(6, std::vector<int>(5, 0));
    r2_map[4][3] = map[0];
    r2_map[3][3] = map[1];
    r2_map[2][3] = map[2];
    r2_map[1][3] = map[3];
    
    r2_map[4][2] = map[4];
    r2_map[3][2] = map[5];
    r2_map[2][2] = map[6];
    r2_map[1][2] = map[7];

    r2_map[4][1] = map[8];
    r2_map[3][1] = map[9];
    r2_map[2][1] = map[10];
    r2_map[1][1] = map[11];
    return r2_map;
}

void Lrz::ZtxOutPut()
{
    FindPathResult result;

    std::vector<int> blockTyepes;

    auto r1_map = get_R2_map(this->kfs_map_ztx);

    extractBlockTypes(r1_map, blockTyepes);
    
    result = findPath(blockTyepes);
    
    this->R2.FirsrR2KFS = convertR2Result(result);
    
    std::vector<int> your_x = get_R2_result(result.r2ColumnCount, result.r2CollectBlocks, blockTyepes);

    size_t copyLen = std::min(static_cast<size_t>(5), your_x.size());
    
    std::copy(your_x.begin(), your_x.begin() + copyLen, this->R2.Action);
}

int Lrz::SearchTrunk()
{
    static int current_row = 0;
    static int current_col = 0;

    for (int i = current_row; i < 5; i++) 
    {
        int j_start = (i == current_row) ? current_col : 0;
        
        for (int j = j_start; j < 4; j++) 
        {
            if (this->trunk_flag[i][j] != 0) 
            {
                int ret = this->trunk_flag[i][j];

                if (j + 1 < 4) 
                {
                    current_col = j + 1;
                    current_row = i;
                } 
                else 
                {
                    current_col = 0;
                    current_row = i + 1;
                
                }

                return ret;
            }
        }
    }

    current_row = 0;
    current_col = 0;

    return 0;
}