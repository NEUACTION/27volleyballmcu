"""核对 WAY1 七模式参数表，并保留原函数逐周期 oracle 差分夹具。"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile
from verify_row_reference import clean, function_body

CASES = [
    ('HIGH','High',2173,2232,'STARTHIGH','1.f','0.f'),
    ('HIGHDOWN','HighDown',2237,2296,'STARTHIGHDOWN','1.f','0.f'),
    ('HIGHDOWN117','HighDown117',2301,2360,'STARTHIGHDOWN','1.f','0.009f'),
    ('HIGHDOWN1110','HighDown1110',2365,2421,'STARTHIGHDOWN','1.f','0.009f'),
    ('LOW118A','Low118A',2426,2484,'STARTLOW118A','-1.f','0.f'),
    ('LOW118B','Low118B',2489,2548,'STARTLOW118B','-1.f','0.f'),
    ('LOW119','Low119',2553,2609,'STARTLOW119','-1.f','0.f'),
]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=Path(r'D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c'))
    parser.add_argument('--compiler')
    parser.add_argument('--compile-only',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    lines=args.reference.read_text(encoding='utf-8-sig').splitlines()
    actual=(root/'src/serve/arm_legacy.cpp').read_text(encoding='utf-8')
    rows=re.findall(r'\{(HIGH\w*|LOW\w*),([^{}]+)\}',actual)
    assert len(rows)==7
    oracle='''#include "planner/serve/arm_legacy.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
namespace oracle {
using namespace planner::legacy_way1;
struct Gains { float pos_kp=-1, pos_kd=-1; } armTar1, armTar2;
int hfdcan3, disable_count=0, last_stage=0;
float last_vel=0;
enum { PTP_MODE, ARM1, ARM2, DISABLE };
void SetStateAndPosCtrl(int*,int,int,int,float) { ++disable_count; }
'''
    for (mode,suffix,first,last,start,direction,offset), (key,data) in zip(CASES,rows):
        original='\n'.join(lines[first-1:last])
        effective=re.sub(r'//[^\n]*|/\*.*?\*/','',original,flags=re.S)
        expected=[re.search(r'const float '+field+r'\s*=\s*([\d.]+)f;',effective)[1]
                  for field in ('targetVelocity','s1','s2')]
        cells=data.split(',')
        assert key==mode and len(cells)==6
        assert [float(c.strip()[:-1]) for c in cells[:3]]==list(map(float,expected)),mode
        assert list(map(str.strip,cells[3:]))==[start,direction,offset],mode
        # 只在原 return 前读取 static 速度/阶段供断言，不改原计算或状态转移。
        instrumented=original.replace('return referencePosition;',
            'last_vel=vel; last_stage=stage; return referencePosition;')
        oracle+=instrumented+'\n'
    for name,first,last in [('EvaluateFive1QuinticState',2827,2867),('PlanFive1Trajectory',2872,2948),('EvaluateFive1Trajectory',2950,2992)]:
        original='\n'.join(lines[first-1:last])
        assert clean(function_body(original,name))==clean(function_body(actual,name)),name
    oracle+='}\nint main() {\nplanner::legacy_way1::RowVariants actual;\nunsigned count=0;\n'
    # 交错各模式，验证各自累计状态以及共享增益的真实调用次序。
    oracle+='for(int frame=0; frame<150; ++frame) {\n'
    for mode,suffix,_,_,start,direction,offset in CASES:
        oracle+=f'''{{
const int before=oracle::disable_count;
const float position=oracle::GetRowPosition{suffix}Trajectory();
const auto sample=actual.GetRowPositionReference(planner::legacy_way1::{mode});
assert(sample.referencePosition==position);
assert(sample.vel==oracle::last_vel && sample.stage==oracle::last_stage);
assert(sample.arm1.positionRevolutions==planner::legacy_way1::{start}+({direction})*position);
assert(sample.arm2.positionRevolutions==-sample.arm1.positionRevolutions-({offset}));
// planner 输出驱动 PosCtrl 内部乘之前的原值：kp 乘 60000、kd 乘 1000 后再与原源码比较。
auto near=[&](float a,float b){{return std::fabs(a-b)<=1e-5f*std::fabs(b)+1e-4f;}};
assert(near(sample.arm1.pos_kp*60000.0f,oracle::armTar1.pos_kp) && near(sample.arm1.pos_kd*1000.0f,oracle::armTar1.pos_kd));
assert(near(sample.arm2.pos_kp*60000.0f,oracle::armTar2.pos_kp) && near(sample.arm2.pos_kd*1000.0f,oracle::armTar2.pos_kd));
assert(sample.positionIntent && sample.disableIntent==(oracle::disable_count-before==2));
++count;
}}
'''
    oracle+='''}
assert(oracle::disable_count==14);
assert(!actual.GetRowPositionReference(-999).positionIntent);
actual.reset();
planner::legacy_way1::RowVariants independent;
assert(actual.GetRowPositionHighTrajectory().referencePosition==independent.GetRowPositionHighTrajectory().referencePosition);
std::printf("PASS: %u interleaved WAY1 general samples\\n",count);
}
'''
    print('PASS: all 7 WAY1 table rows and 3 FIVE1 math bodies match reference')
    if not args.compiler:
        print('Numerical differential execution NOT RUN; supply native --compiler')
        return
    with tempfile.TemporaryDirectory(prefix='row-variants-reference-') as directory:
        fixture=Path(directory)/'oracle.cpp';fixture.write_text(oracle,encoding='utf-8')
        command=[args.compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'include'),str(root/'src/serve/arm_legacy.cpp'),str(fixture)]
        if args.compile_only:
            subprocess.run(command+['-fsyntax-only'],check=True)
            print('PASS: WAY1 core and original-function oracle compile; numerical execution NOT RUN')
        else:
            binary=Path(directory)/'verify.exe'
            subprocess.run(command+['-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

if __name__=='__main__':
    main()
