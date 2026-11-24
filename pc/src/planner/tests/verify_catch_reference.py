"""三种 Catch 的原源码 oracle；检查时长迁移数据并可编译/执行数值差分。"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    repo=Path(__file__).resolve().parents[4]
    parser.add_argument('--reference',type=Path,default=repo/'2027_RC_vollyball_jieqiu/User_Ctrl/Src/zlr_math.c')
    parser.add_argument('--compiler')
    parser.add_argument('--compile-only',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    source=args.reference.read_text(encoding='utf-8-sig')
    header=(args.reference.parent.parent/'Inc/zlr_math.h').read_text(encoding='utf-8-sig')
    structs='\n'.join(re.findall(r'typedef struct\s*\{.*?\}\s*\w+;',header,re.S))
    assert 'MotionProfile;' in structs and 'ThetaProfile;' in structs
    source=re.sub(r'^#include[^\n]*','',source,flags=re.M)
    fixture='''#include <math.h>
#include <cassert>
#include <cstdio>
#include <limits>
#include "planner/catch/height_profile.hpp"
#include "planner/catch/angle_five2.hpp"
'''
    for macro in ('T','FIVE1','FIVE2'):
        fixture+=f'#define {macro}\nnamespace oracle_{macro} {{\n'+structs+'\n'+source+f'\n}}\n#undef {macro}\n#undef PI\n#undef PI_3\n'
    fields={'acceleration_seconds':'t_acc','constant_seconds':'t_const','deceleration_seconds':'t_dec','total_seconds':'t_total'}
    for stem,member in [('catch/height_profile','profile'),('catch/angle_five2','tp')]:
        actual=(root/'src'/f'{stem}.cpp').read_text(encoding='utf-8')
        for public,old in fields.items():
            if member=='profile' and old=='t_total':
                expected='timing.acceleration_seconds+timing.constant_seconds+timing.deceleration_seconds'
            else:
                expected=re.search(r'\b'+member+r'\.'+old+r'\s*=\s*([^;]+);',source)[1]
                expected=expected.replace(member+'.', 'timing.')
                for new,previous in fields.items(): expected=expected.replace('timing.'+previous,'timing.'+new)
            found=re.search(r'timing\.'+public+r'\s*=\s*([^;]+);',actual)[1]
            compact=lambda s: re.sub(r'\s+','',s)
            assert compact(found)==compact(expected),(stem,public,found,expected)
        assert not re.search(r'Get(?:Acceleration|Constant|Total)Duration\(',actual)
    fixture+='int main() { unsigned count=0;\n'
    for macro,constructor in [('T','planner::CatchHeightPlanner actual(planner::CatchHeightMode::Trapezoidal);'),('FIVE1','planner::CatchHeightPlanner actual(planner::CatchHeightMode::Quintic);'),('FIVE2','planner::CatchFive2Planner actual;')]:
        member='tp' if macro=='FIVE2' else 'profile'
        fixture+='{\n'+constructor+f'''
oracle_{macro}::init_theta_profile();
const auto& expected=oracle_{macro}::{member};
assert(actual.timing.acceleration_seconds==expected.t_acc);
assert(actual.timing.constant_seconds==expected.t_const);
assert(actual.timing.deceleration_seconds==expected.t_dec);
assert(actual.timing.total_seconds==expected.t_acc+expected.t_const+expected.t_dec);
auto check=[&](double t) {{
assert(actual.GetMotorRevsThetaPlan(t)==oracle_{macro}::get_motor_revs_theta_plan(t));
assert(actual.GetMotorVelAtTime(t)==oracle_{macro}::get_motor_vel_at_time(t));
++count;
}};
const double boundaries[]={{0.0,expected.t_acc,expected.t_acc+expected.t_const,actual.timing.total_seconds}};
for(double t:boundaries) {{
check(nextafter(t,-std::numeric_limits<double>::infinity()));
check(t);
check(nextafter(t,std::numeric_limits<double>::infinity()));
}}
for(int i=-10;i<10000;++i) check(i*0.0001);
actual.InitThetaProfile();
assert(actual.timing.acceleration_seconds==expected.t_acc);
}}
'''
    fixture+='std::printf("PASS: %u Catch position/velocity samples and timing checks\\n",count);\n}\n'
    print('PASS: both Catch timing structures match original duration expressions; six getters removed')
    if not args.compiler:
        print('Numerical differential execution NOT RUN; supply native --compiler')
        return
    with tempfile.TemporaryDirectory(prefix='catch-reference-') as directory:
        path=Path(directory)/'oracle.cpp';path.write_text(fixture,encoding='utf-8')
        command=[args.compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'include'),str(root/'src/catch/height_profile.cpp'),str(root/'src/catch/angle_five2.cpp'),str(path)]
        if args.compile_only:
            subprocess.run(command+['-fsyntax-only'],check=True)
            print('PASS: all three Catch original-source differential fixtures compile; numerical execution NOT RUN')
        else:
            binary=Path(directory)/'verify.exe'
            subprocess.run(command+['-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

if __name__=='__main__':
    main()
