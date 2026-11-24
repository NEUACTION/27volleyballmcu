"""只读验证row数学核心来源，选择本机C++编译器时再运行原函数差分。"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def clean(text):
    """去除排版和注释，保留有效语句的常量、运算与执行顺序。"""
    text = re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.S)
    return re.sub(r'\s+', '', text)


def function_body(text, name):
    """按花括号嵌套提取命名函数体。"""
    start = text.index('{', text.index(name + '('))
    depth, cursor = 1, start + 1
    while depth:
        depth += (text[cursor] == '{') - (text[cursor] == '}')
        cursor += 1
    return text[start:cursor]


def fold_constant_products(text):
    """把只由数字字面量和乘除号组成的表达式折叠为一个float，避免写法不同造成误报。"""
    pattern = re.compile(r'(?<![\w.])(?:[0-9]+(?:\.[0-9]*)?f|f?[0-9]*\.[0-9]+f|\.[0-9]+f)'
                         r'(?:\s*[*/]\s*(?:[0-9]+(?:\.[0-9]*)?f|f?[0-9]*\.[0-9]+f|\.[0-9]+f))+')

    def fold(match):
        expr = match.group().replace('f', '')
        value = float(eval(expr, {'__builtins__': {}}, {}))  # 仅折叠已验证为纯数字的乘除
        return f'{value!r}f'  # 保留f后缀，只比较数值本身

    return pattern.sub(fold, text)


def main():
    """对照FIVE全部语句及一般轨迹的状态/积分/增益，编译选项不写工程。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=Path(
        r'D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c'))
    parser.add_argument('--compiler')
    parser.add_argument('--compile-only', action='store_true', help='只检查核心及临时oracle语法，不运行交叉编译产物')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    lines = args.reference.read_text(encoding='utf-8-sig').splitlines()
    core = '\n'.join(lines[766:939])
    general = '\n'.join(lines[943:1006])
    migrated = (root / 'src/serve/arm_trajectory.cpp').read_text(encoding='utf-8')
    for name in ('EvaluateQuinticTrajectory', 'PlanRowPositionTrajectory', 'GetRowPositionReference'):
        assert clean(function_body(core, name)) == clean(function_body(migrated, name)), name
    original_general = re.sub(r'//[^\n]*|/\*.*?\*/', '', function_body(general, 'GetRowPositionReference'), flags=re.S)
    original_general = re.sub(r'static (?:float|int) \w+\s*=.*?;', '', original_general, flags=re.S)
    for old, new in (
        ('referencePosition', 'reference_position_'), ('referenceVelocity', 'reference_velocity_'),
        ('trajectoryStage', 'trajectory_stage_'),
        ('ROW_MOTION_STAGE_ACCELERATING', 'RowMotionStage::Accelerating'),
        ('ROW_MOTION_STAGE_CONSTANT_SPEED', 'RowMotionStage::ConstantSpeed'),
        ('ROW_MOTION_STAGE_STOPPING', 'RowMotionStage::Stopping'),
        ('ROW_MOTION_STAGE_COMPLETE', 'RowMotionStage::Complete'),
        ('armTar1.pos_kp', 'arm1_gains_.kp'), ('armTar1.pos_kd', 'arm1_gains_.kd'),
        ('armTar2.pos_kp', 'arm2_gains_.kp'), ('armTar2.pos_kd', 'arm2_gains_.kd')):
        original_general = original_general.replace(old, new)
    original_general = re.sub(r'SetStateAndPosCtrl\([^;]+;\s*SetStateAndPosCtrl\([^;]+;',
                              'disable_requested = true;', original_general)
    original_general = original_general.replace('return reference_position_;', '')
    actual_general = re.sub(r'//[^\n]*|/\*.*?\*/', '', function_body(migrated, 'GeneralRowPlanner::Step'), flags=re.S)
    actual_general = actual_general.replace('bool disable_requested = false;', '')
    actual_general = re.sub(r'(arm[12]_gains_) = \{([^,]+), ([^}]+)\};',
                            r'\1.kp = \2; \1.kd = \3;', actual_general)
    actual_general = re.sub(r'return\s*\{[^}]+\}\s*;', '', actual_general)
    # planner 现在输出驱动内部乘之前的原始增益；补回 PosCtrl 的倍率才能与原源码比较。
    actual_general = re.sub(r'(arm[12]_gains_\.kp\s*=\s*)([0-9.]+f)',
                            r'\1\2*1000.f*60.f', actual_general)
    actual_general = re.sub(r'(arm[12]_gains_\.kd\s*=\s*)([0-9.]+f)',
                            r'\1\2*1000.f', actual_general)
    # 两边先把纯数字乘式折叠成同一个float值，再逐字符比较其余语句。
    # 这样改写为例如1800.0f而非1.8f*1000.f时不会误报，但真正的常量或运算差异仍会报错。
    original_norm = fold_constant_products(original_general)
    actual_norm = fold_constant_products(actual_general)
    assert clean(original_norm) == clean(actual_norm), 'general state/integration/gain statements'
    print('PASS: FIVE 3 function bodies and general trajectory statements match reference')
    if not args.compiler:
        print('C++ runtime differential test NOT RUN: pass --compiler <g++ or clang++>')
        return
    oracle = '''#include "planner/serve/arm_trajectory.hpp"
#include <cmath>
#include <cstddef>
#include <cassert>
#include <cstdio>
#include <limits>
namespace oracle {
using namespace planner;
using std::isfinite;
'''
    oracle += core + '''
struct Gains { float pos_kp; float pos_kd; } armTar1, armTar2;
int hfdcan3;
int disable_count = 0;
enum { PTP_MODE, ARM1, ARM2, DISABLE };
enum { ROW_MOTION_STAGE_ACCELERATING, ROW_MOTION_STAGE_CONSTANT_SPEED,
       ROW_MOTION_STAGE_STOPPING, ROW_MOTION_STAGE_COMPLETE };
void SetStateAndPosCtrl(int*, int, int, int, float) { ++disable_count; }
'''
    oracle += general + '\n}\n'
    oracle += '''
int main()
{
  planner::GeneralRowPlanner general;
  float first_position = 0.0f;
  int disabled = 0;
  for (int i = 0; i < 150; ++i) {
    const float expected = oracle::GetRowPositionReference();
    const auto actual = general.Step();
    if (i == 0) first_position = actual.position_turns;
    assert(actual.position_turns == expected);
    assert(actual.arm1_gains.kp == oracle::armTar1.pos_kp);
    assert(actual.arm1_gains.kd == oracle::armTar1.pos_kd);
    assert(actual.arm2_gains.kp == oracle::armTar2.pos_kp);
    assert(actual.arm2_gains.kd == oracle::armTar2.pos_kd);
    if (actual.disable_requested) ++disabled;
  }
  assert(disabled == 1 && oracle::disable_count == 2);
  general.Reset();
  assert(general.Step().position_turns == first_position);
  unsigned count = 0;
  for (float sign : {1.0f, -1.0f}) {
    planner::PosRowPlan_t expected{0, sign, 2*sign, 2*sign, 5, 0, 0, false};
    auto actual = expected;
    assert(oracle::PlanRowPositionTrajectory(&expected, 0, 2, 100, 0.035f));
    assert(planner::PlanRowPositionTrajectory(&actual, 0, 2, 100, 0.035f));
    assert(expected.startTimeSeconds == actual.startTimeSeconds);
    assert(expected.endTimeSeconds == actual.endTimeSeconds);
    for (int i = 0; i <= 1400; ++i) {
      float expected_pos = 123, expected_vel = 456, actual_pos = 123, actual_vel = 456;
      const float time = i * 0.005f;
      auto a = oracle::GetRowPositionReference(&expected, time, &expected_pos, &expected_vel);
      auto b = planner::GetRowPositionReference(&actual, time, &actual_pos, &actual_vel);
      assert(a == b && expected_pos == actual_pos && expected_vel == actual_vel);
      ++count;
    }
    float position = 123, velocity = 456;
    assert(planner::GetRowPositionReference(nullptr, 0, &position, &velocity) == planner::POS_ROW_ERROR);
    assert(position == 123 && velocity == 456);
    actual.hitAngularVelocityRadPerSecond = 0;
    assert(!planner::PlanRowPositionTrajectory(&actual, 0, 2, 100, 0.035f));
    assert(!actual.valid);
  }
  const auto mapped = planner::MapLogicalRowTurns(2, {11,12}, {21,22});
  assert(mapped.arm1_position_turns == 2 && mapped.arm2_position_turns == -2.0f-0.009f);
  assert(mapped.arm1_gains.kp == 11 && mapped.arm1_gains.kd == 12);
  assert(mapped.arm2_gains.kp == 21 && mapped.arm2_gains.kd == 22);
  const auto offset = planner::MapGeneralRowTurns(2, 0.5f, {11,12}, {21,22});
  assert(offset.arm1_position_turns == -0.5f+2.0f);
  assert(offset.arm2_position_turns == 0.5f-2.0f-0.009f);
  std::printf("PASS: 150 general samples, %u FIVE samples, disable/reset/invalid/mapping checks\\n", count);
}
'''
    # 范围for的花括号列表依赖initializer_list头，显式包含以避免依赖间接头。
    oracle = '#include <initializer_list>\n' + oracle
    with tempfile.TemporaryDirectory(prefix='row-reference-') as directory:
        fixture = Path(directory) / 'oracle.cpp'
        binary = Path(directory) / 'verify.exe'
        fixture.write_text(oracle, encoding='utf-8')
        command = [args.compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                   '-I', str(root / 'include'), str(root / 'src/serve/arm_trajectory.cpp'), str(fixture)]
        if args.compile_only:
            subprocess.run(command + ['-fsyntax-only'], check=True)
            print('PASS: core and runtime differential fixture compile; numerical execution NOT RUN')
        else:
            subprocess.run(command + ['-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
