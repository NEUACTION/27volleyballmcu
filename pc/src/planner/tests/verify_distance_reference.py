"""只读核对实际参考语句；可选编译临时 oracle 做速度/PID 差分，不写工程文件。"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

# 条件分支、函数名、原始首末行；遇参考行号漂移时主动失败，要求人工重新核对。
CASES = [
    ('Way0High', 'GetPlateVelocityHigh', 1186, 1285),
    ('Way0Try', 'GetPlateVelocityTry', 1290, 1397),
    ('Way0Low', 'GetPlateVelocityLow', 1402, 1464),
    ('Way1High', 'GetPlateVelocityHigh', 3367, 3438),
    ('Way1HighDown', 'GetPlateVelocityHighDown', 3443, 3513),
    ('Way1HighDown117', 'GetPlateVelocityHighDown117', 3518, 3591),
    ('Way1HighDown1110', 'GetPlateVelocityHighDown1110', 3596, 3663),
    ('Way1Low118A', 'GetPlateVelocityLow118A', 3668, 3729),
    ('Way1Low118B', 'GetPlateVelocityLow118B', 3734, 3797),
    ('Way1Low119', 'GetPlateVelocityLow119', 3802, 3862),
    ('Way1Low', 'GetPlateVelocityLow', 3867, 3926),
]
# 这些源函数没有末尾返回；有限输入都有已迁移的返回分支。
MISSING_RETURN_PROFILES = {
    'Way0High', 'Way0Try', 'Way0Low', 'Way1Low118A', 'Way1Low118B', 'Way1Low119'
}


def uncomment(text):
    """移除注释用于语句比对，保留全部有效常量、运算及分支。"""
    return re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.S)


def main():
    """先完整对照参数表与增益规则，指定编译器时构建原函数逐点差分。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=Path(
        r'D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c'))
    parser.add_argument('--compiler', help='支持 -std=c++17 的 g++ 或 clang++ 路径')
    parser.add_argument('--compile-only', action='store_true', help='只检查核心及oracle夹具语法，不运行交叉产物')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    lines = args.reference.read_text(encoding='utf-8-sig').splitlines()
    migrated = uncomment((root / 'src/serve/plate_distance.cpp').read_text(encoding='utf-8'))
    oracle = '#include <cmath>\n#include <cassert>\n#include <limits>\n#include <cstdio>\n#include "planner/serve/plate_distance.hpp"\n#include "planner/slide/position.hpp"\n'
    # 表按公开枚举顺序排列，逐行检查全部速度/加减速/增益规则数据。
    enum_text = (root / 'include/planner/serve/plate_distance.hpp').read_text(encoding='utf-8')
    enum_body = enum_text.split('enum class ServeDistanceProfile', 1)[1].split('};', 1)[0]
    modes = re.findall(r'\b(Way[01]\w+)\s*(?:,|$)', uncomment(enum_body), re.M)
    rows = re.findall(r'\{([0-9.]+f),\s*([0-9.]+f),\s*([0-9.]+f),\s*GainRule::(\w+)\}', migrated)
    assert len(rows) == len(modes) == len(CASES)
    table = dict(zip(modes, rows))
    rules = {'Way0High':'High0', 'Way0Try':'Try', 'Way1Low':'Low1'}
    gain_sequences = {
        'High0': [('kp','1.8'),('ki','0.4'),('kp','1.8'),('ki','0.4'),('kp','10.0'),('ki','1.0'),('kp','0.8'),('ki','0.2'),('kp','1.65'),('ki','0.4'),('kp','2.0'),('ki','0.0'),('kp','0.6'),('ki','0.4')],
        'High1': [('kp','1.8'),('ki','0.4'),('kp','1.6'),('ki','0.4'),('kp','0.9'),('ki','0.2'),('kp','1.65'),('ki','0.4'),('kp','2.0'),('ki','0.2'),('kp','0.6'),('ki','0.4')],
        'Low': [('kp','1.8'),('ki','0.5'),('kp','1.8'),('ki','0.2'),('kp','1.8'),('kp','2.5'),('ki','0.0'),('ki','1.0')],
        'Low1': [('kp','2.5'),('kp','1.8'),('kp','2.5'),('ki','0.0'),('ki','1.0')],
    }
    gain_sequences['Try'] = gain_sequences['High0'][:8] + [('kp','0.8'),('ki','0.2')] + gain_sequences['High0'][10:]
    for mode, name, first, last in CASES:
        original = uncomment('\n'.join(lines[first-1:last]))
        assert original.startswith(f'float {name}(float travelDistanceRevolutions)'), mode
        values = tuple(re.search(r'float '+field+r'\s*=\s*([0-9.]+f);', original)[1]
                       for field in ('targetVelocity','acceleration','deceleration'))
        rule = rules.get(mode, 'Low' if 'Low' in mode else 'High1')
        assert table[mode] == (*values, rule), (mode, table[mode], values, rule)
        assignments = re.findall(r'plateTar\.vel_k([pi])\s*=\s*([0-9.]+)f\s*\*\s*1000\.f', original)
        assert [('k'+field,value) for field,value in assignments] == gain_sequences[rule], mode
        # oracle 仍直接使用完整原函数，不从新表生成预期值。
        oracle += f'namespace oracle_{mode} {{\nstruct Gains {{ float vel_kp; float vel_ki; }} plateTar;\n'
        if mode in MISSING_RETURN_PROFILES:
            original = original.rsplit('}', 1)[0] + 'return 0.0f;\n}'
        oracle += original + '\n}\n'
    assert 'GetPlateVelocity' not in migrated, '不要保留十一份内部模式实现'
    print('PASS: all 11 table rows match source speed/acceleration/deceleration and complete PID write sequences')
    if not args.compiler:
        print('C++ runtime differential test NOT RUN: pass --compiler <g++ or clang++>')
        return
    oracle += '''
int main()
{
  // 对照阈值及相邻浮点、密集逆序扫描和同模式历史增益，结果逐项相同。
  const float boundaries[] = {-1.0f, 0.0f, 0.6f, 1.0f, 1.2f, 1.4f, 1.5f, 1.6f, 1.7f,
    2.0f, 2.2f, 2.95f, 3.0f, 0.3f*0.3f/(2.0f*1500.0f),
    0.3f*0.3f/(2.0f*900.0f), 0.3f*0.3f/(2.0f*200.0f),
    0.3f*0.3f/(2.0f*400.0f), 30.0f*30.0f/(2.0f*1500.0f),
    10.0f*10.0f/(2.0f*900.0f), 11.2f*11.2f/(2.0f*200.0f),
    32.5f*32.5f/(2.0f*1500.0f), 20.0f*20.0f/(2.0f*400.0f),
    1.7f+10.0f*10.0f/(2.0f*900.0f), 1.7f+11.2f*11.2f/(2.0f*200.0f),
    1.7f+30.0f*30.0f/(2.0f*1500.0f), 1.7f+32.5f*32.5f/(2.0f*1500.0f),
    1.7f+20.0f*20.0f/(2.0f*400.0f),
    35.80f*35.80f/(2.0f*1500.0f), 33.5f*33.5f/(2.0f*1500.0f),
    58.0f*58.0f/(2.0f*1500.0f), 7.6f*7.6f/(2.0f*200.0f),
    9.0f*9.0f/(2.0f*200.0f),
    1.7f+35.80f*35.80f/(2.0f*1500.0f), 1.7f+33.5f*33.5f/(2.0f*1500.0f),
    1.7f+58.0f*58.0f/(2.0f*1500.0f), 1.7f+7.6f*7.6f/(2.0f*200.0f),
    1.7f+9.0f*9.0f/(2.0f*200.0f)};
  unsigned count = 0;
'''
    for mode, name, _, _ in CASES:
        oracle += f'''{{
  planner::DistanceVelocityGains previous{{123.0f, 456.0f}};
  auto check = [&](float distance) {{
    oracle_{mode}::plateTar = {{previous.kp, previous.ki}};
    const float speed = oracle_{mode}::{name}(distance);
    const auto actual = planner::EvaluateServeDistance(planner::ServeDistanceProfile::{mode}, distance, previous);
    assert(actual && actual->velocity_turns_per_second == speed);
    // planner 输出驱动 SetVelKpKi 内部乘之前的原值；乘回 1000 再与原源码的预缩放值比较。
    assert(std::fabs(actual->gains.kp * 1000.0f - oracle_{mode}::plateTar.vel_kp) <=
        1e-5f * std::fabs(oracle_{mode}::plateTar.vel_kp) + 1e-4f);
    assert(std::fabs(actual->gains.ki * 1000.0f - oracle_{mode}::plateTar.vel_ki) <=
        1e-5f * std::fabs(oracle_{mode}::plateTar.vel_ki) + 1e-4f);
    previous = actual->gains;
    ++count;
  }};
  for (float distance : boundaries) {{
    check(std::nextafter(distance, -std::numeric_limits<float>::infinity()));
    check(distance);
    check(std::nextafter(distance, std::numeric_limits<float>::infinity()));
  }}
  for (int i = 40000; i >= -1000; --i) check(i / 10000.0f);
}}
'''
    oracle += '''
  const auto mode = planner::ServeDistanceProfile::Way0Low;
  assert(!planner::EvaluateServeDistance(mode, std::numeric_limits<float>::quiet_NaN(), {1,2}));
  assert(!planner::EvaluateServeDistance(mode, std::numeric_limits<float>::infinity(), {1,2}));
  assert(!planner::EvaluateServeDistance(static_cast<planner::ServeDistanceProfile>(99), 0, {1,2}));
  const auto hold = planner::EvaluateServeDistance(planner::ServeDistanceProfile::Way1Low, 0, {123,456});
  assert(hold && hold->gains.kp == 123 && hold->gains.ki == 456);
  const planner::ServeDistanceProfile low_modes[] = {
    planner::ServeDistanceProfile::Way1Low118A,
    planner::ServeDistanceProfile::Way1Low118B,
    planner::ServeDistanceProfile::Way1Low119};
  for (auto low_mode : low_modes) {
    const auto plateau = planner::EvaluateServeDistance(low_mode, 1.65f, {123,456});
    const auto stopped = planner::EvaluateServeDistance(low_mode, 3.0f, {123,456});
    // 恒速段 kp 写 1.8、停止段 ki 写 1.0，均为驱动乘之前的原值。
    assert(plateau && plateau->gains.kp == 1.8f && plateau->gains.ki == 456);
    assert(stopped && stopped->gains.kp == 123 && stopped->gains.ki == 1.0f);
  }
  planner::ScrewPositionPlanner screw;
  assert(!screw.Step());
  assert(!screw.Configure({1.0f, 0.0f}, 0.0f));
  assert(!screw.Configure({1.0f, 0.1f}, 2.0f));
  assert(screw.Configure({0.5f, 0.25f}, 0.0f));
  assert(screw.Step() == 0.25f);
  assert(screw.Step() == 0.5f);
  assert(screw.Step() == 0.5f);
  assert(screw.Configure({0.3f, 0.25f}, 0.0f));
  assert(screw.Step() == 0.25f);
  assert(screw.Step() == 0.3f);
  assert(!screw.Configure({1.0f, -0.1f}, 0.0f));
  assert(!screw.Step());
  std::printf("PASS: %u speed/PID differential samples plus invalid-input/configured-position checks\\n", count);
}
'''
    with tempfile.TemporaryDirectory(prefix='serve-distance-check-') as directory:
        fixture = Path(directory) / 'oracle.cpp'
        binary = Path(directory) / 'verify.exe'
        fixture.write_text(oracle, encoding='utf-8')
        command = [args.compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                   '-I', str(root / 'include'), str(root / 'src/serve/plate_distance.cpp'), str(root / 'src/slide/position.cpp'), str(fixture)]
        if args.compile_only:
            subprocess.run(command + ['-fsyntax-only'], check=True)
            print('PASS: distance core and differential fixture compile; numerical execution NOT RUN')
        else:
            subprocess.run(command + ['-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
