"""只读核对所有真实GetSlidePosition函数，可编译原函数oracle差分夹具。"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

# 用户已把WAY0低位目标改为11.42圈；只在该模式的oracle中应用同一调整，其余原算法保持。
TARGET_OVERRIDES = {'Way0Low': '11.42f'}

# 源码仅有这五个函数体：同名函数按WAY分开，注释及调用位置不计分支。
CASES = [
    ('Way0High', 'GetSlidePositionHigh', 1045, 1066),
    ('Way0Low', 'GetSlidePositionLow', 1071, 1082),
    ('Way1High', 'GetSlidePositionHigh', 3319, 3332),
    ('Way1Low', 'GetSlidePositionLow', 3337, 3347),
    ('Way1HighDown', 'GetSlidePositionHighDown', 3352, 3362),
]


def uncomment(text):
    """只删除注释，不忽略任何有效常量、比较符或赋值。"""
    return re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.S)


def normalize(text):
    """忽略排版空白，用于有效语句精确对照。"""
    return re.sub(r'\s+', '', uncomment(text))


def main():
    """先对照源码，再按编译器类型选择只编译或执行临时差分程序。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=Path(
        r'D:\action\大合体\排球发球\User_Ctrl\Src\user_task.c'))
    parser.add_argument('--compiler')
    parser.add_argument('--compile-only', action='store_true', help='交叉编译时只检查语法，不执行产物')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = args.reference.read_text(encoding='utf-8-sig').splitlines()
    migrated = uncomment((root / 'src/slide/position.cpp').read_text(encoding='utf-8'))
    oracle = '#include "planner/slide/position.hpp"\n#include <cassert>\n#include <cstdio>\n'
    rows = re.findall(r'\{([0-9.]+f),\s*([0-9.]+f / [0-9.]+f)\}', migrated)
    assert len(rows) == len(CASES)
    for (mode, name, first, last), row in zip(CASES, rows):
        original = uncomment('\n'.join(source[first-1:last]))
        assert original.startswith(f'float {name}(void)'), mode
        target = re.search(r'float targetPosition\s*=\s*([0-9.]+f);', original)[1]
        increment = re.search(r'currentPosition = currentPosition \+ ([^;]+);', original)[1]
        # 原函数仍保留各自的>=或>，用于核对简化后的统一限幅；两者相等时输出数值一致。
        if mode in TARGET_OVERRIDES:
            adjusted = TARGET_OVERRIDES[mode]
            original = re.sub(r'(float targetPosition\s*=\s*)[0-9.]+f', lambda m: m[1] + adjusted, original)
            target = adjusted
        assert tuple(map(normalize, row)) == (normalize(target), normalize(increment)), mode
        oracle += f'namespace oracle_{mode} {{\n{original}\n}}\n'
    assert 'GetSlidePosition' not in migrated
    print('PASS: all 5 targets/increments match reference plus explicit Way0Low=11.42f adjustment')
    if not args.compiler:
        print('C++ runtime differential test NOT RUN: pass --compiler <native g++ or clang++>')
        return
    oracle += 'int main() {\nunsigned count = 0;\n'
    for mode, name, _, _ in CASES:
        oracle += f'''{{
  const auto profile = planner::HistoricalSlideProfile::{mode};
  planner::HistoricalSlidePlanner migrated(profile);
  const auto preset = planner::GetHistoricalSlidePreset(profile);
  assert(preset);
  float first = 0;
  for (int i = 0; i < 10000; ++i) {{
    const float expected = oracle_{mode}::{name}();
    const auto actual = migrated.Step();
    assert(actual && *actual == expected);
    if (i == 0) first = *actual;
    if (i == 9999) assert(*actual == preset->target_position_turns);
    ++count;
  }}
  assert(first == preset->increment_per_call_turns);
  migrated.Reset();
  assert(migrated.Step() == first);
  planner::HistoricalSlidePlanner independent(profile);
  assert(independent.Step() == first);
}}
'''
    oracle += '''
  const auto invalid_profile = static_cast<planner::HistoricalSlideProfile>(99);
  planner::HistoricalSlidePlanner invalid(invalid_profile);
  assert(!invalid.Step());
  assert(!planner::GetHistoricalSlidePreset(invalid_profile));
  const auto high0 = planner::GetHistoricalSlidePreset(planner::HistoricalSlideProfile::Way0High);
  const auto low0 = planner::GetHistoricalSlidePreset(planner::HistoricalSlideProfile::Way0Low);
  const auto down1 = planner::GetHistoricalSlidePreset(planner::HistoricalSlideProfile::Way1HighDown);
  assert(high0->target_position_turns == 241.16f);
  assert(low0->target_position_turns == 11.42f); // 当前用户调整值，不将原始711.42覆盖回去。
  assert(down1->target_position_turns == 241.17f);
  std::printf("PASS: %u slide samples plus reset/independence/preset/invalid checks\\n", count);
}
'''
    with tempfile.TemporaryDirectory(prefix='slide-reference-') as directory:
        fixture = Path(directory) / 'oracle.cpp'
        binary = Path(directory) / 'verify.exe'
        fixture.write_text(oracle, encoding='utf-8')
        command = [args.compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                   '-I', str(root / 'include'), str(root / 'src/slide/position.cpp'), str(fixture)]
        if args.compile_only:
            subprocess.run(command + ['-fsyntax-only'], check=True)
            print('PASS: slide core and differential fixture compile; numerical execution NOT RUN')
        else:
            subprocess.run(command + ['-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
