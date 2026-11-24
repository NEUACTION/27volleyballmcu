"""按EIDE已生成编译数据库复用真实ARM参数，产物只写系统临时目录。
不改生成数据库、不刷机、不覆盖已有用户构建产物；新合并模块单独补入检查。
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    """编译各真实源文件，报告失败；不把逐文件编译误称为固件链接。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--link', action='store_true', help='按现有FLASH脚本链接至TEMP，仅验证，不刷机')
    options = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    project = root / '2027_RC_vollyball_jieqiu'
    database = json.loads((project / 'build/Debug/compile_commands.json').read_text(encoding='utf-8-sig'))
    output = Path(tempfile.mkdtemp(prefix='vb-mcu-compile-'))
    commands = []
    template = None
    for item in database:
        args = shlex.split(item['command'], posix=True)
        # 编译数据库的引号是Windows双引号，shlex仅解析引号，路径中的反斜线在引号内保留。
        source = Path(item['file'])
        filtered = []
        skip = False
        for arg in args:
            if skip:
                skip = False
                continue
            if arg in ('-o', '-MF', '-MT', '-MQ'):
                skip = True
                continue
            if arg in ('-MMD', '-MD', '-MP'):
                continue
            if arg.replace('\\', '/').lower().endswith(source.name.lower()) and not arg.startswith('-'):
                continue
            filtered.append(arg)
        filtered += ['-I../protocol']
        if source.name == 'user_task.c':
            template = filtered[:]
        commands.append((source, filtered))
    assert template, 'No user_task.c compiler template found'
    new_source = project / 'User_Ctrl/Src/merged_control.c'
    if not any(path.name == new_source.name for path, _ in commands):
        commands.append((new_source, template))
    for index, (source, flags) in enumerate(commands):
        assert source.is_file(), source
        result = subprocess.run(flags + [str(source), '-o', str(output / f'{index:03d}_{source.stem}.o')], cwd=project)
        if result.returncode:
            print(f'FAIL {source.name}; temporary outputs: {output}', flush=True)
            return result.returncode
        print(f'PASS compile {index+1}/{len(commands)}: {source.name}', flush=True)
    print(f'PASS all {len(commands)} translation units. Objects: {output}', flush=True)
    if options.link:
        # 复用现有目标ABI与FLASH布局；链接产物仅用于验证，永不刷写硬件。
        compiler = template[0]
        objects = [str(output / f'{i:03d}_{source.stem}.o') for i, (source, _) in enumerate(commands)]
        link_flags = ['-mcpu=cortex-m7', '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-mthumb',
                      '--specs=nano.specs', '--specs=nosys.specs', '-Wl,--gc-sections',
                      '-u', '_printf_float', '-T', str(project/'ld/STM32H723VGTx_FLASH.ld')]
        result = subprocess.run([compiler] + link_flags + objects +
                                ['-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group',
                                 '-Wl,-Map=' + str(output/'merged.map'), '-o', str(output/'merged.elf')], cwd=project)
        if result.returncode:
            print(f'FAIL firmware link; artifacts: {output}', flush=True)
            return result.returncode
        print(f'PASS firmware link (NOT flashed): {output / "merged.elf"}', flush=True)
    else:
        print('Firmware link NOT RUN; pass --link to check', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
