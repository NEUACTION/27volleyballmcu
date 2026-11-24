"""统一C/C++格式：每层四空格，代码块花括号独行，初始化和常量记录保持紧凑，赋值两侧留空格；不改逻辑。
默认覆盖新PC、共享协议、本次整合的主控文件；--check只检查，其他路径可显式传入。
脚本识别字符串、注释、预处理、多行括号及无花括号控制体，不对第三方驱动批量排版。
"""
import argparse
from bisect import bisect_right
from pathlib import Path
import re
import tempfile

# 每个匹配保留完整原文；字符串/注释中的花括号不会作为代码结构计数。
LEX = re.compile(r'(?P<pp>^[ \t]*\#(?:[^\n]*\\\r?\n)*[^\n]*)|(?P<comment>/\*.*?\*/|//[^\n]*)|(?P<string>R"(?P<delim>[^ ()\\\t\r\n]{0,16})\(.*?\)(?P=delim)"|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\')|(?P<word>[A-Za-z_]\w*|(?:\d+(?:\.\d*)?|\.\d+)(?:[eEpP][+-]?\d+)?[A-Za-z0-9_]*)|(?P<op>>>=|<<=|->|\+\+|--|&&|\|\||==|!=|<=|>=|<<|>>|\+=|-=|\*=|/=|%=|&=|\|=|\^=|[^\s])', re.S | re.M)


def reindent(text):
    """根据真实代码括号层次返回新文本；有效记号和每行非缩进内容必须不变。"""
    lines = text.splitlines(keepends=True)
    starts = [0]
    for line in lines:
        starts.append(starts[-1] + len(line))
    all_tokens = list(LEX.finditer(text))
    code = [t for t in all_tokens if t.lastgroup not in ('pp', 'comment')]
    values = [t.group() for t in code]
    pairs = {}
    stack = []
    before = []
    brace_depth = 0
    parens = []
    for i, token in enumerate(code):
        value = values[i]
        before.append((brace_depth, tuple(parens)))
        if value in ('(', '[', '{'):
            stack.append(i)
            if value == '{':
                brace_depth += 1
            elif value == '(':
                parens.append((bisect_right(starts, token.start()) - 1, brace_depth))
        elif value in (')', ']', '}'):
            if not stack or values[stack[-1]] != {')': '(', ']': '[', '}': '{'}[value]:
                raise ValueError('括号不配对；拒绝写入')
            opening = stack.pop()
            pairs[opening] = i
            if value == '}':
                brace_depth -= 1
            elif value == ')':
                parens.pop()
    if stack:
        raise ValueError('文件存在未闭合括号；拒绝写入')

    def statement_end(index):
        """寻找单条控制体终点，以便无花括号的if/for体也保留一级缩进。"""
        if index >= len(code):
            return index
        if values[index] == '{':
            return pairs[index] + 1
        if values[index] in ('if', 'for', 'while', 'switch') and index + 1 < len(code) and values[index + 1] == '(':
            end = statement_end(pairs[index + 1] + 1)
            if values[index] == 'if' and end < len(code) and values[end] == 'else':
                end = statement_end(end + 1)
            return end
        cursor = index
        while cursor < len(code):
            if values[cursor] == ';':
                return cursor + 1
            if values[cursor] in ('(', '[', '{'):
                cursor = pairs[cursor] + 1
            elif values[cursor] == '}':
                return cursor
            else:
                cursor += 1
        return cursor

    extra = [0] * (len(code) + 1)
    for i, value in enumerate(values):
        body = None
        if value in ('if', 'for', 'while', 'switch', 'catch') and i + 1 < len(code) and values[i + 1] == '(':
            body = pairs[i + 1] + 1
        elif value in ('else', 'do'):
            body = i + 1
            if body < len(code) and values[body] == 'if':
                continue
        if body is not None and body < len(code) and values[body] not in ('{', ';'):
            end = statement_end(body)
            extra[body] += 1
            extra[end] -= 1
    level = 0
    for i in range(len(code)):
        level += extra[i]
        extra[i] = level

    first_code = {}
    for i, token in enumerate(code):
        first_code.setdefault(bisect_right(starts, token.start()) - 1, i)
    protected = set()
    comment_starts = {}
    for token in all_tokens:
        line = bisect_right(starts, token.start()) - 1
        end = bisect_right(starts, token.end() - 1) - 1
        if token.lastgroup == 'pp' or (token.lastgroup == 'string' and end > line):
            # 多行字面量/宏续行保持完整，避免修改宏语义或字符串内容。
            protected.update(range(line, end + 1))
        elif token.lastgroup == 'comment' and text[starts[line]:token.start()].strip() == '':
            for row in range(line, end + 1):
                comment_starts[row] = line
    result = []
    cursor = 0
    last_depth = 0
    comment_indent = {}
    for row, line in enumerate(lines):
        if row in protected or not line.strip():
            result.append(line if row in protected else ('\r\n' if line.endswith('\r\n') else '\n') if line.endswith('\n') else '')
            continue
        if row in first_code:
            i = first_code[row]
            depth, opened = before[i]
            depth -= values[i] == '}'
            indent = depth + extra[i]
            if values[i] == ':':
                indent += 1  # 构造函数成员初始化列表和分行三元表达式续行。
            if opened and opened[-1][0] < row:
                indent += 1
            last_depth = before[i][0]
            # 行尾所有花括号对后续纯注释行也生效。
            cursor = i
            while cursor < len(code) and code[cursor].start() < starts[row + 1]:
                last_depth = before[cursor][0] + (values[cursor] == '{') - (values[cursor] == '}')
                cursor += 1
        else:
            indent = last_depth
            if cursor < len(before) and before[cursor][1] and before[cursor][1][-1][0] < row:
                indent += 1
        if row in comment_starts:
            first = comment_starts[row]
            if row == first:
                comment_indent[first] = indent
            indent = comment_indent.get(first, indent)
        content = line.lstrip(' \t')
        prefix = ' ' * (4 * max(0, indent))
        if row in comment_starts and row != comment_starts[row] and content.startswith('*'):
            prefix += ' '
        ending = '\r\n' if line.endswith('\r\n') else '\n' if line.endswith('\n') else ''
        result.append(prefix + content.rstrip(' \t\r\n') + ending)
    formatted = ''.join(result)
    assert [line.strip(' \t') for line in text.splitlines()] == [line.strip(' \t') for line in formatted.splitlines()], '每行非空白内容改变'
    assert [(t.lastgroup, t.group()) for t in LEX.finditer(text) if t.lastgroup not in ('pp', 'comment')] == [(t.lastgroup, t.group()) for t in LEX.finditer(formatted) if t.lastgroup not in ('pp', 'comment')], '有效代码或注释内容改变'
    return formatted


def code_signature(text):
    """比较有效C/C++记号；宏先去物理续行，再比较，不能改变宏定义含义。"""
    signature = []
    for token in LEX.finditer(text):
        if token.lastgroup == 'comment':
            continue
        if token.lastgroup == 'pp':
            logical = re.sub(r'\\\r?\n', '', token.group()).lstrip()
            # 去掉#以免被重新识别为整条预处理指令，逐个比较宏内记号。
            signature.append(('pp', tuple(t.group() for t in LEX.finditer(logical[1:]) if t.lastgroup != 'comment')))
        else:
            signature.append((token.lastgroup, token.group()))
    return signature


def empty_initializer_ranges(text):
    """识别空值初始化，不把函数空体或空命名空间误当初始化。"""
    tokens = [t for t in LEX.finditer(text) if t.lastgroup not in ('pp', 'comment')]
    ranges = []
    for i in range(1, len(tokens) - 1):
        if tokens[i].group() != '{' or tokens[i + 1].group() != '}':
            continue
        previous = tokens[i - 1].group()
        declaration = tokens[i - 2].group() if i >= 2 else ''
        if previous in ('=', '>', ']') or (re.fullmatch(r'[A-Za-z_]\w*', previous) and previous not in ('else', 'do', 'try', 'namespace') and declaration not in ('struct', 'class', 'enum', 'namespace')):
            ranges.append((tokens[i].start(), tokens[i + 1].end(), tokens[i - 1].end()))
    return ranges


def compact_empty_initializers(text):
    """必要的数组/对象空初始化保持一行；标量明确初值由源码声明给出。"""
    for start, end, previous_end in reversed(empty_initializer_ranges(text)):
        if any(t.lastgroup == 'comment' for t in LEX.finditer(text[previous_end:end])):
            continue
        replacement = (' ' if text[previous_end - 1:previous_end] == '=' else '') + '{}'
        semicolon = re.match(r'\s*;', text[end:])
        if semicolon:
            end += semicolon.end()
            replacement += ';'
        text = text[:previous_end] + replacement + text[end:]
    return text


def space_assignment_operators(text):
    """赋值及相等/大小比较运算符两侧用空格，不改字符串、注释、换行和复合运算符本身。"""
    changes = []
    operators = {'=', '+=', '-=', '*=', '/=', '%=', '&=', '|=', '^=', '<<=', '>>=', '==', '!=', '<=', '>='}
    for token in LEX.finditer(text):
        if token.lastgroup == 'op' and token.group() in operators:
            left, right = token.start(), token.end()
            while left > 0 and text[left - 1] in ' \t':
                left -= 1
            while right < len(text) and text[right] in ' \t':
                right += 1
            prefix = ' ' if left > 0 and text[left - 1] not in '\r\n' else text[left:token.start()]
            suffix = ' ' if right < len(text) and text[right] not in '\r\n' else ''
            changes.append((left, right, prefix + token.group() + suffix))
    for start, end, replacement in reversed(changes):
        text = text[:start] + replacement + text[end:]
    return text


def initializer_ranges(text):
    """识别普通变量初始化/聚合赋值；常量表外层仍使用独行规则，lambda与控制块不按初始化拆排。"""
    tokens = [t for t in LEX.finditer(text) if t.lastgroup not in ('pp', 'comment')]
    stack, result = [], []
    for i, token in enumerate(tokens):
        value = token.group()
        if value == '{':
            previous = tokens[i - 1].group() if i else ''
            j = i - 1
            while j >= 0 and tokens[j].group() not in (';', '{', '}'):
                j -= 1
            prefix = [t.group() for t in tokens[j + 1:i]]
            inherited = bool(stack and stack[-1][1])
            direct = (previous in ('=', '>') or bool(re.fullmatch(r'[A-Za-z_]\w*', previous)))
            definition = any(word in prefix for word in ('struct', 'class', 'enum', 'namespace'))
            control = previous in ('else', 'do', 'try', 'namespace')
            constant = any(word in prefix for word in ('const', 'constexpr'))
            is_initializer = inherited or (direct and not definition and not control and not constant)
            stack.append((token.start(), is_initializer))
        elif value == '}':
            opening, is_initializer = stack.pop()
            if is_initializer:
                result.append((opening, token.end()))
    return result


def split_braces(text):
    """将有效代码中的每个花括号单独成行；字符串和注释保持原文。"""
    newline = '\r\n' if '\r\n' in text else '\n'
    chunks = []
    cursor = 0
    preserved_ranges = initializer_ranges(text) + [(a, b) for a, b, _ in empty_initializer_ranges(text)]
    for token in LEX.finditer(text):
        if token.lastgroup == 'pp':
            original = token.group()
            logical = re.sub(r'\\\r?\n', '\n', original)
            inner = list(LEX.finditer(logical.lstrip()[1:]))
            if not any(t.group() in ('{', '}') and t.lastgroup == 'op' for t in inner):
                continue
            if any(t.lastgroup == 'comment' and t.group().startswith('//') for t in inner):
                raise ValueError('含花括号与行注释的宏需保留原物理行，拒绝盲目改写')
            # #替换为普通字符，只为禁止此处再次匹配pp；恢复后语义不变。
            hash_at = logical.index('#')
            surrogate = logical[:hash_at] + '@' + logical[hash_at + 1:]
            expanded = split_braces(surrogate).replace('@', '#', 1)
            rows = [row.strip() for row in expanded.splitlines() if row.strip()]
            depth = 0
            physical = []
            for index, row in enumerate(rows):
                depth -= row == '}'
                prefix = '' if index == 0 else ' ' * (4 * max(depth, 0))
                physical.append(prefix + row + (' \\' if index < len(rows) - 1 else ''))
                depth += row == '{'
            replacement = newline.join(physical)
            chunks.extend([text[cursor:token.start()], replacement])
            cursor = token.end()
        elif token.lastgroup == 'op' and token.group() in ('{', '}'):
            if any(start <= token.start() < end for start, end in preserved_ranges):
                continue
            previous = text[text.rfind('\n', 0, token.start()) + 1:token.start()]
            end = text.find('\n', token.end())
            following = text[token.end():end if end >= 0 else len(text)]
            before_break = newline if previous.strip() else ''
            after_break = newline if following.strip() else ''
            chunks.extend([text[cursor:token.start()], before_break, token.group(), after_break])
            cursor = token.end()
    chunks.append(text[cursor:])
    return ''.join(chunks)


def constant_record_ranges(text):
    """找出const/constexpr初始化表中没有嵌套子表的记录；外层表仍独占花括号行。"""
    tokens = [t for t in LEX.finditer(text) if t.lastgroup not in ('comment', 'pp')]
    pairs = {}
    stack = []
    for i, token in enumerate(tokens):
        if token.group() == '{':
            stack.append(i)
        elif token.group() == '}':
            opening = stack.pop()
            pairs[opening] = i
    ranges = set()
    for i, token in enumerate(tokens):
        if token.group() not in ('constexpr', 'const'):
            continue
        equals = False
        for j in range(i + 1, len(tokens)):
            value = tokens[j].group()
            if value == ';':
                break
            if value == '=':
                equals = True
            if value == '{':
                if equals:
                    outer_end = pairs[j]
                    for child in range(j + 1, outer_end):
                        if tokens[child].group() == '{' and not any(t.group() == '{' for t in tokens[child + 1:pairs[child]]):
                            start, end = tokens[child].start(), tokens[pairs[child]].end()
                            if not any(t.lastgroup == 'comment' for t in LEX.finditer(text[start:end])):
                                ranges.add((start, end))
                break
    return sorted(ranges)


def compact_constant_records(text):
    """把常量表内部的单条记录恢复为{字段, 字段}，逗号跟随该记录；保留外层与分号独行。"""
    for start, end in reversed(constant_record_ranges(text)):
        record = text[start:end]
        chunks = []
        previous = 0
        for token in LEX.finditer(record):
            gap = record[previous:token.start()]
            if gap and chunks and token.group() != '}' and chunks[-1] != '{':
                chunks.append(' ')
            chunks.append(token.group())
            previous = token.end()
        replacement = ''.join(chunks)
        comma = re.match(r'[ \t\r\n]*,', text[end:])
        if comma:
            end += comma.end()
            replacement += ','
        text = text[:start] + replacement + text[end:]
    return text


def pack_constant_rows(text, width=110):
    """相邻短常量记录按行宽并排；带注释或超长记录不合并。"""
    ranges = constant_record_ranges(text)
    rows = text.splitlines(keepends=True)
    candidates = set()
    offset = 0
    for row, line in enumerate(rows):
        start = offset + len(line) - len(line.lstrip(' \t'))
        if any(a == start and text[b:offset + len(line)].strip() == ',' for a, b in ranges):
            candidates.add(row)
        offset += len(line)
    result = []
    previous_candidate = False
    for row, line in enumerate(rows):
        prefix = line[:len(line) - len(line.lstrip(' \t'))]
        if row in candidates and previous_candidate and result and result[-1].startswith(prefix + '{') and len(result[-1].rstrip()) + 1 + len(line.strip()) <= width:
            ending = '\r\n' if line.endswith('\r\n') else '\n'
            result[-1] = result[-1].rstrip() + ' ' + line.strip() + ending
        else:
            result.append(line)
        previous_candidate = row in candidates
    return ''.join(result)


def format_source(text):
    """普通代码花括号独行；常量表保留紧凑记录，再统一四空格缩进。"""
    formatted = pack_constant_rows(reindent(compact_constant_records(split_braces(compact_empty_initializers(space_assignment_operators(text))))))
    assert code_signature(text) == code_signature(formatted), '格式化改变有效代码或宏定义'
    compact_ranges = constant_record_ranges(formatted) + initializer_ranges(formatted) + [(a, b) for a, b, _ in empty_initializer_ranges(formatted)]
    for token in LEX.finditer(formatted):
        if token.lastgroup == 'op' and token.group() in ('{', '}'):
            if any(start <= token.start() < end for start, end in compact_ranges):
                continue
            start = formatted.rfind('\n', 0, token.start()) + 1
            end = formatted.find('\n', token.end())
            assert formatted[start:end if end >= 0 else len(formatted)].strip() == token.group(), '花括号未独占一行'
    return formatted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='*', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    paths = args.paths or sorted([*root.glob('pc/src/**/*.cpp'), *root.glob('pc/src/**/*.hpp'), *root.glob('protocol/**/*.h'), *root.glob('protocol/**/*.c')])
    if not args.paths:
        project = root / '2027_RC_vollyball_jieqiu/User_Ctrl'
        paths += [project / folder / name for folder, name in [('Src', 'merged_control.c'), ('Inc', 'merged_control.h'), ('Src', 'user_task.c'), ('Src', 'user_communication.c'), ('Inc', 'user_communication.h'), ('Src', 'user_usart.c'), ('Inc', 'user_usart.h')]]
    backup = None
    changed = 0
    for path in paths:
        original = path.read_bytes()
        bom = original.startswith(b'\xef\xbb\xbf')
        text = original.decode('utf-8-sig')
        formatted = format_source(text)
        assert format_source(formatted) == formatted, '格式化必须幂等：' + str(path)
        if formatted == text:
            continue
        changed += 1
        if not args.check:
            backup = backup or Path(tempfile.mkdtemp(prefix='vb-indent-backup-'))
            relative = path.relative_to(root)
            destination = backup / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(original)
            path.write_bytes((b'\xef\xbb\xbf' if bom else b'') + formatted.encode('utf-8'))
        print(('NEEDS FORMAT ' if args.check else 'FORMATTED ') + str(path))
    print(f'{len(paths)} files checked; {changed} changed; backup={backup}')
    return 1 if args.check and changed else 0


if __name__ == '__main__':
    raise SystemExit(main())
