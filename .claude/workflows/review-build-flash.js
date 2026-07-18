// Workflow: 审查 CODE/build_flash.py (465 行)
// 用途: 把 CODE/ 精简为 CODE/.flash/ 用于上传到 MCU
//
// 主要功能:
//   1. tokenize 去 # 注释
//   2. 去除 docstring (模块/类/函数)
//   3. 压缩空行 (连续 >=2 -> 1)
//   4. 去除尾随空格
//   5. 跳过 [例程] / build_flash.py
//   6. config.json "调试输出"=false 时剥离 log.py 与所有 info/log_setup 调用
// 关联文件: CODE/config.json (用 "__xxx" 作为内联注释), CODE/log.py, CODE/main.py, CODE/match*.py

export const meta = {
  name: 'review-build-flash',
  description: '审查 build_flash.py 的 bug 和功能完整性',
  phases: [
    { title: '分析' },
    { title: '验证' },
    { title: '汇总' }
  ]
}

const TASKS = [
  {
    name: 'python-strip',
    prompt: [
      '你是 Python 代码审查专家。审查 build_flash.py 的 strip_python 和 _is_docstring_line 函数。',
      '',
      '【文件路径】C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py',
      '【阅读方式】PowerShell: Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py" -Raw | Out-String',
      '【关联文件】CODE/main.py, CODE/match.py 等被处理的文件 —— 抽样看一两个文件理解 docstring 长什么样。',
      'Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\main.py" -Raw | Out-String',
      'Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\match.py" -Raw | Out-String',
      '',
      '【重点检查】',
      '1. tokenize 去除 COMMENT 后 untokenize 重建源码 —— tokenize.untokenize 是 round-trip 吗?',
      '2. 多行 docstring 检测 (in_triple 状态机) 边界: ',
      '   - 字符串里包含三引号 (x="""abc""" 形式)',
      '   - 单行 triple 但带前缀 (r"""xxx""")',
      '   - docstring 后接代码 ("""doc"""\\ncode)',
      '3. _is_docstring_line 对单行 docstring 的判断:',
      '   - x = "abc" 这种赋值会被误判为 docstring 吗? (注意函数里 stripped 是 line.strip() 后整行)',
      '   - "abc" 在行尾 (前面有 code)? 例如: x = foo()  # "comment"',
      '4. 移除 artifact 单行 \\\\: 这是要处理 Python 续行符吗? 为什么要这么做',
      '5. 压缩连续空行: 文档说"连续 >=2 -> 1"，代码先 tokenize 处理 NL 再 split 是否还能正确压缩?',
      '6. 末尾 return 是 +"\\n" —— 如果原文末尾本来就没有 \\n 会怎样?',
      '7. src_text.decode("utf-8") —— 如果文件是 GBK 编码会崩吗?',
      '',
      '【返回格式】每条发现: { file, line, summary, severity(CRITICAL/HIGH/MEDIUM/LOW), evidence, suggested_fix }',
      '【返回结构】{ findings: [...] }'
    ].join('\n')
  },
  {
    name: 'ast-log-strip',
    prompt: [
      '你是 AST/语法树专家。审查 build_flash.py 的 strip_log_lines 函数 (用 AST NodeTransformer)。',
      '',
      '【文件路径】C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py',
      '【阅读方式】PowerShell: Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py" -Raw | Out-String',
      '【关联文件】',
      'Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\log.py" -Raw | Out-String',
      'Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\main.py" -Raw | Out-String',
      'Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\match.py" -Raw | Out-String',
      '',
      '【重点检查】',
      '1. visit_ImportFrom 移除 from log import —— 如果其他模块还 import log 但调用了非 info/log_setup 函数, 怎么处理?',
      '2. visit_Expr 删除 info(...) / log_setup(...) 顶层调用:',
      '   - 模块顶层表达式 info("start") 是合法吗?',
      '   - 如果 info() 嵌套在另一个调用内 (print(info("x"))) 会不会被错误删除?',
      '   - _LOG_FUNCS 只匹配顶层, 如果赋给变量 (f = info; f()) 会怎样?',
      '3. visit_If 检测 cfg.debug_output 整块删除 —— ',
      '   - 紧凑写法 if cfg.debug_output: x=1\\nelse: y=2 AST 怎么处理?',
      '   - 反向 if not cfg.debug_output: y=2 怎么办?',
      '4. _normalize_body 和 _visit_with_pass_body:',
      '   - 容器 body 为空时插入 ast.Pass() —— MicroPython 接受吗?',
      '   - if body 空 + orelse 单个 If: 返回 orelse[0] —— elif 提升为外层 if, 对缩进/上下文影响如何?',
      '5. visit_Try: handlers 块可以为空? MicroPython 的 try...except: (空) 合法吗?',
      '6. fix_missing_locations 是否对所有节点都生效?',
      '7. self.generic_visit(node) 在 _visit_with_pass_body 末尾调用 —— 这意味着子节点会被再次 visit, 会不会重复删除/出错?',
      '',
      '【返回格式】每条发现: { file, line, summary, severity, evidence, suggested_fix }',
      '【返回结构】{ findings: [...] }'
    ].join('\n')
  },
  {
    name: 'json-strip',
    prompt: [
      '你是 JSON/正则专家。审查 build_flash.py 的 strip_json_comments 和 strip_json_for_build。',
      '',
      '【文件路径】C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py',
      '【阅读方式】PowerShell: Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py" -Raw | Out-String',
      '【关联 config.json】',
      'Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\config.json" -Raw | Out-String',
      '',
      '【重点检查】',
      '1. _INLINE 正则只匹配行内尾部 ", "__xxx": value" 的形式。',
      '   - 如果 __xxx 的 value 跨越多行 (嵌套 object {...}) 怎么办?',
      '   - 如果 __key 后面跟着嵌套数组/对象 (跨多行), 会被 partial 删除留下垃圾吗?',
      '2. 整行注释: s.startswith(""//"") 或 s.startswith(""__"") —— ',
      '   - CODE/config.json 用 ""//--- xxx ---"" 作为节标题',
      '   - 如果一行是 "key": "value", "__xxx" 在 value 位置 (而非 key 位置) 怎么办?',
      '3. 行内删除后留下的尾逗号:',
      '   - re.sub 只处理 ,}\\n 和 ,]\\n',
      '   - 如果 ", "__xxx": true,\\n"foo":1" 在中间, 第二个元素前会多一个逗号吗?',
      '4. strip_json_for_build 里 "调试输出" 中文 key 删除:',
      '   - 先 strip_json_comments 再 parse 再 dump —— 但前面 strip 已经把 __key 删了',
      '   - 假定 json.loads 成功才删 —— 失败时仍保留 cleaned (其中 "调试输出" 还在)',
      '5. read_debug_output: strip_json_comments 已经能去掉 __key, json.loads 之后 data.get("调试输出")——',
      '   - 如果原 config.json 写 "调试输出": true, 没问题; 但如果格式略有不同(空格、unicode escape)?',
      '',
      '【返回格式】每条发现: { file, line, summary, severity, evidence, suggested_fix }',
      '【返回结构】{ findings: [...] }'
    ].join('\n')
  },
  {
    name: 'file-traversal-main',
    prompt: [
      '你是 Python 文件系统专家。审查 build_flash.py 的 process_tree 和 main。',
      '',
      '【文件路径】C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py',
      '【阅读方式】PowerShell: Get-Content "C:\\Temp\\MicroPython\\#WCBSG\\CODE\\build_flash.py" -Raw | Out-String',
      '',
      '【重点检查】',
      '1. dst_root 在 src_root 内 (默认 CODE/.flash 在 CODE/ 下):',
      '   - os.walk 遍历 src 时如果遇到 dst 会进 dst 子目录吗?',
      '   - SKIP_DIR_NAMES 里有 .flash 所以被过滤掉 —— rmtree(dst_root) 是否必要?',
      '   - 增量构建: 旧文件还在但不再 walk, shutil.rmtree 是必需的。',
      '2. .json 文件处理: 复制所有 .json, 注释删除. 哪些 json 应该跳过?',
      '3. main 函数:',
      '   - src/dst 相对路径处理是否正确?',
      '   - read_debug_output(os.path.join(src, "config.json")) 是否正确?',
      '   - 返回值: n_err == 0 时 return 0, 否则 return 2 —— 但没有 fail-fast',
      '4. 字符编码: 输出文件用 "utf-8", newline="\\n". 输入也用 utf-8 read. MicroPython 默认读取 UTF-8?',
      '5. should_skip_file 只跳过 build_flash.py 自身. 那其他 .py 都被处理?',
      '6. 边界:',
      '   - __init__.py 怎么处理?',
      '   - src_root 不存在时 main 有判断 ✓',
      '7. windows 路径兼容性: [例程] 目录名是否被正确跳过?',
      '',
      '【返回格式】每条发现: { file, line, summary, severity, evidence, suggested_fix }',
      '【返回结构】{ findings: [...] }'
    ].join('\n')
  }
]

const FINDINGS_SCHEMA = {
  type: 'object',
  properties: {
    findings: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          file: { type: 'string' },
          line: { type: 'integer' },
          summary: { type: 'string' },
          severity: { type: 'string', enum: ['CRITICAL', 'HIGH', 'MEDIUM', 'LOW'] },
          evidence: { type: 'string' },
          suggested_fix: { type: 'string' }
        },
        required: ['summary', 'severity', 'evidence']
      }
    }
  },
  required: ['findings']
}

const results = await parallel(TASKS.map(t => () =>
  agent(t.prompt, { label: t.name, phase: '分析', schema: FINDINGS_SCHEMA })
))

const all = results.filter(Boolean).flatMap(r => r.findings || [])
return { rawCount: all.length, findings: all }