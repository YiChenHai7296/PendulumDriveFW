# -*- coding: utf-8 -*-
"""Fix garbled chars - try binary replace"""
with open('usart.c', 'rb') as f:
    data = f.read()

# Common garbled byte sequences when GBK/GB2312 is read as UTF-8
# Value 0xe6 (义 in UTF-8 义的 first byte) - garbled often shows as 1 byte
# Try replacing EF BF BD (U+FFFD UTF-8) with correct UTF-8 sequences
repl = [
    (b'\xef\xbf\xbd\xe5\xae\x9a ', b'\xe5\xae\x9a\xe4\xb9\x89 '),  # ?定 -> 定义
]
# Actually simpler: read as latin-1 (no conversion), find patterns, replace
# Or: decode with errors=replace, then the ? chars become \ufffd
with open('usart.c', 'r', encoding='utf-8', errors='replace') as f:
    s = f.read()

# Count replacement chars
cnt = s.count('\ufffd')
print('U+FFFD count:', cnt)

# Do replacements - the replacement char is \ufffd
import re
s = re.sub(r'私有宏定\ufffd ', '私有宏定义 ', s)
s = re.sub(r'\ufffd6字节=\ufffd新帧，后6字节=上一\ufffd', '前6字节=最新帧，后6字节=上一帧', s)
s = re.sub(r'出队\ufffd帧', '出队一帧', s)
s = re.sub(r'（\ufffd simulink', '（供 simulink', s)
s = re.sub(r'（字节\ufffd）', '（字节数）', s)
s = re.sub(r'成功\ufffd-1', '成功，-1', s)
s = re.sub(r'发\ufffd\ufffd一帧数\ufffd', '发送一帧数据', s)
s = re.sub(r'缓冲区指\ufffd', '缓冲区指针', s)
s = re.sub(r'发送失\ufffd', '发送失败', s)
s = re.sub(r'串\ufffd3\ufffd4\ufffd5', '串口3/4/5', s)
s = re.sub(r'发\ufffd\ufffd单字节', '发送单字节', s)
s = re.sub(r'均使\ufffd DMA 发\ufffd', '均使用 DMA 发送', s)
s = re.sub(r'用\ufffd UART1', '用于 UART1', s)
s = re.sub(r'静\ufffd\ufffd缓冲区', '静态缓冲区', s)
s = re.sub(r'生命周期有效\ufffd', '生命周期有效', s)
s = re.sub(r'调用）\ufffd', '调用）', s)
s = re.sub(r'\ufffd6字节=\ufffd新帧', '前6字节=最新帧', s)
s = re.sub(r'移至\ufffd6字节', '移至前6字节', s)
s = re.sub(r'写入\ufffd6字节', '写入前6字节', s)
s = re.sub(r'\ufffd仅 IDLE', '仅 IDLE', s)
s = re.sub(r'不重启）\ufffd', '不重启）', s)
s = re.sub(r'DMA使用\ufffd', 'DMA使用', s)

with open('usart.c', 'w', encoding='utf-8', newline='') as f:
    f.write(s)
print('Done, remaining FFFD:', s.count('\ufffd'))
