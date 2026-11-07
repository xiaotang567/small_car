# -*- coding: utf-8 -*-
"""一次性把一条命令发给板子（板子是 TCP 客户端，所以这里当服务端听 8086）。

用法：  python rb_off.py "rb 0"
注意：  必须先退出 car_ui.py（同一个端口只能一个监听）。
"""
import socket
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 2 else 8086
CMD = sys.argv[1] if len(sys.argv) > 1 else 'rb 0'

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('0.0.0.0', PORT))
srv.listen(1)
srv.settimeout(40)
print('listening :%d ...' % PORT)
conn, addr = srv.accept()
print('connected from %s' % (addr,))
time.sleep(0.6)
conn.sendall((CMD + '\n').encode('ascii'))
print('sent: %r' % CMD)

conn.settimeout(0.5)
t0 = time.time()
txt = b''
while time.time() - t0 < 3.0:
    try:
        d = conn.recv(8192)
    except socket.timeout:
        continue
    if not d:
        break
    txt += d

# 只挑可打印的整行（图像帧是二进制的，跳过）
out = []
for chunk in txt.split(b'\n'):
    s = ''.join(chr(c) for c in chunk if 32 <= c < 127)
    if 'ACK' in s or 'CFG' in s:
        out.append(s.strip())
print('--- replies ---')
for s in out[-6:]:
    print(s)
conn.close()
srv.close()
