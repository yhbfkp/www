# -*- coding: utf-8 -*-
"""
给 Keil 工程加 Car 组, 移出平衡车残留文件, 加头文件路径
用法: python fix_uvprojx.py
"""
import re, shutil, os, sys

P = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 'BalanceCar', 'MDK-ARM', 'BalanceCar.uvprojx')

shutil.copyfile(P, P + '.bak_uvprojx')
print('backup ->', P + '.bak_uvprojx')

s = open(P, 'rb').read().decode('utf-8', errors='replace')

# ---------- 1. 移出这些文件(平衡车残留 / 已废的 SPI OLED) ----------
REMOVE = [
    r'..\..\App\Src\control.c',
    r'..\..\App\Src\filter.c',
    r'..\..\App\Src\inv_mpu.c',
    r'..\..\App\Src\inv_mpu_dmp_motion_driver.c',
    r'..\..\App\Src\MPU6050.c',
    r'..\..\App\Src\show.c',
    r'..\..\App\Src\pid.c',
    r'..\..\CSL\Src\oled.c',
]

for path in REMOVE:
    pat = re.compile(
        r'\s*<File>\s*<FileName>[^<]*</FileName>\s*<FileType>1</FileType>\s*<FilePath>'
        + re.escape(path) + r'</FilePath>\s*</File>', re.I)
    s, n = pat.subn('', s)
    print(('  removed ' if n else '  MISS    ') + path)

# ---------- 2. 插入 Car 组 ----------
CAR_FILES = [
    'car_core.c', 'car_motor.c', 'car_encoder.c', 'car_pid.c',
    'car_gray.c', 'car_mpu.c', 'car_key.c', 'car_oled.c', 'car_ultra.c',
]

def file_block(name):
    return ('            <File>\n'
            '              <FileName>%s</FileName>\n'
            '              <FileType>1</FileType>\n'
            '              <FilePath>..\\..\\Car\\Src\\%s</FilePath>\n'
            '            </File>\n' % (name, name))

car_group = ('        <Group>\n'
             '          <GroupName>Car</GroupName>\n'
             '          <Files>\n'
             + ''.join(file_block(f) for f in CAR_FILES) +
             '          </Files>\n'
             '        </Group>\n')

anchor = '        <Group>\n          <GroupName>App</GroupName>\n'
if anchor in s:
    s = s.replace(anchor, car_group + anchor, 1)
    print('  inserted Car group before App group')
else:
    print('  ERROR: App group anchor not found')
    sys.exit(1)

# ---------- 3. 加头文件路径 ----------
old_inc = '../../App/Inc;../../CSL/Inc;../../System/Inc'
new_inc = '../../App/Inc;../../CSL/Inc;../../System/Inc;../../Car/Inc'
if new_inc not in s:
    if old_inc in s:
        s = s.replace(old_inc, new_inc, 1)
        print('  include path += ../../Car/Inc')
    else:
        print('  ERROR: IncludePath pattern not found')

open(P, 'wb').write(s.encode('utf-8'))
print('written ->', P)
