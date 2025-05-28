@echo off
echo "current directory: %cd%"
echo "copy ECycle_G070B8.hex to current directory"
cd ..\firmware
del /Q .\*
echo "current directory: %cd%"
copy ..\MDK-ARM\ECycle_G070B8\ECycle_G070B8.hex .\
set hexfile=.\ECycle_G070B8.hex
echo %hexfile%
..\firmwaretools\hex2bin.exe %hexfile%

REM 获取 .bin 文件的大小
set binfile=.\ECycle_G070B8.bin
for %%I in (%binfile%) do set filesize=%%~zI

REM 输出 .bin 文件的大小
echo "ECycle_G070B8.bin size: %filesize% bytes"

..\firmwaretools\file_crc32 .\ECycle_G070B8.bin
