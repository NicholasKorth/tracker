@echo off
set PATH=C:\ncs\toolchains\66cdf9b75e\opt\bin;C:\ncs\toolchains\66cdf9b75e\mingw64\bin;C:\ncs\toolchains\66cdf9b75e\bin;C:\ncs\toolchains\66cdf9b75e\opt\bin\Scripts;%PATH%
set ZEPHYR_BASE=C:\ncs\v3.2.1\zephyr
cd /d C:\ncs\v3.2.1
C:\ncs\toolchains\66cdf9b75e\opt\bin\python.exe -m west build --pristine --no-sysbuild -b nrf21540dk/nrf52840 c:\Users\korth\peripheral_uart -d c:\Users\korth\peripheral_uart\build
