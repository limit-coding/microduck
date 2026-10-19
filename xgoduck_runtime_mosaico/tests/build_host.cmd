@echo off
set "PATH=C:\Windows\System32;C:\Windows"
call "G:\Software\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /LD /O2 /fp:precise /DDUCK_HOST_TEST /I main main/duck_core.c main/mosaico_imu_raw.c main/mosaico_attitude.c main/policy.c main/policy_data.c /Fe:tests/duck_host.dll /Fo:tests/
exit /b %errorlevel%
