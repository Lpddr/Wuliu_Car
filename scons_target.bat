cd /d F:\SoftWare\IDE\RT-ThreadStudio\workspace\Wuliu_Car
set RTT_ROOT=
set ENV_ROOT=F:/SoftWare/IDE/RT-ThreadStudio/platform/env_released/env
set RTT_EXEC_PATH=F:/SoftWare/IDE/RT-ThreadStudio/repo/Extract/ToolChain_Support_Packages/ARM/GNU_Tools_for_ARM_Embedded_Processors/5.4.1/bin
set KCONF_EXEC_PATH=%ENV_ROOT%\tools\exec
set RTT_CC=gcc
set RTTS_PLATFORM=STUDIO
set SCONS=%ENV_ROOT%\tools\Python27\Scripts
set PKGS_ROOT=%ENV_ROOT%\packages
set PATH=%ENV_ROOT%\tools\MinGit-2.25.1-32-bit\cmd;%PATH%
set PATH=%ENV_ROOT%\tools\Python27;%PATH%
set PATH=%ENV_ROOT%\tools\Python27\Scripts;%PATH%
set PATH=%ENV_ROOT%\tools\bin;%PATH%
set PATH=%RTT_EXEC_PATH%;%PATH%
set PATH=%KCONF_EXEC_PATH%;%PATH%
set PATH=%ENV_ROOT%\tools\scripts\pkgs;%PATH%
set PATH=%ENV_ROOT%\tools\qemu\qemu32;%PATH%
set PYTHONPATH=%ENV_ROOT%\tools\Python27;
set PATH=%PATH%;
scons --target=eclipse --project-name=Wuliu_Car
