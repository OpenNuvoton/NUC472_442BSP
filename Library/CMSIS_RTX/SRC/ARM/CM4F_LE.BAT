SET PATH=C:\Keil\ARM\BIN40;C:\Program Files\ARM\ADSv1_2\bin;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;c:\Program Files\Microsoft SQL Server\90\Tools\binn\;C:\Program Files\doxygen\bin;C:\Program Files\TortoiseSVN\bin;C:\Program Files\TortoiseGit\bin
SET ARMCC41INC=C:\Keil\ARM\RV31\Inc
SET ARMCC41LIB=C:\Keil\ARM\RV31\Lib
SET CPU_TYPE=Cortex-M4
SET CPU_VENDOR=ARM
SET UV2_TARGET=CM4F_LE
SET CPU_CLOCK=0x00B71B00
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_cmsis.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_task.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_system.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_event.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_list.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_mailbox.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_semaphore.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_time.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_timer.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_mutex.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_robin.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_membox.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\rt_memory.__i"
"C:\Keil\ARM\BIN40\ArmAsm" --Via ".\cm4f_le\svc_table._ia"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\hal_cm.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\cm4f_le\hal_cm4.__i"
"C:\Keil\ARM\BIN40\ArmAr" --Via "ArInp.Scr"
cmd.exe /C copy ArInp.Scr CM4F_LE\BuildLib.scr
cmd.exe /C copy CM4F_LE\RTX_CM4.lib ..\..\LIB\ARM
