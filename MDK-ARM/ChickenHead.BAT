SET PATH=D:\MDK5\ARM\ARMCC\Bin;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Program Files\dotnet\;C:\Users\LZ\AppData\Local\Microsoft\WindowsApps;C:\Users\LZ\AppData\Local\GitHubDesktop\bin
SET CPU_TYPE=STM32F427IIHx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=ChickenHead
SET CPU_CLOCK=0x0A037A00
"D:\MDK5\ARM\ARMCC\Bin\ArmAsm" --Via "..\obj\startup_stm32f427xx._ia"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_can.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_tim.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_tim_ex.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_uart.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_rcc.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_rcc_ex.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_flash.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_flash_ex.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_flash_ramfunc.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_gpio.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_dma_ex.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_dma.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_pwr.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_pwr_ex.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_cortex.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_hal_spi.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\stm32f4xx_it.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\system_stm32f4xx.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\main.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\can_ctrl.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\control_loop.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\datascope_dp.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\motor_ctrl.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\remote_ctrl.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\shoot_ctrl.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\imu.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\adrc.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\ins.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\pc_uart.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\referee_comm.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\crc.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\sys.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\delay.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\can.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\dma.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\gpio.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\tim.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\usart.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\spi.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\ahrs_middleware.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\ist8310driver.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\ist8310driver_middleware.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\mpu6500driver.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmCC" --Via "..\obj\mpu6500driver_middleware.__i"
"D:\MDK5\ARM\ARMCC\Bin\ArmLink" --Via "..\OBJ\ChickenHead.lnp"
"D:\MDK5\ARM\ARMCC\Bin\fromelf.exe" "..\OBJ\ChickenHead.axf" --i32combined --output "..\OBJ\ChickenHead.hex"
