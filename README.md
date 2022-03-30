Alternate PIN option need to be changed for SPI NSS. To do this find old laptop with Windows OS and install here ST Visual Programmer.
STM32CubeProg which replaces that Visual Programmer seems to not support STM8 family boards.
Connect STM8S via ST LINK, select board in Visual Programmer, load current settings, change *AFR1* option to **... SPI_NSS ...**,
then push settings into STM8S.
For more detailed info please look at:
 - https://blog.mark-stevens.co.uk/2012/11/stm8s-spi-slave-device/
 - https://blog.mark-stevens.co.uk/2012/11/stm8s-spi-slave-part-2/
 - https://blog.mark-stevens.co.uk/2014/07/stm8s-beep-function/
 - https://github.com/huochaigunkeji/STM8S003-SPI-Slave

And about running ST Visual Programmer on Mac M1, it worked on Windows 11 ARM running inside UTM, the only thing was to... disable
signature checks. Detailed info about installing unsigned drivers: https://community.st.com/s/question/0D53W000011vaXZSAY/stlink-stcubeprogrammer-support-on-windows-arm64

