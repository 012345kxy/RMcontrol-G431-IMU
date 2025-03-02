make -j6
openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg -c "program build/RMCONTROL-G431.bin 0x08000000 verify" -c "reset run" -c "exit"
