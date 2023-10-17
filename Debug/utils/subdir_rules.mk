################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
utils/RunTimeStatsConfig.obj: ../utils/RunTimeStatsConfig.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/portable/CCS/ARM_CM4F" --define=ccs="ccs" --define=TARGET_IS_BLIZZARD_RB1 --define=UART_BUFFERED --define=WANT_CMDLINE_HISTORY --define=WANT_FREERTOS_SUPPORT --define=PART_TM4C123GH6PM --define=DEBUG -g --c89 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="utils/RunTimeStatsConfig.d_raw" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/cmdline.obj: ../utils/cmdline.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/portable/CCS/ARM_CM4F" --define=ccs="ccs" --define=TARGET_IS_BLIZZARD_RB1 --define=UART_BUFFERED --define=WANT_CMDLINE_HISTORY --define=WANT_FREERTOS_SUPPORT --define=PART_TM4C123GH6PM --define=DEBUG -g --c89 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="utils/cmdline.d_raw" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/cpu_usage.obj: ../utils/cpu_usage.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/portable/CCS/ARM_CM4F" --define=ccs="ccs" --define=TARGET_IS_BLIZZARD_RB1 --define=UART_BUFFERED --define=WANT_CMDLINE_HISTORY --define=WANT_FREERTOS_SUPPORT --define=PART_TM4C123GH6PM --define=DEBUG -g --c89 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="utils/cpu_usage.d_raw" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/uartstdio.obj: ../utils/uartstdio.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/include" --include_path="D:/Post-Graduation/Embedded Systems/EUI Internship/Technical/projects/EUI_EME_ARM13_FREERTOS_QUEUE/EUI_EME_ARM13_FREERTOS_QUEUE/FreeRTOS/Source/portable/CCS/ARM_CM4F" --define=ccs="ccs" --define=TARGET_IS_BLIZZARD_RB1 --define=UART_BUFFERED --define=WANT_CMDLINE_HISTORY --define=WANT_FREERTOS_SUPPORT --define=PART_TM4C123GH6PM --define=DEBUG -g --c89 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="utils/uartstdio.d_raw" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


