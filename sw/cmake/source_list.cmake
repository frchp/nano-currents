message("source_list.cmake")

set(source_list ${source_list}
${PROJ_PATH}/vendor/system_stm32l0xx.c

${PROJ_PATH}/src/main.c
${PROJ_PATH}/src/startup_stm32l011f4px.s
${PROJ_PATH}/src/syscalls.c
)
