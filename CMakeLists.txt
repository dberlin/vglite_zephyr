if (CONFIG_VGLITE)
    set(VGLITE_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

    zephyr_library()

    zephyr_include_directories(${VGLITE_ROOT_DIR}/inc
            ${VGLITE_ROOT_DIR}/font
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder
            ${VGLITE_ROOT_DIR}/VGLite/rtos
            ${VGLITE_ROOT_DIR}/VGLiteKernel
            ${VGLITE_ROOT_DIR}/VGLiteKernel/rtos
            ${VGLITE_ROOT_DIR}/elementary/inc
            ${VGLITE_ROOT_DIR}/elementary/src)
    zephyr_library_compile_definitions(
            -DVG_COMMAND_CALL=1
            -DVG_TARGET_FAST_CLEAR=0
            -DRTOS=1
            -DVG_RENDER_TEXT=1
    )
    zephyr_library_sources(${VGLITE_ROOT_DIR}/VGLite/vg_lite.c
            ${VGLITE_ROOT_DIR}/VGLite/vg_lite_image.c
            ${VGLITE_ROOT_DIR}/VGLite/vg_lite_matrix.c
            ${VGLITE_ROOT_DIR}/VGLite/vg_lite_path.c
            ${VGLITE_ROOT_DIR}/VGLite/nxp_support.c
            ${VGLITE_ROOT_DIR}/VGLite/vg_lite_flat.c
            ${VGLITE_ROOT_DIR}/VGLite/rtos/vg_lite_os_zephyr.c
            ${VGLITE_ROOT_DIR}/font/buf_reader.c
            ${VGLITE_ROOT_DIR}/font/rle_font_read.c
            ${VGLITE_ROOT_DIR}/font/vft_debug.c
            ${VGLITE_ROOT_DIR}/font/vft_draw.c
            ${VGLITE_ROOT_DIR}/font/vg_lite_text.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_bwfont.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_encoding.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_font.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_justify.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_kerning.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_rlefont.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_scaledfont.c
            ${VGLITE_ROOT_DIR}/font/mcufont/decoder/mf_wordwrap.c
            ${VGLITE_ROOT_DIR}/VGLiteKernel/vg_lite_kernel.c
            ${VGLITE_ROOT_DIR}/VGLiteKernel/rtos/vg_lite_hal.c
            ${VGLITE_ROOT_DIR}/elementary/src/elm_buffer.c
            ${VGLITE_ROOT_DIR}/elementary/src/elm_draw.c
            ${VGLITE_ROOT_DIR}/elementary/src/elm_init.c
            ${VGLITE_ROOT_DIR}/elementary/src/elm_object.c
            ${VGLITE_ROOT_DIR}/elementary/src/elm_os.c
            ${VGLITE_ROOT_DIR}/elementary/src/elm_text.c)
endif ()