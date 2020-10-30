idf_component_register(SRCS "main.c"
                            "adapter/adapter.c"
                            "adapter/config.c"
                            "adapter/dc.c"
                            "wired/maple.c"
                            "zephyr/atomic.S"
                            "zephyr/xtensa_context.S"
                    INCLUDE_DIRS "."
                                 "adapter"
                                 "wired"
                                 "zephyr")
idf_build_set_property(COMPILE_DEFINITIONS "-DBLUERETRO" APPEND)
