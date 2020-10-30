idf_component_register(SRCS "main.c"
                            "maple.c"
                    INCLUDE_DIRS ".")
idf_build_set_property(COMPILE_DEFINITIONS "-DBLUERETRO" APPEND)
