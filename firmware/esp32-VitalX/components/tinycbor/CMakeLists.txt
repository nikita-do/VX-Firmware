idf_component_register(SRCS "src/cborencoder.c"
                            "src/cborencoder_close_container_checked.c"
                            "src/cborerrorstrings.c"
                            "src/cborparser.c"
                            "src/cborparser_dup_string.c"
                            "src/cborpretty.c"
                            "src/cbortojson.c"
                            "src/cborvalidation.c"
                    INCLUDE_DIRS "src")