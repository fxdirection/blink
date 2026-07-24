idf_component_register(SRCS "motor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        pid
                        tb6612
                    )
