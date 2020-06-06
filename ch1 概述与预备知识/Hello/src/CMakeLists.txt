add_library(HELLO SHARED
    hello.cpp
)

target_link_libraries(HELLO
    ${THIRD_PARTY_LIBS}
)
# TARGET_LINK_LIBRARIES(HELLO libhello.so)