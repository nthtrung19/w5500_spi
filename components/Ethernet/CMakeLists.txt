idf_component_register(
    SRCS "W5500/w5500.c"
         "wizchip_conf.c"
         "socket.c"
         # Thêm các file source khác nếu có, ví dụ: "loopback.c"
    INCLUDE_DIRS "W5500" "."
    REQUIRES driver
)