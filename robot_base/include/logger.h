#include <iostream>

#define LOG_DEBUG(...) (std::cout << "[DEBUG] " << __VA_ARGS__ << std::endl)
#define LOG_INFO(...)  (std::cout << "[INFO] "  << __VA_ARGS__ << std::endl)
#define LOG_WARN(...)  (std::cerr << "[WARN] "  << __VA_ARGS__ << std::endl)
#define LOG_ERROR(...) (std::cerr << "[ERROR] " << __VA_ARGS__ << std::endl)