#ifndef LOGGER_H
#define LOGGER_H

#include <cstdio>
#include <sstream>
#include <string>

using std::string;

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class Logger {
   public:
    static LogLevel level;

    static void log(LogLevel logLevel, const string& message) {
        if (static_cast<int>(logLevel) >= static_cast<int>(level)) {
            string prefix;
            switch (logLevel) {
                case LogLevel::DEBUG:
                    prefix = "[DEBUG] ";
                    break;
                case LogLevel::INFO:
                    prefix = "[INFO] ";
                    break;
                case LogLevel::WARN:
                    prefix = "[WARN] ";
                    break;
                case LogLevel::ERROR:
                    prefix = "[ERROR] ";
                    break;
            }
            printf("%s%s\n", prefix.c_str(), message.c_str());
        }
    }
};

/// @brief Macros that can be used for logging.
#define LOG_DEBUG(msg)                           \
    {                                            \
        std::ostringstream oss;                  \
        oss << msg;                              \
        Logger::log(LogLevel::DEBUG, oss.str()); \
    }
#define LOG_INFO(msg)                           \
    {                                           \
        std::ostringstream oss;                 \
        oss << msg;                             \
        Logger::log(LogLevel::INFO, oss.str()); \
    }
#define LOG_WARN(msg)                           \
    {                                           \
        std::ostringstream oss;                 \
        oss << msg;                             \
        Logger::log(LogLevel::WARN, oss.str()); \
    }
#define LOG_ERROR(msg)                           \
    {                                            \
        std::ostringstream oss;                  \
        oss << msg;                              \
        Logger::log(LogLevel::ERROR, oss.str()); \
    }

#endif