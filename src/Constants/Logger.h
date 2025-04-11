#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <sstream>
#include <string>

using std::cout;
using std::string;
using std::ostringstream;
using std::endl;
enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class Logger {
   public:
    static LogLevel level;

    static void log(LogLevel logLevel, const string& message) {
        if (static_cast<int>(logLevel) >= static_cast<int>(level)) {
            string prefix;
            switch (logLevel) {
                case LogLevel::DEBUG:
                    // prefix = "[DEBUG] ";
                    prefix = "";
                    break;
                case LogLevel::INFO:
                    prefix = "[INFO] ";
                    break;
                case LogLevel::WARN:
                    prefix = "\n[WARN] ";
                    break;
                case LogLevel::ERROR:
                    prefix = "[ERROR] ";
                    break;
            }
            cout << prefix << message << endl;
        }
    }
};

/// @brief Macros that can be used for logging.
#define LOG_DEBUG(msg)                           \
    {                                            \
        ostringstream oss;                       \
        oss << msg;                              \
        Logger::log(LogLevel::DEBUG, oss.str()); \
    }
#define LOG_INFO(msg)                           \
    {                                           \
        ostringstream oss;                      \
        oss << msg;                             \
        Logger::log(LogLevel::INFO, oss.str()); \
    }
#define LOG_WARN(msg)                           \
    {                                           \
        ostringstream oss;                      \
        oss << msg;                             \
        Logger::log(LogLevel::WARN, oss.str()); \
    }
#define LOG_ERROR(msg)                           \
    {                                            \
        ostringstream oss;                       \
        oss << msg;                              \
        Logger::log(LogLevel::ERROR, oss.str()); \
    }

#endif