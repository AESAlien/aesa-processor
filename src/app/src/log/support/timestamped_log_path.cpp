#include "timestamped_log_path.hpp"

#include <cerrno>
#include <ctime>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace app
{
std::filesystem::path reserveTimestampedLogPath(
    const std::filesystem::path& basePath,
    std::chrono::system_clock::time_point started
) {
    if (basePath.filename().empty())
    {
        throw std::invalid_argument("Log path must include a filename");
    }

    const auto milliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(started);
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(milliseconds);
    const auto time = std::chrono::system_clock::to_time_t(seconds);
    std::tm localTime{};
#ifdef _WIN32
    const auto timeError = localtime_s(&localTime, &time);
    if (timeError != 0)
    {
        throw std::system_error(timeError, std::generic_category(), "Cannot format log timestamp");
    }
#else
    if (localtime_r(&time, &localTime) == nullptr)
    {
        throw std::system_error(errno, std::generic_category(), "Cannot format log timestamp");
    }
#endif

    std::ostringstream timestamp;
    timestamp.imbue(std::locale::classic());
    timestamp << std::put_time(&localTime, "%Y%m%d_%H%M%S")
              << '_' << std::setfill('0') << std::setw(3) << (milliseconds - seconds).count();
    const auto stem = basePath.stem().string() + '_' + timestamp.str();
    const auto extension = basePath.extension().string();
    if (basePath.has_parent_path())
    {
        std::filesystem::create_directories(basePath.parent_path());
    }

    for (std::size_t index = 0; ; ++index)
    {
        const auto suffix = index == 0 ? std::string{} : '_' + std::to_string(index);
        const auto path = basePath.parent_path() / (stem + suffix + extension);
#ifdef _WIN32
        const auto descriptor = _wopen(
            path.c_str(), _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY, _S_IREAD | _S_IWRITE
        );
#else
        const auto descriptor = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0666);
#endif
        if (descriptor == -1)
        {
            const auto error = errno;
            if (error == EEXIST)
            {
                continue;
            }
            throw std::filesystem::filesystem_error(
                "Cannot reserve log file", path, std::error_code(error, std::generic_category())
            );
        }
#ifdef _WIN32
        const auto result = _close(descriptor);
#else
        const auto result = close(descriptor);
#endif
        if (result != 0)
        {
            const std::error_code error(errno, std::generic_category());
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
            throw std::filesystem::filesystem_error("Cannot close reserved log file", path, error);
        }
        return path;
    }
}
}
