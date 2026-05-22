#include "modules/air_mapping/system/common/artifact_utils.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <system_error>

#include "cyber/common/log.h"

namespace apollo {
namespace air_mapping {

std::string CurrentIso8601Utc() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);
  std::tm utc_time;
#if defined(_WIN32)
  gmtime_s(&utc_time, &seconds);
#else
  gmtime_r(&seconds, &utc_time);
#endif

  std::ostringstream stream;
  stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

std::string YamlQuote(const std::string& value) {
  std::string result = "\"";
  for (const char c : value) {
    if (c == '\\' || c == '"') {
      result.push_back('\\');
    }
    result.push_back(c);
  }
  result.push_back('"');
  return result;
}

void WriteYamlString(std::ofstream& file, const std::string& key,
                     const std::string& value) {
  file << key << ": " << YamlQuote(value) << "\n";
}

void WriteYamlStringList(std::ofstream& file, const std::string& key,
                         const std::vector<std::string>& values) {
  file << key << ":\n";
  for (const auto& value : values) {
    file << "  - " << YamlQuote(value) << "\n";
  }
}

bool PrepareCleanOutputDirectory(const std::filesystem::path& path,
                                 const std::string& label) {
  std::error_code error;
  if (std::filesystem::exists(path, error)) {
    std::filesystem::remove_all(path, error);
    if (error) {
      AERROR << "Failed to clear old " << label
             << " artifacts: " << path.string()
             << ", error: " << error.message();
      return false;
    }
  }

  std::filesystem::create_directories(path, error);
  if (error) {
    AERROR << "Failed to create " << label << " output directory: "
           << path.string() << ", error: " << error.message();
    return false;
  }
  return true;
}

}  // namespace air_mapping
}  // namespace apollo
