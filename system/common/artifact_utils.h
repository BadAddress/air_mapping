#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace apollo {
namespace air_mapping {

std::string CurrentIso8601Utc();
std::string YamlQuote(const std::string& value);
void WriteYamlString(std::ofstream& file, const std::string& key,
                     const std::string& value);
void WriteYamlStringList(std::ofstream& file, const std::string& key,
                         const std::vector<std::string>& values);
bool PrepareCleanOutputDirectory(const std::filesystem::path& path,
                                 const std::string& label);

}  // namespace air_mapping
}  // namespace apollo
