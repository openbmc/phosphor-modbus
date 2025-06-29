#include <cstdint>
#include <filesystem>
#include <format>
#include <iostream> // For std::cout
#include <optional>
#include <regex>
#include <string> // For std::string
#include <vector> // For std::vector

auto getUSBDevicePath(const std::string& deviceAddress, uint8_t interface,
                      uint8_t port) -> std::optional<std::string>
{
    namespace fs = std::filesystem;
    std::regex pattern(std::format("platform-{}\\.usb-usb.*{}-port{}",
                                   deviceAddress, interface, port));
    fs::path searchDir = "/dev/serial/by-path/";

    for (const auto& entry : fs::recursive_directory_iterator(searchDir))
    {
        if (entry.is_symlink())
        {
            auto filePath = entry.path();
            if (std::regex_search(filePath.filename().string(), pattern))
            {
                return ("/dev/" +
                        fs::read_symlink(filePath).filename().string());
            }
        }
    }

    return std::nullopt;
}

int main()
{
    auto res = getUSBDevicePath("1e6a1000", 0, 0);
    if (res.has_value())
    {
        std::cout << "Found USB path:" << res.value() << std::endl;
    }
    else
    {
        std::cout << "Found no US Path" << std::endl;
    }

    return 0;
}
