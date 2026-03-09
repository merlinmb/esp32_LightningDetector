Import("env")
import os

# Arduino ESP32 3.x moved WiFi's base class into a separate "Network" library.
# PlatformIO doesn't auto-detect this transitive dependency, so add it manually.
framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
if framework_dir:
    network_path = os.path.join(framework_dir, "libraries", "Network", "src")
    env.Append(CPPPATH=[network_path])

# PlatformIO only compiles sources from src/, not include/.
# AS3935SPI.cpp lives in include/, so we add it to the build explicitly.
env.BuildSources(
    os.path.join("$BUILD_DIR", "include_srcs"),
    os.path.join("$PROJECT_DIR", "include"),
    "+<*.cpp>"
)
