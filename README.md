# Build and Run Instructions

## Prerequisites

- **IDE**: CLion 2025.2.3 (or compatible version)
- **Compiler**:
    - Linux: GCC 9.0+ or Clang 10.0+
    - macOS: Xcode Command Line Tools (Clang)
    - Windows: MinGW-w64 or MSVC 2019+
- **CMake**: Version 3.15 or higher
- **SFML**: Version 2.5+ (for graphics)

---

## Linux

### Build
```bash
# Clone the repository
git clone https://github.com/Penguin5681/CPhysicsEngine.git
cd CPhysicsEngine

# Install dependencies (Ubuntu/Debian)
sudo apt-get install libsfml-dev cmake build-essential

# Create build directory
mkdir build && cd build

# Generate build files and compile
cmake ..
make
```

### Run
```bash
./CPhysicsEngine
```

---

## macOS

### Build
```bash
# Install dependencies via Homebrew
brew install sfml cmake

# Clone and navigate to repository
git clone https://github.com/Penguin5681/CPhysicsEngine.git
cd CPhysicsEngine

# Create build directory
mkdir build && cd build

# Generate build files and compile
cmake ..
make
```

### Run
```bash
./CPhysicsEngine
```

---

## Windows

### Build (Using CLion)
1. Install MinGW-w64 or MSVC
2. Install SFML (download from [sfml-dev.org](https://www.sfml-dev.org))
3. Open CLion and select **File → Open** → Select the project folder
4. CLion will automatically detect CMakeLists.txt
5. Configure CMake with SFML path if needed
6. Click **Build → Build Project** or press `Ctrl+F9`

### Build (Command Line with MinGW)
```cmd
# Clone repository
git clone https://github.com/Penguin5681/CPhysicsEngine.git
cd CPhysicsEngine

# Create build directory
mkdir build
cd build

# Generate and build
cmake -G "MinGW Makefiles" ..
mingw32-make
```

### Run
```cmd
CPhysicsEngine.exe
```

---

## IDE Setup (CLion 2025.2.3)

1. Open CLion 2025.2.3
2. Select **File → Open** and choose the `CPhysicsEngine` directory
3. CLion will automatically load the CMake project
4. Configure toolchain: **File → Settings → Build, Execution, Deployment → Toolchains**
5. Build configuration: **Build → Build Project**
6. Run configuration: Click the ▶️ button or press `Shift+F10`

---
