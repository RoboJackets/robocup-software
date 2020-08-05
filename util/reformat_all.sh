if ! command -v clang-tidy-10 &> /dev/null
then
    echo "clang-tidy-10 could not be found. Install it by running"
    echo "    sudo apt install clang-tidy-10"
    exit
fi

if ! command -v clang-apply-replacements-10 &> /dev/null
then
    echo "clang-apply-replacements-10 could not be found. Install it by running"
    echo "    sudo apt install clang-tidy-10"
    exit
fi

if ! command -v clang-format-10 &> /dev/null
then
    echo "clang-format-10 could not be found. Install it by running"
    echo "    sudo apt install clang-format-10"
    exit
fi

if ! command -v cmake-format &> /dev/null
then
    echo "cmake-format could not be found. Install it by running"
    echo "    pip3 install cmake-format"
    exit
fi

echo "Filtering precompiled headers in build/compile_commands.json..."
sed -i 's/-include [^ ]*cmake_pch\.hxx//' build/compile_commands.json

echo "Running run-clang-tidy..."
python3 util/run-clang-tidy.py \
  -clang-tidy-binary clang-tidy-10 \
  -clang-apply-replacements-binary clang-apply-replacements-10 \
  -checks="-*,readability-identifier-naming" \
  -p build \
  -fix \
  "^(?!build)(?!external).*$"

echo "Running run-clang-format..."
python3 util/run-clang-format.py \
  -clang-format-binary clang-format-10 \
  -i \
  -p build \
  "^(?!build)(?!external).*$"

echo "Running run-cmake-format..."
python3 util/run-cmake-format.py \
  -i \
  "^(?!cmake-)(?!build)(?!external).*$"
