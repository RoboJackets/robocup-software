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
