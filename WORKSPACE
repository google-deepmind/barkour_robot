load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

git_repository(
  name = "pigweed",
  commit = "c4cf469a53bed014cbe797a93aec971536e68f86",
  remote = "https://pigweed.googlesource.com/pigweed/pigweed.git",
)

git_repository(
    name = "rules_python",
    commit = "e06b4bae446706db3414e75d301f56821001b554",
    remote = "https://github.com/bazelbuild/rules_python.git",
)

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

# Use Python 3.11 for bazel Python rules.
python_register_toolchains(
    name = "python3",
    # Allows building as root in a docker container. Required by oss-fuzz.
    ignore_root_user_error = True,
    python_version = "3.11",
)

load("@python3//:defs.bzl", "interpreter")
load("@rules_python//python:pip.bzl", "pip_parse")

git_repository(
    name = "python_packages_pygments",
    commit = "d7d11f6e6d3aa97805215c1cc833ea5f0ef1fcbb",
    remote = "https://github.com/pygments/pygments.git",
)

git_repository(
    name = "python_packages_ptpython",
    commit = "fb9bed1e5956ac5f109fd4cb401b3fae997efcd7",
    remote = "https://github.com/prompt-toolkit/ptpython.git",
)

git_repository(
    name = "python_packages_pyyaml",
    commit = "c42fa3bff1eabdb64763bb1526d9ea1ccb708479",
    remote = "https://github.com/yaml/pyyaml.git",
)

git_repository(
    name = "python_packages_prompt_toolkit",
    commit = "6a58564f6a201f1234733f726d866a64e95b6ba3",
    remote = "https://github.com/prompt-toolkit/python-prompt-toolkit.git",
)

git_repository(
    name = "python_packages_websockets",
    commit = "01195322d2620a44039b716cb93c108c2ca9b6b9",
    remote = "https://github.com/python-websockets/websockets.git",
)

git_repository(
    name = "python_packages_jinja2",
    commit = "dd4a8b5466d8790540c181590b14db4d4d889d57",
    remote = "https://github.com/pallets/jinja.git",
)

http_archive(
    name = "python_packages_pyperclip",
    sha256 = "105254a8b04934f0bc84e9c24eb360a591aaf6535c9def5f29d92af107a9bf57",
    strip_prefix = "pyperclip-1.8.2",
    url = "https://files.pythonhosted.org/packages/a7/2c/4c64579f847bd5d539803c8b909e54ba087a79d01bb3aba433a95879a6c5/pyperclip-1.8.2.tar.gz",
)

http_archive(
    name = "rules_license",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_license/releases/download/0.0.8/rules_license-0.0.8.tar.gz",
        "https://github.com/bazelbuild/rules_license/releases/download/0.0.8/rules_license-0.0.8.tar.gz",
    ],
    sha256 = "241b06f3097fd186ff468832150d6cc142247dc42a32aaefb56d0099895fd229",
)

http_archive(
    name = "platforms",
    sha256 = "8150406605389ececb6da07cbcb509d5637a3ab9a24bc69b1101531367d89d74",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/platforms/releases/download/0.0.8/platforms-0.0.8.tar.gz",
        "https://github.com/bazelbuild/platforms/releases/download/0.0.8/platforms-0.0.8.tar.gz",
    ],
)

# Fuzzing
http_archive(
    name = "bazel_skylib",  # 2022-09-01
    sha256 = "4756ab3ec46d94d99e5ed685d2d24aece484015e45af303eb3a11cab3cdc2e71",
    strip_prefix = "bazel-skylib-1.3.0",
    urls = ["https://github.com/bazelbuild/bazel-skylib/archive/refs/tags/1.3.0.zip"],
)

http_archive(
    name = "rules_proto",
    sha256 = "dc3fb206a2cb3441b485eb1e423165b231235a1ea9b031b4433cf7bc1fa460dd",
    strip_prefix = "rules_proto-5.3.0-21.7",
    urls = [
        "https://github.com/bazelbuild/rules_proto/archive/refs/tags/5.3.0-21.7.tar.gz",
    ],
)

load("@rules_python//python:repositories.bzl", "py_repositories")

py_repositories()

http_archive(
    name = "com_google_protobuf",
    sha256 = "616bb3536ac1fff3fb1a141450fa28b875e985712170ea7f1bfe5e5fc41e2cd8",
    strip_prefix = "protobuf-24.4",
    url = "https://github.com/protocolbuffers/protobuf/releases/download/v24.4/protobuf-24.4.tar.gz",
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

http_archive(
    name = "com_github_nanopb_nanopb",
    sha256 = "3f78bf63722a810edb6da5ab5f0e76c7db13a961c2aad4ab49296e3095d0d830",
    strip_prefix = "nanopb-0.4.8",
    url = "https://github.com/nanopb/nanopb/archive/refs/tags/0.4.8.tar.gz",
)

load("@com_github_nanopb_nanopb//extra/bazel:nanopb_deps.bzl", "nanopb_deps")

nanopb_deps()

load("@com_github_nanopb_nanopb//extra/bazel:python_deps.bzl", "nanopb_python_deps")

nanopb_python_deps(interpreter)

load("@com_github_nanopb_nanopb//extra/bazel:nanopb_workspace.bzl", "nanopb_workspace")

nanopb_workspace()

git_repository(
    name = "rules_fuzzing",
    commit = "67ba0264c46c173a75825f2ae0a0b4b9b17c5e59",
    remote = "https://github.com/bazelbuild/rules_fuzzing",
)

load("@rules_fuzzing//fuzzing:repositories.bzl", "rules_fuzzing_dependencies")

rules_fuzzing_dependencies()

load("@rules_fuzzing//fuzzing:init.bzl", "rules_fuzzing_init")

rules_fuzzing_init()

load("@fuzzing_py_deps//:requirements.bzl", fuzzing_install_deps = "install_deps")

fuzzing_install_deps()

git_repository(
    name = "pw_toolchain",
    commit = "c4cf469a53bed014cbe797a93aec971536e68f86",
    remote = "https://pigweed.googlesource.com/pigweed/pigweed.git",
    strip_prefix = "pw_toolchain_bazel",
)

# Set up CIPD.
load(
    "@pigweed//pw_env_setup/bazel/cipd_setup:cipd_rules.bzl",
    "cipd_client_repository",
    "cipd_repository",
)

cipd_client_repository()

# Fetch llvm toolchain.
cipd_repository(
    name = "linux_clang_toolchain",
    build_file = "@pw_toolchain//build_external:llvm_clang.BUILD",
    path = "fuchsia/third_party/clang/${os}-${arch}",
    tag = "git_revision:c58bc24fcf678c55b0bf522be89eff070507a005",
)

cipd_repository(
    name = "linux_sysroot",
    path = "fuchsia/third_party/sysroot/bionic",
    tag = "git_revision:702eb9654703a7cec1cadf93a7e3aa269d053943",
)

http_archive(
    name = "gcc_arm_none_eabi_toolchain",
    urls = ["https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2?revision=ca0cbf9c-9de2-491c-ac48-898b5bbc0443&rev=ca0cbf9c9de2491cac48898b5bbc0443&hash=72D7BCC38C586E3FE39D2A1DB133305C64CA068B"],
    strip_prefix = "gcc-arm-none-eabi-10-2020-q4-major",
    build_file = "//actuator/firmware/toolchains/arm_toolchain:arm_toolchain.BUILD.bazel",
)

register_toolchains(
  "//actuator/firmware/toolchains/linux_clang_toolchain:host_cc_toolchain_linux",
  "//actuator/firmware/toolchains/arm_toolchain:arm_gcc_cc_toolchain_cortex-m4",
  "//actuator/firmware/toolchains/arm_toolchain:arm_gcc_cc_toolchain_cortex-m7",
)

http_archive(
    name = "freertos",
    build_file = "@pigweed//third_party/freertos:freertos.BUILD.bazel",
    sha256 = "89af32b7568c504624f712c21fe97f7311c55fccb7ae6163cda7adde1cde7f62",
    strip_prefix = "FreeRTOS-Kernel-10.5.1",
    urls = ["https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/refs/tags/V10.5.1.tar.gz"],
)

git_repository(
  name = "soes",
  commit = "591d08d474e8bcdca21562b23f00555567a5cb2c",
  remote = "https://github.com/OpenEtherCATsociety/SOES.git",
  build_file = "//actuator/firmware/soes:soes.BUILD.bazel",
)

git_repository(
  name = "stm32cubeh7",
  commit = "abbf9ca1b36c9abdb9339e86f0b7b2f4e79edc35",
  remote = "https://github.com/STMicroelectronics/STM32CubeH7.git",
  build_file = "//:actuator/firmware/stm32cubeh7/stm32cubeh7.BUILD.bazel",
  recursive_init_submodules = True,
)

git_repository(
  name = "mbedtls",
  commit = "2ca6c285a0dd3f33982dd57299012dacab1ff206",
  remote = "https://github.com/Mbed-TLS/mbedtls.git",
  build_file = "//actuator/firmware/mbedtls:mbedtls.BUILD.bazel",
)

git_repository(
  name = "com_google_absl_py",
  remote = "https://github.com/abseil/abseil-py",
  tag = "v2.1.0",
)
