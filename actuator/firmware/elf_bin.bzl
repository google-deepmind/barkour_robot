"""
A rule to strip off elf headers and make a directly loadable .bin file using objcopy.
"""

load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain", "use_cpp_toolchain")
load("@pw_toolchain//actions:providers.bzl", "ActionNameInfo")

def _elf_bin(ctx):
    cc_toolchain = find_cpp_toolchain(ctx)

    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    objcopy_path = cc_common.get_tool_for_action(
        feature_configuration = feature_configuration,
        action_name = ctx.attr._objcopy[ActionNameInfo].name,
    )

    ctx.actions.run_shell(
        inputs = depset(
            direct = [ctx.files.elf_input[0]],
            transitive = [
                cc_toolchain.all_files,
            ],
        ),
        outputs = [ctx.outputs.bin_out],
        command = "{objcopy} {args} {input} {output}".format(
            objcopy = objcopy_path,
            args = "-Obinary",
            input = ctx.files.elf_input[0].path,
            output = ctx.outputs.bin_out.path,
        ),
    )

    return DefaultInfo(
        files = depset([ctx.outputs.bin_out] + ctx.files.elf_input[1:]),
    )

elf_bin = rule(
    implementation = _elf_bin,
    attrs = {
        "bin_out": attr.output(mandatory = True),
        "elf_input": attr.label(mandatory = True),
        "_objcopy": attr.label(
            default = "@pw_toolchain//actions:objcopy_embed_data",
            providers = [ActionNameInfo],
        ),
    },
    toolchains = use_cpp_toolchain(),
    fragments = ["cpp"],
)
