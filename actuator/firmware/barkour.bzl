"""Constants and transitions used in firmware build files."""

HARDWARE_VERSION = [
    "GEBRU_VERSION=4",
    "CORTES_VERSION=5",
    "ENCODER_RLS_AKSIM2=1",
    "ENCODER_MA732_ADA=2",
    "ENCODER_TYPE=2",
    "MOTOR_AK80_9=105",
    "MOTOR_TYPE=105",
]

def _m4_transition_impl(settings, attr):
    # buildifier: disable=unused-variable
    _ignore = attr
    current_features = settings.get("//command_line_option:features", default = [])
    if "mcpu_cortex_m4" not in current_features:
        new_features = current_features + ["mcpu_cortex_m4"]
    else:
        new_features = current_features

    return {
        # You need speed-optimized builds or the firmware won't work.
        "//command_line_option:compilation_mode": "opt",
        # Select the appropriate toolchain features (-mcpu flag).
        "//command_line_option:features": new_features,
        "//command_line_option:platforms": "//actuator/firmware/targets/m4:platform",
        "@pigweed//pw_assert:backend": "@pigweed//pw_assert_basic",
        "@pigweed//pw_assert:backend_impl": "@pigweed//pw_assert_basic:impl",
        "@pigweed//pw_sys_io:backend": "//actuator/firmware/barkour/m4:sys_io_impl",
        "@pigweed//pw_log:backend": "@pigweed//pw_log_string",
        "@pigweed//pw_log_string:handler_backend": "@pigweed//pw_system:log_backend",

        # FreeRTOS configuration
        "@freertos//:freertos_config": "//actuator/firmware/targets/m4:freertos_config",

        # SOES configuration
        "@soes//:ecat_options": "//actuator/firmware/barkour/m4:ecat_options",
        "@soes//:objectlist": "//actuator/firmware/barkour/m4:objectlist",
        "@soes//:cc_config": "//actuator/firmware/barkour/m4:cc_config",

        # STM32Cube HAL configuration
        "@stm32cubeh7//:hal_config": "//actuator/firmware/targets/m4:hal_config",

        # Mbed-TLS configuration
        "@mbedtls//:mbedtls_config": "//actuator/firmware/targets/m4:mbedtls_config",
    }

_m4_transition = transition(
    implementation = _m4_transition_impl,
    inputs = [],
    outputs = [
        "//command_line_option:compilation_mode",
        "//command_line_option:features",
        "//command_line_option:platforms",
        "@pigweed//pw_assert:backend",
        "@pigweed//pw_assert:backend_impl",
        "@pigweed//pw_sys_io:backend",
        "@pigweed//pw_log:backend",
        "@pigweed//pw_log_string:handler_backend",

        # FreeRTOS configuration
        "@freertos//:freertos_config",

        # SOES configuration
        "@soes//:ecat_options",
        "@soes//:objectlist",
        "@soes//:cc_config",

        # STM32Cube HAL configuration
        "@stm32cubeh7//:hal_config",

        # Mbed-TLS configuration
        "@mbedtls//:mbedtls_config",
    ],
)

def _binary_impl(ctx):
    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.symlink(output = out, target_file = ctx.executable.binary)
    return [DefaultInfo(files = depset([out]), executable = out)]

m4_binary = rule(
    _binary_impl,
    attrs = {
        "binary": attr.label(
            doc = "cc_binary to build for M4",
            cfg = _m4_transition,
            executable = True,
            mandatory = True,
        ),
    },
)

def _m7_transition_impl(settings, attr):
    # buildifier: disable=unused-variable
    _ignore = attr
    current_features = settings.get("//command_line_option:features", default = [])
    if "mcpu_cortex_m7" not in current_features:
        new_features = current_features + ["mcpu_cortex_m7"]
    else:
        new_features = current_features

    return {
        # You need speed-optimized builds or the firmware won't work.
        "//command_line_option:compilation_mode": "opt",
        # Select the appropriate toolchain features (-mcpu flag).
        "//command_line_option:features": new_features,
        "//command_line_option:platforms": "//actuator/firmware/targets/m7:platform",
        "@pigweed//pw_assert:backend": "@pigweed//pw_assert_basic",
        "@pigweed//pw_assert:backend_impl": "@pigweed//pw_assert_basic:impl",
        "@pigweed//pw_sys_io:backend": "//actuator/firmware/barkour/m7:sys_io_impl",

        # FreeRTOS configuration
        "@freertos//:freertos_config": "//actuator/firmware/targets/m7:freertos_config",

        # STM32Cube HAL configuration
        "@stm32cubeh7//:hal_config": "//actuator/firmware/targets/m7:hal_config",
    }

_m7_transition = transition(
    implementation = _m7_transition_impl,
    inputs = [],
    outputs = [
        "//command_line_option:compilation_mode",
        "//command_line_option:features",
        "//command_line_option:platforms",
        "@pigweed//pw_assert:backend",
        "@pigweed//pw_assert:backend_impl",
        "@pigweed//pw_sys_io:backend",

        # FreeRTOS configuration
        "@freertos//:freertos_config",

        # STM32Cube HAL configuration
        "@stm32cubeh7//:hal_config",
    ],
)

m7_binary = rule(
    _binary_impl,
    attrs = {
        "binary": attr.label(
            doc = "cc_binary to build for M7",
            cfg = _m7_transition,
            executable = True,
            mandatory = True,
        ),
    },
)
