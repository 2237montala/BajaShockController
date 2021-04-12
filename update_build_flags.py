# Custom settings, as referred to as "extra_script" in platformio.ini
#
# See http://docs.platformio.org/en/latest/projectconf.html#extra-script

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

env.Append(
    # CCFLAGS=[
    #     "-mthumb",
    #     #"-mcpu=%s" % env.BoardConfig().get("build.cpu"),
    #     "-mcpu=cortex-m4",
    #     "-mfloat-abi=hard",
    #     "-mfpu=fpv4-sp-d16"
    # ],

    LINKFLAGS=[
        "-mthumb",
        "-mfloat-abi=soft",
        "-u_printf_float"
    ]
)
