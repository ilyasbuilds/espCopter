# Apply patches/dshot_rmt_allow_zero.patch to derdoktor667/DShotRMT (oldAPI).
# Allows throttle 0 (motor stop / disarm) required by AM32.
from os.path import isfile, join
from pathlib import Path

Import("env")

PATCH_FILE = join(env["PROJECT_DIR"], "patches", "dshot_rmt_allow_zero.patch")
FLAG_NAME = ".dshot_rmt_allow_zero_patched"

libdeps = Path(env["PROJECT_LIBDEPS_DIR"]) / env["PIOENV"]
if not libdeps.is_dir():
    print("patch_dshot_rmt: libdeps not ready yet; skip")
else:
    assert isfile(PATCH_FILE), f"Missing patch file: {PATCH_FILE}"
    for cpp in libdeps.rglob("DShotRMT.cpp"):
        # oldAPI keeps DShotRMT.cpp at the library root; skip newer layouts
        if cpp.parent.name != "DShotRMT":
            continue
        lib_dir = cpp.parent
        flag_path = lib_dir / FLAG_NAME
        if flag_path.is_file():
            print(f"patch_dshot_rmt: already patched {lib_dir}")
            continue
        # Idempotent if the user already edited the installed copy
        if "throttle_value < DSHOT_THROTTLE_MIN && throttle_value != 0" in cpp.read_text():
            flag_path.write_text("")
            print(f"patch_dshot_rmt: change already present in {lib_dir}")
            continue
        env.Execute(f"patch -d {lib_dir} -p1 < {PATCH_FILE}")
        flag_path.write_text("")
        print(f"patch_dshot_rmt: patched {lib_dir}")
