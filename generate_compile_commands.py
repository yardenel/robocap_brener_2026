import os
import subprocess


def before_build(source, target, env):
    subprocess.run(["pio", "run", "-t", "compiledb"])
