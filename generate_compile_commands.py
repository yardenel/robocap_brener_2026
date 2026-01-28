import os
import subprocess


def main(*arg, **kwargs):
    subprocess.run(["pio", "run", "-t", "compiledb"])


if __name__ == "__main__":
    main()
