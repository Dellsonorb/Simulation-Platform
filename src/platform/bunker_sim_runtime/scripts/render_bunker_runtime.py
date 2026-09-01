#!/usr/bin/env python3
import sys

from bunker_sim_runtime.renderer import (
    RenderContractError, render_runtime_urdf)


def main(argv=None, stdout=None, stderr=None):
    arguments = tuple(sys.argv[1:] if argv is None else argv)
    output = sys.stdout if stdout is None else stdout
    errors = sys.stderr if stderr is None else stderr
    if len(arguments) != 1:
        errors.write("bunker-render: expected one xacro path\n")
        return 64
    try:
        rendered = render_runtime_urdf(arguments[0])
    except (RenderContractError, OSError, ValueError) as error:
        errors.write("bunker-render: %s\n" % error)
        return 65
    output.write(rendered)
    return 0


if __name__ == "__main__":
    sys.exit(main())
