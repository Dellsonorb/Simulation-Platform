#!/usr/bin/env python3

import sys

from ground_manipulator_runtime.renderer import RenderError, render_ground_robot


def main(argv=None, stdout=None, stderr=None):
    arguments = tuple(sys.argv[1:] if argv is None else argv)
    output = sys.stdout if stdout is None else stdout
    errors = sys.stderr if stderr is None else stderr
    if len(arguments) != 1:
        errors.write("ground-render: expected one xacro path\n")
        return 64
    try:
        payload = render_ground_robot(arguments[0])
    except (RenderError, OSError, ValueError) as error:
        errors.write("ground-render: %s\n" % error)
        return 65
    output.write(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
