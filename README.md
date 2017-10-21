A minimal port of SuperPixelator to Cpp:
https://bitbucket.org/piffany/superpixelator

Paper:
https://dl.acm.org/citation.cfm?id=2486044

The relevant portion that converts svg paths to pixels has been ported. Other
shapes are currently not supported. But it should be relatively easy to add
those as well by porting the missing parts from the original Java library.

Deps:

CMake

libpng

Eigen

Usage:

`./super_pix <path_to_svg_file> <path_to_png_output_file>`
