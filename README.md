# mesh-svg-tool

Render mesh as wireframe to SVG. Made for pen plotting meshes with hidden surface (edge) elemination.

![](docs/teapot.svg)
*Utah Teapot wireframe. [Inkscape](https://inkscape.org/) modified output. See [examples/output.svg](examples/output.svg) for tool output.*

![](docs/DSC07810-3.jpg)
*Utah Teapot, pen on paper. Draw by [Creality Ender-3 V3 KE.](https://www.creality.com/products/creality-ender-3-v3-ke) Image edited in [Adobe Lightroom.](https://lightroom.adobe.com/)*

## Getting Started

### Prequisites

- CMake https://cmake.org/
- vcpkg https://github.com/microsoft/vcpkg
    - Manually clone and bootstrap vcpkg according to instructions.
    - https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-cmd

### Building (\*NIX)

``` sh
$ git clone https://github.com/matthewgattis/mesh-svg-tool.git
$ cd cl-renderer/
```

- Adjust `tools/configure.sh` as needed.

``` sh
$ mkdir build/
$ cd build/
$ ../tools/configure.sh ../
$ cmake --build . -j
```

### Render Example Image

``` sh
$ wget https://raw.githubusercontent.com/gnomeby/canvas3D/refs/heads/master/teapot.ply -O teapot.ply
$ ./mesh-svg-tool --fovy 30 --angle-axis -60 1 0 0 --width 1280.0 --height 720.0 --distance 10 --units "px" --stroke-width 1 teapot.ply
```

### More

- Use `--help` for more information:

```
$ ./mesh-svg-tool --help
Usage: mesh-svg-tool [--help] [--version] [--svg VAR] [--fovy VAR] [--width VAR] [--height VAR] [--units VAR] [--distance VAR] [--angle-axis VAR...] [--no-hidden-surface-elimination] [--hse-step VAR] [--do-back-face-culling] [--stroke-width VAR] [--debug] mesh_filename

Positional arguments:
  mesh_filename                    mesh file to load [required]

Optional arguments:
  -h, --help                       shows help message and exits 
  -v, --version                    prints version information and exits 
  --svg                            output file svg [nargs=0..1] [default: "output.svg"]
  --fovy                           field of view in degrees, 0 for orthographic [nargs=0..1] [default: 30]
  --width                          output image width [nargs=0..1] [default: 210]
  --height                         output image height [nargs=0..1] [default: 297]
  --units                          units for width and height (e.g., mm, cm, in, px) [nargs=0..1] [default: "mm"]
  --distance                       distance to translate the model away from the camera (along -Z axis in view space) [nargs=0..1] [default: 10]
  --angle-axis                     rotate the model by angle (degrees) and axis (x, y, z) [nargs=0..4] [default: {0 1 0 0}]
  --no-hidden-surface-elimination  disable hidden surface elimination 
  --hse-step                       step size for hidden surface elimination [nargs=0..1] [default: 0.01]
  --do-back-face-culling           enable back-face culling 
  --stroke-width                   stroke width in units [nargs=0..1] [default: 0.5]
  --debug                          enable debug output
```