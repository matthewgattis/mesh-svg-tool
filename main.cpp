#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <optional>

#ifndef OM_STATIC_BUILD
#define OM_STATIC_BUILD
#endif
#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_RADIANS
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/trigonometric.hpp>
#include <glm/detail/qualifier.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "spdlog/spdlog.h"
#include "argparse/argparse.hpp"
#include "bvh_openmesh.hpp"
#include "clip_openmesh.hpp"
#include "mesh.hpp"

using Polyline2D = std::vector<glm::dvec2>;

void render_svg(
    const std::vector<Polyline2D> &polylines,
    const std::string &svg_filename,
    double width,
    double height,
    const std::string &units,
    double stroke_width)
{
    std::ofstream ofs(svg_filename);

    // Start SVG
    ofs << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << units << "\" height=\"" << height << units << "\" viewBox=\"0 0 " << width << " " << height << "\">" << std::endl;

    for (const auto &polyline : polylines)
    {
        ofs << "<polyline points=\"";
        for (const auto &p0 : polyline)
        {
            ofs <<  p0[0] << "," << -p0[1] << " ";
        }
        ofs << "\" style=\"fill:none;stroke:black;stroke-width:" << stroke_width << "\" />" << std::endl;
    }

    ofs << "</svg>" << std::endl;
}

std::vector<Polyline2D> get_polylines(Mesh4 &clip_mesh, double step)
{
    Mesh4 &mesh = clip_mesh;

    OpenMesh::EPropHandleT<bool> visited;
    mesh.add_property(visited);
    for (auto e : mesh.edges())
        mesh.property(visited, e) = false;

    const auto a = [&](
        const HalfedgeHandle &heh) -> std::optional<HalfedgeHandle>
    {
        std::optional<HalfedgeHandle> heh3;

        auto vh = mesh.to_vertex_handle(heh);
        auto p0 = mesh.point(mesh.from_vertex_handle(heh));
        auto p1 = mesh.point(vh);
        auto v0 = (p1 - p0).normalize();

        double c = std::numeric_limits<double>::lowest();
        for (auto heh2 : mesh.voh_range(vh))
        {
            if (mesh.property(visited, mesh.edge_handle(heh2)))
                continue;

            auto p2 = mesh.point(mesh.to_vertex_handle(heh2));
            auto v1 = (p2 - p1).normalize();

            // Dot product
            double d = v0 | v1;

            if (d > c)
            {
                c = d;
                heh3 = heh2;
            }
        }

        return heh3;
    };

    std::vector<std::vector<HalfedgeHandle>> halfedge_polylines;
    for (const auto e : mesh.edges())
    {
        if (mesh.property(visited, e))
            continue;

        halfedge_polylines.push_back({});

        std::optional<HalfedgeHandle> heh = mesh.halfedge_handle(e, 0);

        do
        {
            halfedge_polylines.back().push_back(heh.value());

            mesh.property(visited, mesh.edge_handle(heh.value())) = true;
            heh = a(heh.value());

        } while (heh.has_value());
    }

    for (auto &v : mesh.vertices())
    {
        auto p = mesh.point(v);
        p /= p[3];
        mesh.set_point(v, p);
    }

    BVH bvh = BVHBuilder::build(mesh);

    std::vector<Polyline2D> polylines;

    for (const auto &halfedge_polyline : halfedge_polylines)
    {
        int pen_state = 0;

        for (const auto &heh : halfedge_polyline)
        {
            const auto &p0 = mesh.point(mesh.from_vertex_handle(heh));
            const auto &p1 = mesh.point(mesh.to_vertex_handle(heh));

            auto d = p1 - p0;
            double l = glm::length(glm::dvec2(d[0], d[1]));

            int n = (int)(l / step);
            if (n < 1)
                n = 1;

            for (int j = 0; j < n; j++)
            {
                double t = (double)j / (double)n;
                auto p = p0 + d * t;

                // Check occlusion
                bool occluded = bvh_raycast_closest(
                    bvh,
                    mesh,
                    RayOM{ .o = { p[0], p[1], p[2] - 1e-9 }, .d = { 0.0, 0.0, -1.0 } }).has_value();

                if (!occluded)
                {
                    if (pen_state == 0)
                    {
                        polylines.push_back({ { p[0], p[1] } });
                        pen_state = 1;
                    }
                    if (pen_state == 1)
                    {
                        polylines.back().push_back({ p[0], p[1] });
                        pen_state = 2;
                    }
                    if (pen_state == 2)
                    {
                        polylines.back().back() = { p[0], p[1] };
                    }
                    if (pen_state == 3)
                    {
                        polylines.back().back() = { p[0], p[1] };
                        pen_state = 1;
                    }
                }
                else
                {
                    pen_state = 0;
                }
            }

            if (pen_state == 2)
                pen_state = 3;
        }
    }

    return polylines;
}

void transform_mesh(
    Mesh4 &mesh,
    const glm::dmat4 &rotation,
    double distance,
    double fovy,
    double width,
    double height,
    bool do_back_face_culling,
    bool debug = false)
{
    for (const auto &vh : mesh.vertices())
    {
        auto pos = mesh.point(vh);
        glm::dvec4 p0 = rotation * glm::dvec4(pos[0], pos[1], pos[2], 1.0);
        mesh.set_point(vh, { p0.x, p0.y, p0.z, p0.w });
    }

    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "rotated.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("rotated.ply saved");
    }

    AABB bounds;
    for (const auto &vh : mesh.vertices())
    {
        const auto &p = mesh.point(vh);
        bounds.expand({ p[0], p[1], p[2] });
    }

    const auto half_extent = bounds.extent() / 2.0;
    double bounds_aspect = half_extent[0] / half_extent[1];

    const auto center = bounds.center();
    glm::dmat4 t =
        glm::translate(glm::dmat4(1.0), { -center[0], -center[1] , -center[2] - distance });

    for (const auto &vh : mesh.vertices())
    {
        auto pos = mesh.point(vh);
        glm::dvec4 p0 = t * glm::dvec4(pos[0], pos[1], pos[2], 1.0);
        mesh.set_point(vh, { p0.x, p0.y, p0.z, p0.w });
    }

    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "translated.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("translated.ply saved");
    }

    const double znear = -(bounds.bmax[2] - center[2] - distance) - 1e-9;
    const double zfar = -(bounds.bmin[2] - center[2] - distance) + 1e-9;

    spdlog::info("znear: {}, zfar: {}", znear, zfar);

    const double aspect = width / height;

    glm::dmat4 p = glm::perspective(glm::radians(fovy), aspect, znear, zfar);

    for (const auto &vh : mesh.vertices())
    {
        auto pos = mesh.point(vh);
        glm::dvec4 p0 = p * glm::dvec4(pos[0], pos[1], pos[2], pos[3]);
        mesh.set_point(vh, { p0.x, p0.y, p0.z, p0.w });
    }

    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "projected.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("projected.ply saved");
    }

    // Back-face culling in clip space
    if (do_back_face_culling)
    {
        mesh.request_face_status();
        mesh.request_vertex_status();
        mesh.request_edge_status();

        for (auto &f : mesh.faces())
        {
            auto it = mesh.fv_range(f).begin();
            const Vec4 &p0 = mesh.point(*it); ++it;
            const Vec4 &p1 = mesh.point(*it); ++it;
            const Vec4 &p2 = mesh.point(*it);
            glm::dvec2 a = glm::dvec2(p1[0], p1[1]) / p1[3] - glm::dvec2(p0[0], p0[1]) / p0[3];
            glm::dvec2 b = glm::dvec2(p2[0], p2[1]) / p2[3] - glm::dvec2(p0[0], p0[1]) / p0[3];
            double c = a.x * b.y - a.y * b.x; // 2D cross product
            if (c < 0.0)
                mesh.delete_face(f);
        }

        mesh.garbage_collection();
    }

    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "culled.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("culled.ply saved");
    }

    clip_mesh_inplace(mesh, ClipConv::OpenGL, 1e-14);

    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "clipped.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("clipped.ply saved");
    }

    // NDC transform
    for (const auto &vh : mesh.vertices())
    {
        auto pos = mesh.point(vh);
        if (glm::abs(pos[3]) > 1e-30)
        {
            pos /= pos[3];
            mesh.set_point(vh, pos);
        }
    }

    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "ndc.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("ndc.ply saved");
    }

    glm::dmat4 viewport = glm::dmat4(
        width / 2.0, 0.0, 0.0, 0.0,
        0.0, height / 2.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        width / 2.0, -height / 2.0, 0.0, 1.0);

    for (const auto &vh : mesh.vertices())
    {
        auto pos = mesh.point(vh);
        glm::dvec4 p0 = viewport * glm::dvec4(pos[0], pos[1], pos[2], pos[3]);
        mesh.set_point(vh, { p0.x, p0.y, p0.z, p0.w });
    }

    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "viewport.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("viewport.ply saved");
    }
}

void run(const std::vector<std::string> &args)
{
    argparse::ArgumentParser program("mesh-svg-tool");

    program.add_argument("mesh_filename")
        .help("mesh file to load")
        .required();

    program.add_argument("--svg")
        .help("output file svg")
        .default_value(std::string("output.svg"));

    // Fovy (degrees).
    program.add_argument("--fovy")
        .help("field of view in degrees")
        .default_value(30.0)
        .scan<'g', double>();

    program.add_argument("--width")
        .help("output image width")
        .default_value(210.0)
        .scan<'g', double>();

    program.add_argument("--height")
        .help("output image height")
        .default_value(297.0)
        .scan<'g', double>();

    program.add_argument("--units")
        .help("units for width and height (e.g., mm, cm, in, px)")
        .default_value(std::string("mm"));

    // Translation away from camera (along -Z axis in view space).
    program.add_argument("--distance")
        .help("distance to translate the model away from the camera (along -Z axis in view space)")
        .default_value(10.0)
        .scan<'g', double>();

    // Angle (degrees) and axis (x, y, z) of rotation.
    program.add_argument("--angle-axis")
        .help("rotate the model by angle (degrees) and axis (x, y, z)")
        .nargs(4)
        .default_value(std::vector<double>{ 0.0, 1.0, 0.0, 0.0 })
        .scan<'g', double>();

    program.add_argument("--no-hidden-surface-elimination")
        .help("disable hidden surface elimination")
        .default_value(false)
        .implicit_value(true);

    program.add_argument("--hse-step")
        .help("step size for hidden surface elimination")
        .default_value(0.01)
        .scan<'g', double>();

    program.add_argument("--do-back-face-culling")
        .help("enable back-face culling")
        .default_value(false)
        .implicit_value(true);

    program.add_argument("--stroke-width")
        .help("stroke width in units")
        .default_value(0.5)
        .scan<'g', double>();

    program.add_argument("--debug")
        .help("enable debug output")
        .default_value(false)
        .implicit_value(true);

    program.parse_args(args);

    const std::string mesh_filename = program.get<std::string>("mesh_filename");
    const std::string svg_filename = program.get<std::string>("--svg");
    const double fovy = program.get<double>("--fovy");
    const double width = program.get<double>("--width");
    const double height = program.get<double>("--height");
    const std::string units = program.get<std::string>("--units");
    const double distance = program.get<double>("--distance");
    const auto angle_axis = program.get<std::vector<double>>("--angle-axis");
    const bool no_hidden_surface_elimination = program.get<bool>("--no-hidden-surface-elimination");
    const double hse_step = program.get<double>("--hse-step");
    const bool do_back_face_culling = program.get<bool>("--do-back-face-culling");
    const double stroke_width = program.get<double>("--stroke-width");
    const bool debug = program.get<bool>("--debug");

    spdlog::info("mesh_filename: {}", mesh_filename);
    spdlog::info("svg_filename: {}", svg_filename);
    spdlog::info("fovy: {}", fovy);
    spdlog::info("width: {} {}", width, units);
    spdlog::info("height: {} {}", height, units);
    spdlog::info("units: {}", units);
    spdlog::info("distance: {}", distance);
    spdlog::info("angle-axis: angle {} axis {} {} {}", angle_axis[0], angle_axis[1], angle_axis[2], angle_axis[3]);
    spdlog::info("no_hidden_surface_elimination: {}", no_hidden_surface_elimination);
    spdlog::info("hse_step: {}", hse_step);
    spdlog::info("do_back_face_culling: {}", do_back_face_culling);
    spdlog::info("stroke_width: {}", stroke_width);
    spdlog::info("debug: {}", debug);

    const auto q = glm::angleAxis(
        glm::radians(angle_axis[0]),
        glm::normalize(glm::dvec3(angle_axis[1], angle_axis[2], angle_axis[3])));
    const glm::dmat4 rotation = glm::mat4_cast(q);

    Mesh4 mesh;

    if (!load_mesh_as_clip_w1(mesh_filename, mesh))
        throw std::runtime_error("mesh load error");

    spdlog::info("mesh loaded");

    // Write original mesh
    if (debug)
    {
        if (!OpenMesh::IO::write_mesh(mesh, "original.ply"))
            throw std::runtime_error("mesh write error");

        spdlog::info("original.ply saved");
    }

    transform_mesh(
        mesh,
        rotation,
        distance,
        fovy,
        width,
        height,
        do_back_face_culling,
        debug);

    auto polylines = get_polylines(mesh, hse_step);

    render_svg(polylines, svg_filename, width, height, units, stroke_width);
}

int main(int argc, char **argv)
{
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++)
        args.push_back(argv[i]);

    try
    {
        run(args);
    }
    catch (const std::exception &e)
    {
        spdlog::error("exception: {}", e.what());
        return EXIT_FAILURE;
    }

    spdlog::info("done");

    return EXIT_SUCCESS;
}

