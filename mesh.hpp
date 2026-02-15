#pragma once
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <vector>

// --- Source mesh type (for import) ---
struct Traits3 : public OpenMesh::DefaultTraits {
    using Point = OpenMesh::Vec3d; // importer-friendly (xyz only)
    VertexAttributes(OpenMesh::Attributes::Normal  | OpenMesh::Attributes::Status);
    FaceAttributes  (OpenMesh::Attributes::Normal  | OpenMesh::Attributes::Status);
    EdgeAttributes  (OpenMesh::Attributes::Status);
    HalfedgeAttributes(OpenMesh::Attributes::Status);
};
using Mesh3  = OpenMesh::TriMesh_ArrayKernelT<Traits3>;
using Vec3   = OpenMesh::Vec3d;

// --- Destination mesh type (clip space) ---
struct Traits4 : public OpenMesh::DefaultTraits {
    using Point = OpenMesh::Vec4d; // clip-space (x,y,z,w)
    VertexAttributes(OpenMesh::Attributes::Normal  | OpenMesh::Attributes::Status);
    FaceAttributes  (OpenMesh::Attributes::Normal  | OpenMesh::Attributes::Status);
    EdgeAttributes  (OpenMesh::Attributes::Status);
    HalfedgeAttributes(OpenMesh::Attributes::Status);
};
using Mesh4  = OpenMesh::TriMesh_ArrayKernelT<Traits4>;
using Vec4   = OpenMesh::Vec4d;
using VertexHandle   = Mesh4::VertexHandle;
using FaceHandle     = Mesh4::FaceHandle;
using HalfedgeHandle = Mesh4::HalfedgeHandle;

// Import a mesh from disk (OBJ/PLY/…) into Mesh4, setting w=1 for every vertex.
// Returns true on success.
inline bool load_mesh_as_clip_w1(const std::string& path, Mesh4& out, OpenMesh::IO::Options opts = {})
{
    Mesh3 in;
    if (!OpenMesh::IO::read_mesh(in, path, opts)) {
        return false;
    }

    out.clear();
    out.request_vertex_status();
    out.request_edge_status();
    out.request_face_status();
    out.request_halfedge_status();

    // Map source vertex idx -> dest VertexHandle
    std::vector<Mesh4::VertexHandle> mapVH(in.n_vertices());

    // 1) Copy vertices (upgrade to Vec4 with w=1)
    for (auto v_it = in.vertices_begin(); v_it != in.vertices_end(); ++v_it) {
        const auto vh_in = *v_it;
        const Vec3 p3 = in.point(vh_in);
        const Vec4 p4(p3[0], p3[1], p3[2], 1.0);
        mapVH[vh_in.idx()] = out.add_vertex(p4);
    }

    // (Optional) Copy vertex normals if present
    if (opts.check(OpenMesh::IO::Options::VertexNormal) && in.has_vertex_normals() && out.has_vertex_normals()) {
        for (auto v_it = in.vertices_begin(); v_it != in.vertices_end(); ++v_it) {
            auto vh_in = *v_it;
            out.set_normal(mapVH[vh_in.idx()], in.normal(vh_in));
        }
    }

    // 2) Copy faces
    for (auto f_it = in.faces_begin(); f_it != in.faces_end(); ++f_it) {
        std::vector<Mesh4::VertexHandle> tri;
        tri.reserve(3);
        for (auto vh_in : in.fv_range(*f_it)) {
            tri.push_back(mapVH[vh_in.idx()]);
        }
        if (tri.size() == 3) {
            auto fh = out.add_face(tri);
            if (!fh.is_valid()) {
                // Try flipped winding if importer created opposite orientation
                std::swap(tri[1], tri[2]);
                (void)out.add_face(tri);
            }
        } else if (tri.size() > 3) {
            // In case the source still has n-gons (shouldn’t for TriMesh), do a simple fan.
            for (size_t i = 1; i + 1 < tri.size(); ++i) {
                std::vector<Mesh4::VertexHandle> f = { tri[0], tri[i], tri[i+1] };
                auto fh = out.add_face(f);
                if (!fh.is_valid()) { std::swap(f[1], f[2]); (void)out.add_face(f); }
            }
        }
    }

    out.garbage_collection(true, true, true); // clean any failed faces/isolated verts
    return true;
}

