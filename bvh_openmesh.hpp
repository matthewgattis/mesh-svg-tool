// Written by OpenAI ChatGPT based on user instructions.
//
#pragma once

// ================================ C++ std =================================
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>
#include <cmath>

#include "mesh.hpp"

// ================================ Helpers =================================
inline bool clip_to_ndc(const Vec4& P, Vec3& out, double w_eps = 1e-30) {
    if (std::abs(P[3]) < w_eps) return false;
    const double invw = 1.0 / P[3];
    out = Vec3(P[0]*invw, P[1]*invw, P[2]*invw);
    return true;
}

struct AABB {
    Vec3 bmin = Vec3(+std::numeric_limits<double>::infinity(),
                 +std::numeric_limits<double>::infinity(),
                 +std::numeric_limits<double>::infinity());
    Vec3 bmax = Vec3(-std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity());

    void expand(const Vec3& p) {
        bmin[0] = std::min(bmin[0], p[0]); bmax[0] = std::max(bmax[0], p[0]);
        bmin[1] = std::min(bmin[1], p[1]); bmax[1] = std::max(bmax[1], p[1]);
        bmin[2] = std::min(bmin[2], p[2]); bmax[2] = std::max(bmax[2], p[2]);
    }
    void expand(const AABB& b) { expand(b.bmin); expand(b.bmax); }

    Vec3 extent() const {
        return Vec3(std::max(0.0, bmax[0]-bmin[0]),
                  std::max(0.0, bmax[1]-bmin[1]),
                  std::max(0.0, bmax[2]-bmin[2]));
    }
    int longest_axis() const {
        Vec3 e = extent();
        if (e[0] >= e[1] && e[0] >= e[2]) return 0;
        if (e[1] >= e[2]) return 1;
        return 2;
    }
    Vec3 center() const {
        return 0.5 * (bmin + bmax);
    }
};
inline bool aabb_overlap(const AABB& a, const AABB& b) {
    return (a.bmin[0] <= b.bmax[0] && a.bmax[0] >= b.bmin[0]) &&
           (a.bmin[1] <= b.bmax[1] && a.bmax[1] >= b.bmin[1]) &&
           (a.bmin[2] <= b.bmax[2] && a.bmax[2] >= b.bmin[2]);
}
inline AABB bounds_of_triangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    AABB box; box.expand(a); box.expand(b); box.expand(c); return box;
}
inline Vec3 centroid_of_triangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    return (a + b + c) / 3.0;
}

// ================================ BVH core ================================
struct BVHNode {
    AABB box;               // NDC-space AABB
    uint32_t left  = 0;     // internal: left child index; leaf: start in triIndices
    uint32_t right = 0;     // internal: right child index; leaf: unused
    uint16_t triCount = 0;  // >0 => leaf
    uint16_t _pad = 0;
    bool is_leaf() const { return triCount > 0; }
};

struct BVH {
    std::vector<BVHNode> nodes;                       // nodes[0] is root
    std::vector<uint32_t> triIndices;                 // leaf content (indices)
    std::vector<std::array<uint32_t,3>> triVerts;     // tri -> vertex indices (OpenMesh indices)
    std::vector<FaceHandle> faces;                    // tri -> face handle

    // Cached NDC data per triangle
    std::vector<std::array<Vec3,3>> triNDC;             // tri vertices in NDC
    std::vector<AABB>             triAABBs;           // tri AABBs in NDC
};

// ================================ Builder =================================
struct BuildItem {
    AABB box;
    Vec3   centroid;
    uint32_t triIndex;
};
struct BuildParams { uint32_t leafMaxTris = 4; };

class BVHBuilder {
public:
    static BVH build(const Mesh4& mesh, BuildParams params = {}) {
        BVHBuilder b(mesh, params); b.run(); return std::move(b.bvh_);
    }
private:
    const Mesh4& mesh_;
    BuildParams params_;
    BVH bvh_;
    std::vector<BuildItem> items_;

    BVHBuilder(const Mesh4& m, BuildParams p) : mesh_(m), params_(p) {}

    void gather_triangles() {
        const size_t F = mesh_.n_faces();
        bvh_.triVerts.resize(F);
        bvh_.faces.resize(F);
        bvh_.triNDC.resize(F);
        bvh_.triAABBs.resize(F);
        items_.resize(F);
        bvh_.triIndices.clear();

        uint32_t ti = 0;
        for (auto f_it = mesh_.faces_begin(); f_it != mesh_.faces_end(); ++f_it, ++ti) {
            FaceHandle fh = *f_it;
            bvh_.faces[ti] = fh;

            // Gather the 3 vertex indices for this triangle
            int k = 0; std::array<uint32_t,3> tv{};
            for (auto vh : mesh_.fv_range(fh)) tv[k++] = static_cast<uint32_t>(vh.idx());
            bvh_.triVerts[ti] = tv;

            // Convert clip -> NDC for the triangle
            Vec3 A,B,C;
            const Vec4 a = mesh_.point(VertexHandle(tv[0]));
            const Vec4 b = mesh_.point(VertexHandle(tv[1]));
            const Vec4 c = mesh_.point(VertexHandle(tv[2]));
            const bool okA = clip_to_ndc(a, A);
            const bool okB = clip_to_ndc(b, B);
            const bool okC = clip_to_ndc(c, C);

            if (!(okA && okB && okC)) {
                // Mark as "invalid" via sentinel (won't overlap any normal query AABB)
                A = B = C = Vec3(1e300, 1e300, 1e300);
            }

            bvh_.triNDC[ti]   = {A,B,C};
            bvh_.triAABBs[ti] = bounds_of_triangle(A,B,C);

            items_[ti].triIndex = ti;
            items_[ti].box      = bvh_.triAABBs[ti];
            items_[ti].centroid = centroid_of_triangle(A,B,C);
        }
    }

    uint32_t alloc_node() { bvh_.nodes.push_back({}); return uint32_t(bvh_.nodes.size() - 1); }

    uint32_t build_node(uint32_t start, uint32_t count) {
        const uint32_t ni = alloc_node();

        // Node bounds
        AABB nodeBox;
        for (uint32_t i = 0; i < count; ++i) nodeBox.expand(items_[start + i].box);
        bvh_.nodes[ni].box = nodeBox;

        // Leaf?
        if (count <= params_.leafMaxTris) {
            const uint32_t outStart = uint32_t(bvh_.triIndices.size());
            bvh_.triIndices.reserve(outStart + count);
            for (uint32_t i = 0; i < count; ++i)
                bvh_.triIndices.push_back(items_[start + i].triIndex);
            bvh_.nodes[ni].left     = outStart;             // start in triIndices
            bvh_.nodes[ni].triCount = uint16_t(count);
            return ni;
        }

        // Split by centroid-box longest axis, partition, with median fallback
        AABB centroidBox;
        for (uint32_t i = 0; i < count; ++i) centroidBox.expand(items_[start + i].centroid);
        const int axis = centroidBox.longest_axis();
        const double mid = 0.5 * (centroidBox.bmin[axis] + centroidBox.bmax[axis]);

        auto begin = items_.begin() + start;
        auto end   = items_.begin() + start + count;
        auto itMid = std::partition(begin, end, [&](const BuildItem& it){
            return it.centroid[axis] < mid;
        });

        uint32_t leftCount  = uint32_t(itMid - begin);
        uint32_t rightCount = count - leftCount;
        if (leftCount == 0 || rightCount == 0) {
            const uint32_t half = count / 2;
            std::nth_element(begin, begin + half, end, [axis](const BuildItem& a, const BuildItem& b){
                return a.centroid[axis] < b.centroid[axis];
            });
            leftCount = half; rightCount = count - half;
        }

        bvh_.nodes[ni].triCount = 0;
        const uint32_t L = build_node(start, leftCount);
        const uint32_t R = build_node(start + leftCount, rightCount);
        bvh_.nodes[ni].left  = L;
        bvh_.nodes[ni].right = R;
        return ni;
    }

    void run() {
        bvh_.nodes.clear();
        bvh_.triIndices.clear();
        items_.clear();

        gather_triangles();
        if (!bvh_.triVerts.empty())
            (void)build_node(0, uint32_t(bvh_.triVerts.size()));
    }
};

// ===================== SAT (triangle–AABB) in NDC (3D) ====================
inline bool tri_aabb_overlap_sat_3d(const Vec3& A, const Vec3& B, const Vec3& C, const AABB& box)
{
    using OpenMesh::cross; using OpenMesh::dot;
    const Vec3 c = (box.bmin + box.bmax) * 0.5;
    const Vec3 h = (box.bmax - box.bmin) * 0.5;

    const Vec3 v0 = A - c, v1 = B - c, v2 = C - c;

    auto axisOverlap = [&](int a)->bool {
        const double mn = std::min({v0[a], v1[a], v2[a]});
        const double mx = std::max({v0[a], v1[a], v2[a]});
        return !(mn >  h[a] || mx < -h[a]);
    };
    if (!axisOverlap(0) || !axisOverlap(1) || !axisOverlap(2)) return false;

    const Vec3 e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;
    const Vec3 n  = cross(e0, e1);
    {
        const double r = h[0]*std::abs(n[0]) + h[1]*std::abs(n[1]) + h[2]*std::abs(n[2]);
        const double s = dot(n, v0);
        if (std::abs(s) > r) return false;
    }

    auto satAxis = [&](const Vec3& axis)->bool {
        const double len2 = OpenMesh::dot(axis, axis);
        if (len2 < 1e-24) return true;
        const double p0 = OpenMesh::dot(axis, v0);
        const double p1 = OpenMesh::dot(axis, v1);
        const double p2 = OpenMesh::dot(axis, v2);
        const double tmin = std::min({p0, p1, p2});
        const double tmax = std::max({p0, p1, p2});
        const double r = h[0]*std::abs(axis[0]) + h[1]*std::abs(axis[1]) + h[2]*std::abs(axis[2]);
        return !(tmin >  r || tmax < -r);
    };
    const Vec3 ex(1,0,0), ey(0,1,0), ez(0,0,1);
    if (!satAxis(OpenMesh::cross(e0, ex))) return false;
    if (!satAxis(OpenMesh::cross(e0, ey))) return false;
    if (!satAxis(OpenMesh::cross(e0, ez))) return false;
    if (!satAxis(OpenMesh::cross(e1, ex))) return false;
    if (!satAxis(OpenMesh::cross(e1, ey))) return false;
    if (!satAxis(OpenMesh::cross(e1, ez))) return false;
    if (!satAxis(OpenMesh::cross(e2, ex))) return false;
    if (!satAxis(OpenMesh::cross(e2, ey))) return false;
    if (!satAxis(OpenMesh::cross(e2, ez))) return false;

    return true;
}

// ============================ AABB range queries ===========================
inline void bvh_query_aabb_broadphase_ndc( // returns triangle indices
    const BVH& bvh, const AABB& query, std::vector<uint32_t>& out_indices)
{
    out_indices.clear();
    if (bvh.nodes.empty()) return;

    std::vector<uint32_t> stack; stack.reserve(64);
    stack.push_back(0);

    while (!stack.empty()) {
        const uint32_t ni = stack.back(); stack.pop_back();
        const BVHNode& node = bvh.nodes[ni];
        if (!aabb_overlap(node.box, query)) continue;

        if (node.is_leaf()) {
            for (uint32_t i = 0; i < node.triCount; ++i) {
                const uint32_t ti = bvh.triIndices[node.left + i];
                if (aabb_overlap(bvh.triAABBs[ti], query))
                    out_indices.push_back(ti);
            }
        } else {
            stack.push_back(node.right);
            stack.push_back(node.left);
        }
    }
}

inline void bvh_query_aabb_exact_ndc(     // SAT on NDC triangles
    const BVH& bvh, const AABB& query, std::vector<uint32_t>& out_indices)
{
    out_indices.clear();
    if (bvh.nodes.empty()) return;

    std::vector<uint32_t> stack; stack.reserve(64);
    stack.push_back(0);

    while (!stack.empty()) {
        const uint32_t ni = stack.back(); stack.pop_back();
        const BVHNode& node = bvh.nodes[ni];
        if (!aabb_overlap(node.box, query)) continue;

        if (node.is_leaf()) {
            for (uint32_t i = 0; i < node.triCount; ++i) {
                const uint32_t ti = bvh.triIndices[node.left + i];
                if (!aabb_overlap(bvh.triAABBs[ti], query)) continue;
                const auto& T = bvh.triNDC[ti];
                if (tri_aabb_overlap_sat_3d(T[0], T[1], T[2], query))
                    out_indices.push_back(ti);
            }
        } else {
            stack.push_back(node.right);
            stack.push_back(node.left);
        }
    }
}

// =============================== Refit (NDC) ===============================
inline void bvh_recompute_triangle_ndc(BVH& bvh, const Mesh4& mesh) {
    const uint32_t N = uint32_t(bvh.triVerts.size());
    bvh.triNDC.resize(N);
    bvh.triAABBs.resize(N);

    for (uint32_t ti = 0; ti < N; ++ti) {
        const auto& tv = bvh.triVerts[ti];
        const Vec4 a = mesh.point(VertexHandle(tv[0]));
        const Vec4 b = mesh.point(VertexHandle(tv[1]));
        const Vec4 c = mesh.point(VertexHandle(tv[2]));
        Vec3 A,B,C;
        if (!clip_to_ndc(a, A) || !clip_to_ndc(b, B) || !clip_to_ndc(c, C)) {
            A = B = C = Vec3(1e300,1e300,1e300);
        }
        bvh.triNDC[ti]   = {A,B,C};
        bvh.triAABBs[ti] = bounds_of_triangle(A,B,C);
    }
}

inline void bvh_refit_ndc(BVH& bvh) {
    if (bvh.nodes.empty()) return;
    struct Frame { uint32_t idx; uint8_t state; };
    std::vector<Frame> st; st.reserve(bvh.nodes.size());
    st.push_back({0,0});
    while (!st.empty()) {
        auto [ni, s] = st.back(); st.pop_back();
        BVHNode& node = bvh.nodes[ni];
        if (node.is_leaf()) {
            AABB box;
            for (uint32_t i = 0; i < node.triCount; ++i) {
                const uint32_t ti = bvh.triIndices[node.left + i];
                box.expand(bvh.triAABBs[ti]);
            }
            node.box = box;
        } else if (s == 0) {
            st.push_back({ni,1});
            st.push_back({node.right,0});
            st.push_back({node.left,0});
        } else {
            const BVHNode& L = bvh.nodes[node.left];
            const BVHNode& R = bvh.nodes[node.right];
            AABB box = L.box; box.expand(R.box);
            node.box = box;
        }
    }
}

// ============================ Convenience maps ============================
inline std::vector<FaceHandle>
bvh_tris_to_faces(const BVH& bvh, const std::vector<uint32_t>& tri_indices){
    std::vector<FaceHandle> out; out.reserve(tri_indices.size());
    for(uint32_t ti:tri_indices) out.push_back(bvh.faces[ti]); return out;
}
inline std::array<VertexHandle,3>
bvh_tri_vertex_handles(const BVH& bvh, uint32_t ti){
    const auto& tv=bvh.triVerts[ti];
    return { VertexHandle(tv[0]), VertexHandle(tv[1]), VertexHandle(tv[2]) };
}

// =========================== Pixel → NDC helpers ===========================
inline double pixel_to_ndc_x(double x_px, double viewport_w_px){
    // OpenGL-style: x in [0..W) maps to [-1..+1]
    return ( (x_px + 0.5) / viewport_w_px ) * 2.0 - 1.0;
}
inline double pixel_to_ndc_y(double y_px, double viewport_h_px, bool origin_top_left = true){
    // If your pixel origin is top-left, flip Y to bottom-left before mapping
    double y_bl = origin_top_left ? (viewport_h_px - 1.0 - y_px) : y_px;
    return ( (y_bl + 0.5) / viewport_h_px ) * 2.0 - 1.0;
}
//
// =============================== Raycast ===================================
struct RayOM {
    Vec3 o;   // origin
    Vec3 d;   // direction (need not be normalized)
};

struct RayHitOM {
    double t;
    uint32_t triIndex;
    FaceHandle face;
    double u, v; // barycentrics (w = 1-u-v)
};

// Robust slab test; intersects box within [tMin, tMax] (passed by ref).
inline bool intersect_aabb_ray(const AABB& b,
                               const RayOM& r,
                               double& tMin,
                               double& tMax,
                               double eps = 1e-12)
{
    for (int a = 0; a < 3; ++a) {
        const double ro = r.o[a];
        const double rd = r.d[a];
        const double lo = b.bmin[a];
        const double hi = b.bmax[a];

        if (std::abs(rd) < eps) {
            if (ro < lo || ro > hi) return false;
        } else {
            const double inv = 1.0 / rd;
            double t1 = (lo - ro) * inv;
            double t2 = (hi - ro) * inv;
            if (t1 > t2) std::swap(t1, t2);
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            if (tMax < tMin) return false;
        }
    }
    return true;
}

// Möller–Trumbore (double). If cullBackFace=true, reject backfacing tris.
inline bool intersect_triangle_mt(const Vec3& orig,
                                  const Vec3& dir,
                                  const Vec3& A,
                                  const Vec3& B,
                                  const Vec3& C,
                                  double& tOut,
                                  double& uOut,
                                  double& vOut,
                                  bool cullBackFace = false,
                                  double eps = 1e-12)
{
    using OpenMesh::cross;
    using OpenMesh::dot;

    const Vec3 e1 = B - A;
    const Vec3 e2 = C - A;
    const Vec3 p  = cross(dir, e2);
    const double det = dot(e1, p);

    if (cullBackFace) {
        if (det <= eps) return false;
    } else {
        if (std::abs(det) < eps) return false;
    }

    const double invDet = 1.0 / det;
    const Vec3 tvec = orig - A;

    const double u = dot(tvec, p) * invDet;
    if (u < 0.0 || u > 1.0) return false;

    const Vec3 q = cross(tvec, e1);
    const double v = dot(dir, q) * invDet;
    if (v < 0.0 || u + v > 1.0) return false;

    const double t = dot(e2, q) * invDet;
    if (t <= 0.0) return false;

    tOut = t; uOut = u; vOut = v;
    return true;
}

// Closest-hit raycast. Returns std::nullopt on miss.
inline std::optional<RayHitOM> bvh_raycast_closest(const BVH& bvh,
                                                   const Mesh4& mesh,
                                                   const RayOM& ray,
                                                   double tMin = 0.0,
                                                   double tMax = std::numeric_limits<double>::infinity(),
                                                   bool cullBackFace = false)
{
    if (bvh.nodes.empty()) return std::nullopt;

    std::vector<uint32_t> stack; stack.reserve(64);
    stack.push_back(0);

    double bestT = tMax;
    uint32_t bestTri = ~0u;
    double bestU = 0.0, bestV = 0.0;

    while (!stack.empty()) {
        const uint32_t ni = stack.back(); stack.pop_back();
        const BVHNode& node = bvh.nodes[ni];

        double tEntry = tMin, tExit = bestT; // clip by current best
        if (!intersect_aabb_ray(node.box, ray, tEntry, tExit)) continue;

        if (node.is_leaf()) {
            for (uint32_t i = 0; i < node.triCount; ++i) {
                const uint32_t ti = bvh.triIndices[node.left + i];

                // Fast cull by tri AABB
                double te0 = tEntry, te1 = tExit;
                if (!intersect_aabb_ray(bvh.triAABBs[ti], ray, te0, te1)) continue;

                const auto& tv = bvh.triVerts[ti];
                const Vec4 A = mesh.point(VertexHandle(tv[0]));
                const Vec4 B = mesh.point(VertexHandle(tv[1]));
                const Vec4 C = mesh.point(VertexHandle(tv[2]));

                double t, u, v;
                if (intersect_triangle_mt(ray.o, ray.d,
                    { A[0], A[1], A[2] },
                    { B[0], B[1], B[2] },
                    { C[0], C[1], C[2] },
                    t, u, v, cullBackFace))
                {
                    if (t >= tMin && t < bestT) {
                        bestT = t; bestTri = ti; bestU = u; bestV = v;
                    }
                }
            }
        } else {
            // (Optional) push farther child first; here we just push both
            stack.push_back(node.right);
            stack.push_back(node.left);
        }
    }

    if (bestTri == ~0u) return std::nullopt;
    return RayHitOM{ bestT, bestTri, bvh.faces[bestTri], bestU, bestV };
}

