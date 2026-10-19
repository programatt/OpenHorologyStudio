// SPDX-License-Identifier: Apache-2.0
#include "fea/mfem_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#ifdef OHS_WITH_MFEM
#include "mfem.hpp"
#endif

namespace ohs::fea {

bool mfem_available() {
#ifdef OHS_WITH_MFEM
    return true;
#else
    return false;
#endif
}

#ifdef OHS_WITH_MFEM

bool mfem_solve(const Model& model, const Vec& load, Vec& u, int& iterations, std::string& note) {
    using namespace mfem;
    if (!model.ties.empty()) {
        note = "MFEM: models with ties are not supported";
        return false;
    }
    if (!model.element_material.empty() || !model.materials.empty()) {
        note = "MFEM: models with several materials are not supported";
        return false;
    }
    const mesh::TetMesh& tm = model.mesh;
    if (tm.elems.empty() || load.size() != static_cast<Eigen::Index>(tm.dofs())) {
        note = "MFEM: the model and the load do not match";
        return false;
    }
    const int npe = tm.nodes_per_element();

    // The corner nodes become MFEM's vertices; the edge midpoints of quadratic elements are MFEM's edge dofs.
    std::vector<int> vertex_of(tm.nodes.size(), -1);
    int vertices = 0;
    for (const auto& e : tm.elems) {
        for (int k = 0; k < 4; ++k) {
            if (vertex_of[static_cast<std::size_t>(e[k])] < 0) vertex_of[static_cast<std::size_t>(e[k])] = vertices++;
        }
    }
    Mesh mfem_mesh(3, vertices, static_cast<int>(tm.elems.size()), 0, 3);
    std::vector<int> node_of_vertex(static_cast<std::size_t>(vertices));
    for (std::size_t n = 0; n < tm.nodes.size(); ++n) {
        if (vertex_of[n] >= 0) node_of_vertex[static_cast<std::size_t>(vertex_of[n])] = static_cast<int>(n);
    }
    for (int v = 0; v < vertices; ++v) {
        const auto& p = tm.nodes[static_cast<std::size_t>(node_of_vertex[static_cast<std::size_t>(v)])];
        mfem_mesh.AddVertex(p.x, p.y, p.z);
    }
    for (const auto& e : tm.elems) {
        mfem_mesh.AddTet(vertex_of[static_cast<std::size_t>(e[0])], vertex_of[static_cast<std::size_t>(e[1])],
                         vertex_of[static_cast<std::size_t>(e[2])], vertex_of[static_cast<std::size_t>(e[3])], 1);
    }
    mfem_mesh.FinalizeTopology();
    mfem_mesh.Finalize(false, true);  // fixes the orientation of tetrahedra that are left-handed

    H1_FECollection fec(tm.order, 3);
    FiniteElementSpace scalar(&mfem_mesh, &fec, 1);
    FiniteElementSpace space(&mfem_mesh, &fec, 3, Ordering::byNODES);

    // Which MFEM dof sits at which OHS node, found from the physical positions of each element's dofs.
    std::vector<int> dof_of_node(tm.nodes.size(), -1);
    for (int e = 0; e < mfem_mesh.GetNE(); ++e) {
        const FiniteElement* fe = scalar.GetFE(e);
        ElementTransformation* tr = scalar.GetElementTransformation(e);
        Array<int> dofs;
        scalar.GetElementDofs(e, dofs);
        const IntegrationRule& nodes = fe->GetNodes();
        const auto& ours = tm.elems[static_cast<std::size_t>(e)];
        for (int i = 0; i < dofs.Size(); ++i) {
            Vector x(3);
            tr->Transform(nodes.IntPoint(i), x);
            int best = -1;
            double best_d = std::numeric_limits<double>::max();
            for (int k = 0; k < npe; ++k) {
                const auto& p = tm.nodes[static_cast<std::size_t>(ours[k])];
                const double d = std::hypot(std::hypot(p.x - x(0), p.y - x(1)), p.z - x(2));
                if (d < best_d) {
                    best_d = d;
                    best = ours[k];
                }
            }
            if (best >= 0) dof_of_node[static_cast<std::size_t>(best)] = dofs[i];
        }
    }
    const auto vdof = [&](int node, int comp) { return space.DofToVDof(dof_of_node[static_cast<std::size_t>(node)], comp); };

    const double e_mod = model.material.youngs_mpa(), nu = model.material.poisson;
    ConstantCoefficient lambda(e_mod * nu / ((1 + nu) * (1 - 2 * nu)));
    ConstantCoefficient mu(e_mod / (2 * (1 + nu)));
    BilinearForm a(&space);
    a.AddDomainIntegrator(new ElasticityIntegrator(lambda, mu));
    a.Assemble();

    GridFunction x(&space);
    x = 0.0;
    LinearForm b(&space);
    b = 0.0;
    for (std::size_t n = 0; n < tm.nodes.size(); ++n) {
        if (dof_of_node[n] < 0) continue;
        for (int c = 0; c < 3; ++c) b[vdof(static_cast<int>(n), c)] = load(static_cast<Eigen::Index>(3 * n) + c);
    }
    Array<int> essential;
    std::set<int> seen;
    for (const Fixed& f : model.fixed) {
        if (dof_of_node[static_cast<std::size_t>(f.node)] < 0) continue;
        const int d = vdof(f.node, f.comp);
        x[d] = f.value;
        if (seen.insert(d).second) essential.Append(d);
    }

    OperatorPtr A;
    Vector B, X;
    a.FormLinearSystem(essential, x, b, A, X, B);
    GSSmoother smoother(static_cast<SparseMatrix&>(*A));
    CGSolver cg;
    cg.SetOperator(*A);
    cg.SetPreconditioner(smoother);
    cg.SetRelTol(1e-12);
    cg.SetAbsTol(0.0);
    cg.SetMaxIter(200000);
    cg.SetPrintLevel(-1);
    cg.Mult(B, X);
    iterations = cg.GetNumIterations();
    if (!cg.GetConverged()) {
        note = "MFEM: the conjugate gradient solver did not converge";
        return false;
    }
    a.RecoverFEMSolution(X, b, x);
    u = Vec::Zero(load.size());
    for (std::size_t n = 0; n < tm.nodes.size(); ++n) {
        if (dof_of_node[n] < 0) continue;
        for (int c = 0; c < 3; ++c) u(static_cast<Eigen::Index>(3 * n) + c) = x[vdof(static_cast<int>(n), c)];
    }
    note = "MFEM " + std::to_string(MFEM_VERSION_MAJOR) + "." + std::to_string(MFEM_VERSION_MINOR) + ", " + std::to_string(iterations) + " CG iterations";
    return true;
}

MfemCheck run_mfem_crosscheck(const Model& model, const Vec& load, const StaticResult& ohs_result) {
    MfemCheck out;
    Vec u;
    if (!mfem_solve(model, load, u, out.cg_iterations, out.note)) return out;
    const mesh::TetMesh& tm = model.mesh;
    double max_ohs = 0.0, max_mfem = 0.0, max_diff = 0.0;
    for (std::size_t n = 0; n < tm.nodes.size(); ++n) {
        for (int c = 0; c < 3; ++c) {
            const Eigen::Index k = static_cast<Eigen::Index>(3 * n) + c;
            max_ohs = std::max(max_ohs, std::abs(ohs_result.u(k)));
            max_mfem = std::max(max_mfem, std::abs(u(k)));
            max_diff = std::max(max_diff, std::abs(ohs_result.u(k) - u(k)));
        }
    }
    out.available = true;
    out.ohs_max_displacement = max_ohs;
    out.mfem_max_displacement = max_mfem;
    out.max_displacement_diff = max_ohs > 0 ? max_diff / max_ohs : max_diff;
    return out;
}

#else

bool mfem_solve(const Model&, const Vec&, Vec&, int&, std::string& note) {
    note = "this build of ohs has no MFEM support; install libmfem-dev (with HYPRE and MPI headers) and reconfigure";
    return false;
}

MfemCheck run_mfem_crosscheck(const Model&, const Vec&, const StaticResult&) {
    MfemCheck out;
    out.note = "this build of ohs has no MFEM support; install libmfem-dev (with HYPRE and MPI headers) and reconfigure";
    return out;
}

#endif

}  // namespace ohs::fea
