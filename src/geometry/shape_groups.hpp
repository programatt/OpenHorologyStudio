// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstddef>
#include <map>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "core/parallel.hpp"

namespace ohs::geometry {

/// Runs f(i) for every index of `shapes` on threads, such that two indices whose shapes share a TShape never
/// run at the same time: one thread takes all the instances of a shape in turn.
///
/// A movement's screws, jewels and pins are many parts with one underlying shape. OCCT writes into the shape
/// it is given (the triangulation of a mesh, tolerances and p-curves of a boolean), so two threads working on
/// instances of one shape corrupt it; meshing 166 parts of which 107 were distinct crashed with a double free.
/// Output is independent of the schedule as long as f(i) writes only slot i. `max_threads` as in parallel_for.
template <class F>
void parallel_for_shapes(const std::vector<TopoDS_Shape>& shapes, F f, unsigned max_threads = 0) {
    std::map<const void*, std::vector<std::size_t>> by_shape;
    for (std::size_t i = 0; i < shapes.size(); ++i) by_shape[shapes[i].TShape().get()].push_back(i);
    std::vector<const std::vector<std::size_t>*> groups;
    groups.reserve(by_shape.size());
    for (const auto& entry : by_shape) groups.push_back(&entry.second);
    // The biggest groups first, so the longest chains start early.
    std::stable_sort(groups.begin(), groups.end(), [](const auto* a, const auto* b) { return a->size() > b->size(); });
    parallel_for(groups.size(), [&](std::size_t g) {
        for (const std::size_t i : *groups[g]) f(i);
    }, max_threads);
}

/// The same, but f receives a whole group: the indices of the instances of one shape, in order. For work that does a
/// step on the shape itself (meshing it) and then one per instance (reading the result in its placement).
template <class F>
void parallel_for_shape_groups(const std::vector<TopoDS_Shape>& shapes, F f, unsigned max_threads = 0) {
    std::map<const void*, std::vector<std::size_t>> by_shape;
    for (std::size_t i = 0; i < shapes.size(); ++i) by_shape[shapes[i].TShape().get()].push_back(i);
    std::vector<const std::vector<std::size_t>*> groups;
    groups.reserve(by_shape.size());
    for (const auto& entry : by_shape) groups.push_back(&entry.second);
    std::stable_sort(groups.begin(), groups.end(), [](const auto* a, const auto* b) { return a->size() > b->size(); });
    parallel_for(groups.size(), [&](std::size_t g) { f(*groups[g]); }, max_threads);
}

}  // namespace ohs::geometry
