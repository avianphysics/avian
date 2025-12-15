//! A stripped down version of petgraph's `UnGraph`.
//!
//! - Index types always use `u32`.
//! - Edge iteration order after serialization/deserialization is preserved.
//! - Fewer iterators and helpers, and a few new ones.

use core::cmp::max;
use core::ops::{Index, IndexMut};

use derive_more::derive::From;

/// A node identifier for a graph structure.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, Eq, Ord, Hash, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct NodeIndex(pub u32);

impl NodeIndex {
    /// A special index used to denote the final node,
    /// for example at the end of an adjacency list.
    ///
    /// Equivalent to `NodeIndex(u32::MAX)`.
    pub const END: NodeIndex = NodeIndex(u32::MAX);

    /// Returns the inner `u32` value as a `usize`.
    pub fn index(self) -> usize {
        self.0 as usize
    }

    /// Returns `true` if this is the special `END` index.
    pub fn is_end(self) -> bool {
        self == NodeIndex::END
    }
}

/// An edge identifier for a graph structure.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, Eq, Ord, Hash, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct EdgeIndex(pub u32);

impl EdgeIndex {
    /// A special index used to denote the abscence of an edge,
    /// for example at the end of an adjacency list.
    ///
    /// Equivalent to `EdgeIndex(u32::MAX)`.
    pub const END: EdgeIndex = EdgeIndex(u32::MAX);

    /// Returns the inner `u32` value as a `usize`.
    pub fn index(self) -> usize {
        self.0 as usize
    }

    /// Returns `true` if this is the special `END` index.
    pub fn is_end(self) -> bool {
        self == EdgeIndex::END
    }
}

/// The direction of a graph edge.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Ord, Eq, Hash)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[repr(usize)]
pub enum EdgeDirection {
    /// An `Outgoing` edge is an outward edge *from* the current node.
    Outgoing = 0,
    /// An `Incoming` edge is an inbound edge *to* the current node.
    Incoming = 1,
}

impl EdgeDirection {
    /// The two possible directions for an edge.
    pub const ALL: [EdgeDirection; 2] = [EdgeDirection::Outgoing, EdgeDirection::Incoming];
}

/// The node type for a graph structure.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Node<N> {
    /// Associated node data.
    pub weight: N,
    /// Next edge in outgoing and incoming edge lists.
    pub(super) next: [EdgeIndex; 2],
}

/// The edge type for a graph structure.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Edge<E> {
    /// Associated edge data.
    pub weight: E,
    /// Next edge in outgoing and incoming edge lists.
    pub(super) next: [EdgeIndex; 2],
    /// Start and End node index
    pub(super) node: [NodeIndex; 2],
}

impl<E> Edge<E> {
    /// Return the source node index.
    pub fn source(&self) -> NodeIndex {
        self.node[0]
    }

    /// Return the target node index.
    pub fn target(&self) -> NodeIndex {
        self.node[1]
    }
}

/// A graph with undirected edges.
///
/// The graph can invalidate node or edge indices when items are removed.
/// If you need stable indices, use [`StableUnGraph`](super::stable_graph::StableUnGraph).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct UnGraph<N, E> {
    pub(super) nodes: Vec<Node<N>>,
    pub(super) edges: Vec<Edge<E>>,
}

impl<N, E> Default for UnGraph<N, E> {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
        }
    }
}

pub(super) enum Pair<T> {
    Both(T, T),
    One(T),
    None,
}

/// Get mutable references at index `a` and `b`.
pub(super) fn index_twice<T>(arr: &mut [T], a: usize, b: usize) -> Pair<&mut T> {
    if max(a, b) >= arr.len() {
        Pair::None
    } else if a == b {
        Pair::One(&mut arr[max(a, b)])
    } else {
        // safe because `a` and `b` are in bounds and distinct.
        unsafe {
            let ar = &mut *(arr.get_unchecked_mut(a) as *mut _);
            let br = &mut *(arr.get_unchecked_mut(b) as *mut _);
            Pair::Both(ar, br)
        }
    }
}

impl<N, E> UnGraph<N, E> {
    /// Creates a new [`UnGraph`] with estimated capacity.
    pub fn with_capacity(nodes: usize, edges: usize) -> Self {
        UnGraph {
            nodes: Vec::with_capacity(nodes),
            edges: Vec::with_capacity(edges),
        }
    }

    /// Returns the number of nodes in the graph.
    ///
    /// Computes in **O(1)** time.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the number of edges in the graph.
    ///
    /// Computes in **O(1)** time.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Adds a node (also called vertex) with associated data `weight` to the graph.
    ///
    /// Computes in **O(1)** time.
    ///
    /// Returns the index of the new node.
    ///
    /// # Panics
    ///
    /// Panics if the graph is at the maximum number of nodes.
    pub fn add_node(&mut self, weight: N) -> NodeIndex {
        let node = Node {
            weight,
            next: [EdgeIndex::END, EdgeIndex::END],
        };
        assert!(self.nodes.len() as u32 != u32::MAX);
        let node_idx = NodeIndex(self.nodes.len() as u32);
        self.nodes.push(node);
        node_idx
    }

    /// Accesses the weight for node `a`.
    ///
    /// Also available with indexing syntax: `&graph[a]`.
    pub fn node_weight(&self, a: NodeIndex) -> Option<&N> {
        self.nodes.get(a.index()).map(|n| &n.weight)
    }

    /// Adds an edge from `a` to `b` to the graph, with its associated
    /// data `weight`.
    ///
    /// Returns the index of the new edge.
    ///
    /// Computes in **O(1)** time.
    ///
    /// **Note:** `UnGraph` allows adding parallel (“duplicate”) edges. If you want
    /// to avoid this, use [`.update_edge(a, b, weight)`](#method.update_edge) instead.
    ///
    /// # Panics
    ///
    /// Panics if any of the nodes don't exist or the graph is at the maximum number of edges.
    pub fn add_edge(&mut self, a: NodeIndex, b: NodeIndex, weight: E) -> EdgeIndex {
        assert!(self.edges.len() as u32 != u32::MAX);
        let edge_idx = EdgeIndex(self.edges.len() as u32);
        let mut edge = Edge {
            weight,
            node: [a, b],
            next: [EdgeIndex::END; 2],
        };
        match index_twice(&mut self.nodes, a.index(), b.index()) {
            Pair::None => panic!("`UnGraph::add_edge`: node indices out of bounds"),
            Pair::One(an) => {
                edge.next = an.next;
                an.next[0] = edge_idx;
                an.next[1] = edge_idx;
            }
            Pair::Both(an, bn) => {
                // `a` and `b` are different indices
                edge.next = [an.next[0], bn.next[1]];
                an.next[0] = edge_idx;
                bn.next[1] = edge_idx;
            }
        }
        self.edges.push(edge);
        edge_idx
    }

    /// Adds or updates an edge from `a` to `b`.
    /// If the edge already exists, its weight is updated.
    ///
    /// Returns the index of the affected edge.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of edges
    /// connected to `a` (and `b`, if the graph edges are undirected).
    ///
    /// # Panics
    ///
    /// Panics if any of the nodes doesn't exist.
    pub fn update_edge(&mut self, a: NodeIndex, b: NodeIndex, weight: E) -> EdgeIndex {
        if let Some(ix) = self.find_edge(a, b)
            && let Some(ed) = self.edge_weight_mut(ix)
        {
            *ed = weight;
            return ix;
        }
        self.add_edge(a, b, weight)
    }

    /// Accesses the weight for edge `e`.
    ///
    /// Also available with indexing syntax: `&graph[e]`.
    pub fn edge_weight(&self, e: EdgeIndex) -> Option<&E> {
        self.edges.get(e.index()).map(|e| &e.weight)
    }

    /// Accesses the weight for edge `e` mutably.
    ///
    /// Also available with indexing syntax: `&mut graph[e]`.
    pub fn edge_weight_mut(&mut self, e: EdgeIndex) -> Option<&mut E> {
        self.edges.get_mut(e.index()).map(|e| &mut e.weight)
    }

    /// Accesses the source and target nodes for `e`.
    pub fn edge_endpoints(&self, e: EdgeIndex) -> Option<(NodeIndex, NodeIndex)> {
        self.edges
            .get(e.index())
            .map(|ed| (ed.source(), ed.target()))
    }

    /// Removes `a` from the graph if it exists, calling `edge_callback` for each of its edges,
    /// and returns its weight. If it doesn't exist in the graph, returns `None`.
    ///
    /// Apart from `a`, this invalidates the last node index in the graph
    /// (that node will adopt the removed node index). Edge indices are
    /// invalidated as they would be following the removal of each edge
    /// with an endpoint in `a`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of affected
    /// edges, including *n* calls to [`remove_edge`](Self::remove_edge),
    /// where *n* is the number of edges with an endpoint in `a`,
    /// and including the edges with an endpoint in the displaced node.
    pub fn remove_node_with<F>(&mut self, a: NodeIndex, mut edge_callback: F) -> Option<N>
    where
        F: FnMut(E),
    {
        if a.index() >= self.nodes.len() {
            return None;
        }

        for d in EdgeDirection::ALL {
            let k = d as usize;

            // Remove all edges from and to this node.
            loop {
                let next = self.nodes[a.index()].next[k];
                if next == EdgeIndex::END {
                    break;
                }
                let edge = self.remove_edge(next).expect("edge not found for removal");
                edge_callback(edge);
            }
        }

        // Use `swap_remove` -- only the swapped-in node is going to change
        // `NodeIndex`, so we only have to walk its edges and update them.

        let node = self.nodes.swap_remove(a.index());

        // Find the edge lists of the node that had to relocate.
        // It may be that no node had to relocate, then we are done already.
        let swap_edges = match self.nodes.get(a.index()) {
            None => return Some(node.weight),
            Some(ed) => ed.next,
        };

        // The swapped element's old index
        let old_index = NodeIndex(self.nodes.len() as u32);
        let new_index = a;

        // Adjust the starts of the out edges, and ends of the in edges.
        for d in EdgeDirection::ALL {
            let k = d as usize;
            let mut edges = EdgesWalkerMut {
                edges: &mut self.edges,
                next: swap_edges[k],
                dir: d,
            };
            while let Some(curedge) = edges.next_edge() {
                debug_assert!(curedge.node[k] == old_index);
                curedge.node[k] = new_index;
            }
        }
        Some(node.weight)
    }

    /// For edge `e` with endpoints `edge_node`, replaces links to it
    /// with links to `edge_next`.
    pub(super) fn change_edge_links(
        &mut self,
        edge_node: [NodeIndex; 2],
        e: EdgeIndex,
        edge_next: [EdgeIndex; 2],
    ) {
        for d in EdgeDirection::ALL {
            let k = d as usize;
            let node = match self.nodes.get_mut(edge_node[k].index()) {
                Some(r) => r,
                None => {
                    debug_assert!(
                        false,
                        "Edge's endpoint dir={:?} index={:?} not found",
                        d, edge_node[k]
                    );
                    return;
                }
            };
            let fst = node.next[k];
            if fst == e {
                node.next[k] = edge_next[k];
            } else {
                let mut edges = EdgesWalkerMut {
                    edges: &mut self.edges,
                    next: fst,
                    dir: d,
                };
                while let Some(curedge) = edges.next_edge() {
                    if curedge.next[k] == e {
                        curedge.next[k] = edge_next[k];
                        // The edge can only be present once in the list.
                        break;
                    }
                }
            }
        }
    }

    /// Removes an edge and returns its edge weight, or `None` if it didn't exist.
    ///
    /// Apart from `e`, this invalidates the last edge index in the graph
    /// (that edge will adopt the removed edge index).
    ///
    /// Computes in **O(e')** time, where **e'** is the size of four particular edge lists, for
    /// the vertices of `e` and the vertices of another affected edge.
    pub fn remove_edge(&mut self, e: EdgeIndex) -> Option<E> {
        // Every edge is part of two lists, outgoing and incoming edges.
        // Remove it from both.
        let (edge_node, edge_next) = match self.edges.get(e.index()) {
            None => return None,
            Some(x) => (x.node, x.next),
        };

        // Remove the edge from its in and out lists by replacing it with
        // a link to the next in the list.
        self.change_edge_links(edge_node, e, edge_next);
        self.remove_edge_adjust_indices(e)
    }

    fn remove_edge_adjust_indices(&mut self, e: EdgeIndex) -> Option<E> {
        // `swap_remove` the edge -- only the removed edge
        // and the edge swapped into place are affected and need updating
        // indices.
        let edge = self.edges.swap_remove(e.index());
        let swap = match self.edges.get(e.index()) {
            // No element needed to be swapped.
            None => return Some(edge.weight),
            Some(ed) => ed.node,
        };
        let swapped_e = EdgeIndex(self.edges.len() as u32);

        // Update the edge lists by replacing links to the old index by references to the new
        // edge index.
        self.change_edge_links(swap, swapped_e, [e, e]);

        Some(edge.weight)
    }

    /// Returns an iterator of all nodes with an edge connected to `a`.
    ///
    /// Produces an empty iterator if the node doesn't exist.
    ///
    /// The iterator element type is `NodeIndex`.
    pub fn neighbors(&self, a: NodeIndex) -> Neighbors<'_, E> {
        Neighbors {
            skip_start: a,
            edges: &self.edges,
            next: match self.nodes.get(a.index()) {
                None => [EdgeIndex::END, EdgeIndex::END],
                Some(n) => n.next,
            },
        }
    }

    /// Returns an iterator of all edges connected to `a`.
    ///
    /// Produces an empty iterator if the node doesn't exist.
    ///
    /// The iterator element type is `EdgeReference<E>`.
    pub fn edges(&self, a: NodeIndex) -> Edges<'_, E> {
        Edges {
            skip_start: a,
            edges: &self.edges,
            direction: EdgeDirection::Outgoing,
            next: match self.nodes.get(a.index()) {
                None => [EdgeIndex::END, EdgeIndex::END],
                Some(n) => n.next,
            },
        }
    }

    /// Returns a mutable iterator of all edges connected to `a`.
    ///
    /// Produces an empty iterator if the node doesn't exist.
    ///
    /// The iterator element type is `EdgeReference<E>`.
    pub fn edges_mut(&mut self, a: NodeIndex) -> EdgesMut<'_, N, E> {
        let incoming_edge = self.first_edge(a, EdgeDirection::Incoming);
        let outgoing_edge = self.first_edge(a, EdgeDirection::Outgoing);

        EdgesMut {
            graph: self,
            incoming_edge,
            outgoing_edge,
        }
    }

    /// Returns an iterator over all edges between `a` and `b`.
    ///
    /// The iterator element type is `EdgeReference<E>`.
    pub fn edges_between(&self, a: NodeIndex, b: NodeIndex) -> EdgesBetween<'_, E> {
        EdgesBetween {
            target_node: b,
            edges: self.edges(a),
        }
    }

    /// Looks up if there is an edge from `a` to `b`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of edges
    /// connected to `a` (and `b`, if the graph edges are undirected).
    pub fn contains_edge(&self, a: NodeIndex, b: NodeIndex) -> bool {
        self.find_edge(a, b).is_some()
    }

    /// Looks up an edge from `a` to `b`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of edges
    /// connected to `a` (and `b`, if the graph edges are undirected).
    pub fn find_edge(&self, a: NodeIndex, b: NodeIndex) -> Option<EdgeIndex> {
        self.find_edge_from_node(self.nodes.get(a.index())?, b)
    }

    pub(super) fn find_edge_from_node(&self, node: &Node<N>, b: NodeIndex) -> Option<EdgeIndex> {
        for &d in &EdgeDirection::ALL {
            let k = d as usize;
            let mut edix = node.next[k];
            while let Some(edge) = self.edges.get(edix.index()) {
                if edge.node[1 - k] == b {
                    return Some(edix);
                }
                edix = edge.next[k];
            }
        }
        None
    }

    /// Returns an iterator yielding immutable access to edge weights for edges from or to `a`.
    pub fn edge_weights(&self, a: NodeIndex) -> EdgeWeights<'_, E> {
        EdgeWeights {
            skip_start: a,
            edges: &self.edges,
            direction: EdgeDirection::Outgoing,
            next: match self.nodes.get(a.index()) {
                None => [EdgeIndex::END, EdgeIndex::END],
                Some(n) => n.next,
            },
        }
    }

    /// Returns an iterator yielding mutable access to edge weights for edges from or to `a`.
    pub fn edge_weights_mut(&mut self, a: NodeIndex) -> EdgeWeightsMut<'_, N, E> {
        let incoming_edge = self.first_edge(a, EdgeDirection::Incoming);
        let outgoing_edge = self.first_edge(a, EdgeDirection::Outgoing);

        EdgeWeightsMut {
            graph: self,
            incoming_edge,
            outgoing_edge,
        }
    }

    /// Returns an iterator yielding immutable access to all edge weights.
    ///
    /// The order in which weights are yielded matches the order of their
    /// edge indices.
    pub fn all_edge_weights(&self) -> AllEdgeWeights<'_, E> {
        AllEdgeWeights {
            edges: self.edges.iter(),
        }
    }

    /// Returns an iterator yielding mutable access to all edge weights.
    ///
    /// The order in which weights are yielded matches the order of their
    /// edge indices.
    pub fn all_edge_weights_mut(&mut self) -> AllEdgeWeightsMut<'_, E> {
        AllEdgeWeightsMut {
            edges: self.edges.iter_mut(),
        }
    }

    /// Accesses the internal node array.
    pub fn raw_nodes(&self) -> &[Node<N>] {
        &self.nodes
    }

    /// Accesses the internal node array mutably.
    pub fn raw_nodes_mut(&mut self) -> &mut [Node<N>] {
        &mut self.nodes
    }

    /// Accesses the internal edge array.
    pub fn raw_edges(&self) -> &[Edge<E>] {
        &self.edges
    }

    /// Accesses the internal edge array mutably.
    pub fn raw_edges_mut(&mut self) -> &mut [Edge<E>] {
        &mut self.edges
    }

    /// Accessor for data structure internals: returns the first edge in the given direction.
    pub fn first_edge(&self, a: NodeIndex, dir: EdgeDirection) -> Option<EdgeIndex> {
        match self.nodes.get(a.index()) {
            None => None,
            Some(node) => {
                let edix = node.next[dir as usize];
                if edix == EdgeIndex::END {
                    None
                } else {
                    Some(edix)
                }
            }
        }
    }

    /// Accessor for data structure internals: returns the next edge for the given direction.
    pub fn next_edge(&self, e: EdgeIndex, dir: EdgeDirection) -> Option<EdgeIndex> {
        match self.edges.get(e.index()) {
            None => None,
            Some(node) => {
                let edix = node.next[dir as usize];
                if edix == EdgeIndex::END {
                    None
                } else {
                    Some(edix)
                }
            }
        }
    }

    /// Removes all nodes and edges.
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.edges.clear();
    }

    /// Removes all edges.
    pub fn clear_edges(&mut self) {
        self.edges.clear();
        for node in &mut self.nodes {
            node.next = [EdgeIndex::END, EdgeIndex::END];
        }
    }

    /// Returns the current node capacity of the graph.
    pub fn nodes_capacity(&self) -> usize {
        self.nodes.capacity()
    }

    /// Returns the current edge capacity of the graph.
    pub fn edges_capacity(&self) -> usize {
        self.edges.capacity()
    }

    /// Reserves capacity for at least `additional` more nodes to be inserted in
    /// the graph. Graph may reserve more space to avoid frequent reallocations.
    ///
    /// # Panics
    ///
    /// Panics if the new capacity overflows `usize`.
    pub fn reserve_nodes(&mut self, additional: usize) {
        self.nodes.reserve(additional);
    }

    /// Reserves capacity for at least `additional` more edges to be inserted in
    /// the graph. Graph may reserve more space to avoid frequent reallocations.
    ///
    /// # Panics
    ///
    /// Panics if the new capacity overflows `usize`.
    pub fn reserve_edges(&mut self, additional: usize) {
        self.edges.reserve(additional);
    }
}

/// An iterator over the neighbors of a node.
///
/// The iterator element type is `NodeIndex`.
#[derive(Debug)]
pub struct Neighbors<'a, E: 'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges to iterate over.
    edges: &'a [Edge<E>],

    /// The next edge to visit.
    next: [EdgeIndex; 2],
}

impl<E> Iterator for Neighbors<'_, E> {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<NodeIndex> {
        // First any outgoing edges.
        match self.edges.get(self.next[0].index()) {
            None => {}
            Some(edge) => {
                self.next[0] = edge.next[0];
                return Some(edge.node[1]);
            }
        }

        // Then incoming edges.
        // For an "undirected" iterator, make sure we don't double
        // count self-loops by skipping them in the incoming list.
        while let Some(edge) = self.edges.get(self.next[1].index()) {
            self.next[1] = edge.next[1];
            if edge.node[0] != self.skip_start {
                return Some(edge.node[0]);
            }
        }
        None
    }
}

impl<E> Clone for Neighbors<'_, E> {
    fn clone(&self) -> Self {
        Neighbors {
            skip_start: self.skip_start,
            edges: self.edges,
            next: self.next,
        }
    }
}

/// An iterator over edges from or to a node.
pub struct Edges<'a, E: 'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges to iterate over.
    edges: &'a [Edge<E>],

    /// The next edge to visit.
    next: [EdgeIndex; 2],

    /// The direction of edges.
    direction: EdgeDirection,
}

impl<'a, E> Iterator for Edges<'a, E> {
    type Item = EdgeReference<'a, E>;

    fn next(&mut self) -> Option<Self::Item> {
        // Outgoing
        let i = self.next[0].index();
        if let Some(Edge {
            node, weight, next, ..
        }) = self.edges.get(i)
        {
            self.next[0] = next[0];
            return Some(EdgeReference {
                index: EdgeIndex(i as u32),
                node: if self.direction == EdgeDirection::Incoming {
                    swap_pair(*node)
                } else {
                    *node
                },
                weight,
            });
        }

        // Incoming
        while let Some(Edge { node, weight, next }) = self.edges.get(self.next[1].index()) {
            let edge_index = self.next[1];
            self.next[1] = next[1];

            // In any of the "both" situations, self-loops would be iterated over twice.
            // Skip them here.
            if node[0] == self.skip_start {
                continue;
            }

            return Some(EdgeReference {
                index: edge_index,
                node: if self.direction == EdgeDirection::Outgoing {
                    swap_pair(*node)
                } else {
                    *node
                },
                weight,
            });
        }

        None
    }
}

impl<E> Clone for Edges<'_, E> {
    fn clone(&self) -> Self {
        Edges {
            skip_start: self.skip_start,
            edges: self.edges,
            next: self.next,
            direction: self.direction,
        }
    }
}

/// An iterator over mutable references to all edges from or to a node.
pub struct EdgesMut<'a, N, E> {
    graph: &'a mut UnGraph<N, E>,
    incoming_edge: Option<EdgeIndex>,
    outgoing_edge: Option<EdgeIndex>,
}

impl<'a, N: Copy, E> Iterator for EdgesMut<'a, N, E> {
    type Item = EdgeMut<'a, E>;

    #[inline]
    fn next(&mut self) -> Option<EdgeMut<'a, E>> {
        if let Some(edge) = self.incoming_edge {
            self.incoming_edge = self.graph.next_edge(edge, EdgeDirection::Incoming);
            let weights = &mut self.graph[edge];
            return Some(EdgeMut {
                index: edge,
                weight: unsafe { core::mem::transmute::<&mut E, &'a mut E>(weights) },
            });
        }

        let edge = self.outgoing_edge?;
        self.outgoing_edge = self.graph.next_edge(edge, EdgeDirection::Outgoing);
        let weights = &mut self.graph[edge];
        Some(EdgeMut {
            index: edge,
            weight: unsafe { core::mem::transmute::<&mut E, &'a mut E>(weights) },
        })
    }
}

/// Iterator over the edges between a source node and a target node.
#[derive(Clone)]
pub struct EdgesBetween<'a, E: 'a> {
    target_node: NodeIndex,
    edges: Edges<'a, E>,
}

impl<'a, E> Iterator for EdgesBetween<'a, E> {
    type Item = EdgeReference<'a, E>;

    fn next(&mut self) -> Option<EdgeReference<'a, E>> {
        let target_node = self.target_node;
        self.edges
            .by_ref()
            .find(|&edge| edge.target() == target_node)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let (_, upper) = self.edges.size_hint();
        (0, upper)
    }
}

fn swap_pair<T>(mut x: [T; 2]) -> [T; 2] {
    x.swap(0, 1);
    x
}

// TODO: Reduce duplication between `Edges` variants.
/// An iterator over edge weights for edges from or to a node.
pub struct EdgeWeights<'a, E: 'a> {
    /// The starting node to skip over.
    skip_start: NodeIndex,

    /// The edges to iterate over.
    edges: &'a [Edge<E>],

    /// The next edge to visit.
    next: [EdgeIndex; 2],

    /// The direction of edges.
    direction: EdgeDirection,
}

impl<'a, E> Iterator for EdgeWeights<'a, E> {
    type Item = &'a E;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.next[0].index();
        if let Some(Edge { weight, next, .. }) = self.edges.get(i) {
            self.next[0] = next[0];
            return Some(weight);
        }

        while let Some(Edge { node, weight, next }) = self.edges.get(self.next[1].index()) {
            self.next[1] = next[1];

            // In any of the "both" situations, self-loops would be iterated over twice.
            // Skip them here.
            if node[0] == self.skip_start {
                continue;
            }

            return Some(weight);
        }

        None
    }
}

impl<E> Clone for EdgeWeights<'_, E> {
    fn clone(&self) -> Self {
        EdgeWeights {
            skip_start: self.skip_start,
            edges: self.edges,
            next: self.next,
            direction: self.direction,
        }
    }
}

/// An iterator over mutable references to all edge weights from or to a node.
pub struct EdgeWeightsMut<'a, N, E> {
    /// A mutable reference to the graph.
    pub graph: &'a mut UnGraph<N, E>,

    /// The next incoming edge to visit.
    pub incoming_edge: Option<EdgeIndex>,

    /// The next outgoing edge to visit.
    pub outgoing_edge: Option<EdgeIndex>,
}

impl<'a, N: Copy, E> Iterator for EdgeWeightsMut<'a, N, E> {
    type Item = &'a mut E;

    #[inline]
    fn next(&mut self) -> Option<&'a mut E> {
        if let Some(edge) = self.incoming_edge {
            self.incoming_edge = self.graph.next_edge(edge, EdgeDirection::Incoming);
            let weight = &mut self.graph[edge];
            return Some(unsafe { core::mem::transmute::<&mut E, &'a mut E>(weight) });
        }

        let edge = self.outgoing_edge?;
        self.outgoing_edge = self.graph.next_edge(edge, EdgeDirection::Outgoing);
        let weight = &mut self.graph[edge];
        Some(unsafe { core::mem::transmute::<&mut E, &'a mut E>(weight) })
    }
}

/// An iterator yielding immutable access to all edge weights.
pub struct AllEdgeWeights<'a, E: 'a> {
    edges: core::slice::Iter<'a, Edge<E>>,
}

impl<'a, E> Iterator for AllEdgeWeights<'a, E> {
    type Item = &'a E;

    fn next(&mut self) -> Option<&'a E> {
        self.edges.next().map(|edge| &edge.weight)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.edges.size_hint()
    }
}

/// An iterator yielding mutable access to all edge weights.
#[derive(Debug)]
pub struct AllEdgeWeightsMut<'a, E: 'a> {
    edges: core::slice::IterMut<'a, Edge<E>>,
}

impl<'a, E> Iterator for AllEdgeWeightsMut<'a, E> {
    type Item = &'a mut E;

    fn next(&mut self) -> Option<&'a mut E> {
        self.edges.next().map(|edge| &mut edge.weight)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.edges.size_hint()
    }
}

struct EdgesWalkerMut<'a, E: 'a> {
    edges: &'a mut [Edge<E>],
    next: EdgeIndex,
    dir: EdgeDirection,
}

impl<E> EdgesWalkerMut<'_, E> {
    fn next_edge(&mut self) -> Option<&mut Edge<E>> {
        self.next().map(|t| t.1)
    }

    fn next(&mut self) -> Option<(EdgeIndex, &mut Edge<E>)> {
        let this_index = self.next;
        let k = self.dir as usize;
        match self.edges.get_mut(self.next.index()) {
            None => None,
            Some(edge) => {
                self.next = edge.next[k];
                Some((this_index, edge))
            }
        }
    }
}

/// Indexes the `UnGraph` by `NodeIndex` to access node weights.
///
/// # Panics
///
/// Panics if the node doesn't exist.
impl<N, E> Index<NodeIndex> for UnGraph<N, E> {
    type Output = N;
    fn index(&self, index: NodeIndex) -> &N {
        &self.nodes[index.index()].weight
    }
}

/// Indexes the `UnGraph` by `NodeIndex` to access node weights.
///
/// # Panics
///
/// Panics if the node doesn't exist.
impl<N, E> IndexMut<NodeIndex> for UnGraph<N, E> {
    fn index_mut(&mut self, index: NodeIndex) -> &mut N {
        &mut self.nodes[index.index()].weight
    }
}

/// Indexes the `UnGraph` by `EdgeIndex` to access edge weights.
///
/// # Panics
///
/// Panics if the edge doesn't exist.
impl<N, E> Index<EdgeIndex> for UnGraph<N, E> {
    type Output = E;
    fn index(&self, index: EdgeIndex) -> &E {
        &self.edges[index.index()].weight
    }
}

/// Indexes the `UnGraph` by `EdgeIndex` to access edge weights.
///
/// # Panics
///
/// Panics if the edge doesn't exist.
impl<N, E> IndexMut<EdgeIndex> for UnGraph<N, E> {
    fn index_mut(&mut self, index: EdgeIndex) -> &mut E {
        &mut self.edges[index.index()].weight
    }
}

/// A reference to a graph edge.
#[derive(Debug)]
pub struct EdgeReference<'a, E: 'a> {
    index: EdgeIndex,
    node: [NodeIndex; 2],
    weight: &'a E,
}

impl<'a, E: 'a> EdgeReference<'a, E> {
    /// Returns the index of the edge.
    #[inline]
    pub fn index(&self) -> EdgeIndex {
        self.index
    }

    /// Returns the source node index.
    #[inline]
    pub fn source(&self) -> NodeIndex {
        self.node[0]
    }

    /// Returns the target node index.
    #[inline]
    pub fn target(&self) -> NodeIndex {
        self.node[1]
    }

    /// Returns the weight of the edge.
    #[inline]
    pub fn weight(&self) -> &'a E {
        self.weight
    }
}

impl<E> Clone for EdgeReference<'_, E> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<E> Copy for EdgeReference<'_, E> {}

impl<E> PartialEq for EdgeReference<'_, E>
where
    E: PartialEq,
{
    fn eq(&self, rhs: &Self) -> bool {
        self.index == rhs.index && self.weight == rhs.weight
    }
}

/// A mutable reference to a graph edge.
#[derive(Debug)]
pub struct EdgeMut<'a, E: 'a> {
    index: EdgeIndex,
    weight: &'a mut E,
}

impl<E> EdgeMut<'_, E> {
    /// Returns the index of the edge.
    #[inline]
    pub fn index(&self) -> EdgeIndex {
        self.index
    }

    /// Returns the weight of the edge.
    #[inline]
    pub fn weight(&self) -> &E {
        self.weight
    }

    /// Returns the weight of the edge mutably.
    #[inline]
    pub fn weight_mut(&mut self) -> &mut E {
        self.weight
    }
}

impl<E> PartialEq for EdgeMut<'_, E>
where
    E: PartialEq,
{
    fn eq(&self, rhs: &Self) -> bool {
        self.index == rhs.index && self.weight == rhs.weight
    }
}
