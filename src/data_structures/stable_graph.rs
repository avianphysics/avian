//! A stripped down and modified version of petgraph's `StableUnGraph`.
//!
//! - Index types always use `u32`.
//! - Instead of free lists, [`IdPool`]s are used to allocate and free nodes and edges.
//! - Vacant nodes and edges are occupied in the order of lowest index first.
//! - Edge iteration order after serialization/deserialization is preserved.
//! - Fewer iterators and helpers, and a few new ones.

use super::graph::{Edge, EdgeDirection, EdgeIndex, Node, NodeIndex, Pair, UnGraph, index_twice};
use super::id_pool::IdPool;

/// A graph with undirected edges and stable indices.
///
/// Unlike [`UnGraph`], this graph *does not* invalidate any unrelated
/// node or edge indices when items are removed. Removed nodes and edges
/// are replaced with vacant slots, which can be reused later in order
/// of lowest index first.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct StableUnGraph<N, E> {
    graph: UnGraph<Option<N>, Option<E>>,
    node_ids: IdPool,
    edge_ids: IdPool,
}

impl<N, E> Default for StableUnGraph<N, E> {
    fn default() -> Self {
        StableUnGraph {
            graph: UnGraph::default(),
            node_ids: IdPool::new(),
            edge_ids: IdPool::new(),
        }
    }
}

impl<N, E> StableUnGraph<N, E> {
    /// Creates a new [`StableUnGraph`] with estimated capacity.
    pub fn with_capacity(nodes: usize, edges: usize) -> Self {
        StableUnGraph {
            graph: UnGraph::with_capacity(nodes, edges),
            node_ids: IdPool::with_capacity(nodes),
            edge_ids: IdPool::with_capacity(edges),
        }
    }

    /// Returns the number of nodes in the graph.
    ///
    /// Computes in **O(1)** time.
    pub fn node_count(&self) -> usize {
        self.node_ids.len()
    }

    /// Returns the number of edges in the graph.
    ///
    /// Computes in **O(1)** time.
    pub fn edge_count(&self) -> usize {
        self.edge_ids.len()
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
        // Allocate a node ID.
        let node_idx = NodeIndex(self.node_ids.alloc());

        if node_idx.index() != self.graph.nodes.len() {
            // Reuse a free node.
            self.occupy_vacant_node(node_idx, weight);
            node_idx
        } else {
            // Create a new node.
            self.graph.add_node(Some(weight))
        }
    }

    /// Creates a new node using a vacant position.
    fn occupy_vacant_node(&mut self, node_idx: NodeIndex, weight: N) {
        let node_slot = &mut self.graph.nodes[node_idx.index()];

        let _old = node_slot.weight.replace(weight);
        debug_assert!(_old.is_none());

        let previous_node = node_slot.next[1];
        let next_node = node_slot.next[0];
        node_slot.next = [EdgeIndex::END, EdgeIndex::END];

        if previous_node != EdgeIndex::END {
            self.graph.nodes[previous_node.index()].next[0] = next_node;
        }
        if next_node != EdgeIndex::END {
            self.graph.nodes[next_node.index()].next[1] = previous_node;
        }
    }

    /// Returns the node at index `a` if it is not vacant.
    fn get_node(&self, a: NodeIndex) -> Option<&Node<Option<N>>> {
        self.graph
            .nodes
            .get(a.index())
            .and_then(|node| node.weight.as_ref().map(|_| node))
    }

    /// Accesses the weight for node `a`.
    ///
    /// Also available with indexing syntax: `&graph[a]`.
    pub fn node_weight(&self, a: NodeIndex) -> Option<&N> {
        let node = self.graph.nodes.get(a.index())?;
        node.weight.as_ref()
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
        let edge_idx;
        let mut new_edge = None::<Edge<Option<E>>>;
        {
            let edge: &mut Edge<Option<E>>;

            // Allocate an edge ID.
            edge_idx = EdgeIndex(self.edge_ids.alloc());

            if edge_idx.index() != self.graph.edges.len() {
                // Reuse a free edge.
                edge = &mut self.graph.edges[edge_idx.index()];

                let _old = edge.weight.replace(weight);
                debug_assert!(_old.is_none());

                edge.node = [a, b];
            } else {
                // Create a new edge.
                assert!(edge_idx != EdgeIndex::END);
                new_edge = Some(Edge {
                    weight: Some(weight),
                    node: [a, b],
                    next: [EdgeIndex::END; 2],
                });
                edge = new_edge.as_mut().unwrap();
            }

            let wrong_index = match index_twice(&mut self.graph.nodes, a.index(), b.index()) {
                Pair::None => Some(core::cmp::max(a.index(), b.index())),
                Pair::One(an) => {
                    if an.weight.is_none() {
                        Some(a.index())
                    } else {
                        edge.next = an.next;
                        an.next[0] = edge_idx;
                        an.next[1] = edge_idx;
                        None
                    }
                }
                Pair::Both(an, bn) => {
                    // `a` and `b` are different indices.
                    if an.weight.is_none() {
                        Some(a.index())
                    } else if bn.weight.is_none() {
                        Some(b.index())
                    } else {
                        edge.next = [an.next[0], bn.next[1]];
                        an.next[0] = edge_idx;
                        bn.next[1] = edge_idx;
                        None
                    }
                }
            };

            assert!(
                wrong_index.is_none(),
                "Tried adding an edge to a non-existent node."
            );
        }

        if let Some(edge) = new_edge {
            self.graph.edges.push(edge);
        }

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
    /// Panics if any of the nodes doesn't exist or the graph is at the maximum number of edges.
    pub fn update_edge(&mut self, a: NodeIndex, b: NodeIndex, weight: E) -> EdgeIndex {
        if let Some(ix) = self.find_edge(a, b) {
            *self.edge_weight_mut(ix).unwrap() = weight;
            return ix;
        }
        self.add_edge(a, b, weight)
    }

    /// Accesses the weight for edge `e`.
    ///
    /// Also available with indexing syntax: `&graph[e]`.
    pub fn edge_weight(&self, e: EdgeIndex) -> Option<&E> {
        let edge = self.graph.edges.get(e.index())?;
        edge.weight.as_ref()
    }

    /// Accesses the weight for edge `e` mutably.
    ///
    /// Also available with indexing syntax: `&mut graph[e]`.
    pub fn edge_weight_mut(&mut self, e: EdgeIndex) -> Option<&mut E> {
        let edge = self.graph.edges.get_mut(e.index())?;
        edge.weight.as_mut()
    }

    /// Accesses the source and target nodes for `e`.
    pub fn edge_endpoints(&self, e: EdgeIndex) -> Option<(NodeIndex, NodeIndex)> {
        let edge = self.graph.edges.get(e.index())?;
        Some((edge.source(), edge.target()))
    }

    /// Removes `a` from the graph if it exists, calling `edge_callback` for each of its edges
    /// right before removal, and returns its weight. If it doesn't exist in the graph, returns `None`.
    ///
    /// Invalidates the node index `a`, but not any other indices.
    /// Edge indices are invalidated as they would be following
    /// the removal of each edge with an endpoint in `a`.
    ///
    /// Computes in **O(e')** time, where **e'** is the number of affected
    /// edges, including *n* calls to [`remove_edge`](Self::remove_edge),
    /// where *n* is the number of edges with an endpoint in `a`,
    /// and including the edges with an endpoint in the displaced node.
    pub fn remove_node_with<F>(&mut self, a: NodeIndex, mut edge_callback: F) -> Option<N>
    where
        F: FnMut(&mut Self, EdgeIndex),
    {
        let node_weight = self.graph.nodes.get_mut(a.index())?.weight.take()?;

        for d in EdgeDirection::ALL {
            let k = d as usize;

            // Remove all edges from and to this node.
            loop {
                let next = self.graph.nodes[a.index()].next[k];
                if next == EdgeIndex::END {
                    break;
                }
                edge_callback(self, next);
                self.remove_edge(next).expect("edge not found for removal");
            }
        }

        // Clear the node and free its ID.
        let node_slot = &mut self.graph.nodes[a.index()];
        node_slot.next = [EdgeIndex::END; 2];

        self.node_ids.free(a.0);

        Some(node_weight)
    }

    /// Removes an edge and returns its edge weight, or `None` if it didn't exist.
    ///
    /// Invalidates the edge index `e`, but not any other indices.
    ///
    /// Computes in **O(e')** time, where **e'** is the size of four particular edge lists, for
    /// the vertices of `e` and the vertices of another affected edge.
    pub fn remove_edge(&mut self, e: EdgeIndex) -> Option<E> {
        // Every edge is part of two lists, outgoing and incoming edges.
        // Remove it from both.
        let (is_edge, edge_node, edge_next) = match self.graph.edges.get(e.index()) {
            None => return None,
            Some(x) => (x.weight.is_some(), x.node, x.next),
        };

        if !is_edge {
            return None;
        }

        // Remove the edge from its in and out lists by replacing it with
        // a link to the next in the list.
        self.graph.change_edge_links(edge_node, e, edge_next);

        // Clear the edge and free its ID.
        let edge_slot = &mut self.graph.edges[e.index()];
        edge_slot.next = [EdgeIndex::END; 2];
        edge_slot.node = [NodeIndex::END; 2];

        self.edge_ids.free(e.0);

        edge_slot.weight.take()
    }

    /// Returns an iterator of all nodes with an edge connected to `a`.
    ///
    /// Produces an empty iterator if the node doesn't exist.
    ///
    /// The iterator element type is `NodeIndex`.
    pub fn neighbors(&self, a: NodeIndex) -> Neighbors<'_, E> {
        Neighbors {
            skip_start: a,
            edges: &self.graph.edges,
            next: match self.get_node(a) {
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
            edges: &self.graph.edges,
            direction: EdgeDirection::Outgoing,
            next: match self.get_node(a) {
                None => [EdgeIndex::END, EdgeIndex::END],
                Some(n) => n.next,
            },
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
        self.graph.find_edge_from_node(self.get_node(a)?, b)
    }

    /// Returns an iterator yielding immutable access to edge weights for edges from or to `a`.
    pub fn edge_weights(&self, a: NodeIndex) -> EdgeWeights<'_, E> {
        EdgeWeights {
            skip_start: a,
            edges: &self.graph.edges,
            direction: EdgeDirection::Outgoing,
            next: match self.get_node(a) {
                None => [EdgeIndex::END, EdgeIndex::END],
                Some(n) => n.next,
            },
        }
    }

    /// Returns an iterator yielding mutable access to edge weights for edges from or to `a`.
    pub fn edge_weights_mut(&mut self, a: NodeIndex) -> EdgeWeightsMut<'_, N, E>
    where
        N: Copy,
    {
        self.graph
            .edge_weights_mut(a)
            .filter_map(|edge| edge.as_mut())
    }

    /// Returns an iterator yielding immutable access to all edge weights.
    ///
    /// The order in which weights are yielded matches the order of their
    /// edge indices.
    pub fn all_edge_weights(&self) -> impl Iterator<Item = &E> {
        self.graph
            .edges
            .iter()
            .filter_map(|edge| edge.weight.as_ref())
    }

    /// Returns an iterator yielding mutable access to all edge weights.
    ///
    /// The order in which weights are yielded matches the order of their
    /// edge indices.
    pub fn all_edge_weights_mut(&mut self) -> impl Iterator<Item = &mut E> {
        self.graph
            .edges
            .iter_mut()
            .filter_map(|edge| edge.weight.as_mut())
    }

    /// Accesses the internal edge array.
    ///
    /// Note that this also includes vacant edges.
    pub fn raw_edges(&self) -> &[Edge<Option<E>>] {
        &self.graph.edges
    }

    /// Accesses the internal edge array mutably.
    ///
    /// Note that this also includes vacant edges.
    pub fn raw_edges_mut(&mut self) -> &mut [Edge<Option<E>>] {
        &mut self.graph.edges
    }

    /// Removes all nodes and edges.
    pub fn clear(&mut self) {
        self.graph.clear();
        self.node_ids.clear();
        self.edge_ids.clear();
    }

    /// Removes all edges.
    pub fn clear_edges(&mut self) {
        self.graph.edges.clear();
        self.edge_ids.clear();

        // Clear edge links for all nodes.
        for node in &mut self.graph.nodes {
            if node.weight.is_some() {
                node.next = [EdgeIndex::END, EdgeIndex::END];
            }
        }
    }

    /// Returns the current node capacity of the graph.
    pub fn nodes_capacity(&self) -> usize {
        self.graph.nodes_capacity()
    }

    /// Returns the current edge capacity of the graph.
    pub fn edges_capacity(&self) -> usize {
        self.graph.edges_capacity()
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
    edges: &'a [Edge<Option<E>>],

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
                debug_assert!(edge.weight.is_some());
                self.next[0] = edge.next[0];
                return Some(edge.node[1]);
            }
        }

        // Then incoming edges.
        // For an "undirected" iterator, make sure we don't double
        // count self-loops by skipping them in the incoming list.
        while let Some(edge) = self.edges.get(self.next[1].index()) {
            debug_assert!(edge.weight.is_some());
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
    edges: &'a [Edge<Option<E>>],

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
            node,
            weight: Some(weight),
            next,
            ..
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
        while let Some(Edge {
            node,
            weight: Some(weight),
            next,
        }) = self.edges.get(self.next[1].index())
        {
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
    edges: &'a [Edge<Option<E>>],

    /// The next edge to visit.
    next: [EdgeIndex; 2],

    /// The direction of edges.
    direction: EdgeDirection,
}

impl<'a, E> Iterator for EdgeWeights<'a, E> {
    type Item = &'a E;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.next[0].index();
        if let Some(Edge {
            weight: Some(weight),
            next,
            ..
        }) = self.edges.get(i)
        {
            self.next[0] = next[0];
            return Some(weight);
        }

        while let Some(Edge {
            node,
            weight: Some(weight),
            next,
        }) = self.edges.get(self.next[1].index())
        {
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
type EdgeWeightsMut<'a, N, E> = core::iter::FilterMap<
    super::graph::EdgeWeightsMut<'a, Option<N>, Option<E>>,
    fn(&mut Option<E>) -> Option<&mut E>,
>;

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
