use bevy::{platform::collections::HashMap, prelude::*};

use super::{Collider, ColliderConstructor};

/// A plugin for caching colliders created from meshes via [`ColliderConstructor`] or [`ColliderConstructorHierarchy`](super::ColliderConstructorHierarchy).
/// With this plugin enabled, colliders created from meshes through such constructors will be created only once and reused.
/// This is especially useful when performing convex decomposition, as this is a very expensive operation.
pub struct ColliderCachePlugin;

impl Plugin for ColliderCachePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ColliderCache>();
        app.add_systems(PreUpdate, clear_unused_colliders);
    }
}

#[derive(Debug, Resource, Default)]
pub(crate) struct ColliderCache(HashMap<AssetId<Mesh>, Vec<(ColliderConstructor, Collider)>>);

impl ColliderCache {
    pub(crate) fn get_or_insert(
        &mut self,
        mesh_handle: &Handle<Mesh>,
        mesh: &Mesh,
        constructor: ColliderConstructor,
    ) -> Option<Collider> {
        let id = mesh_handle.id();
        let Some(entries) = self.0.get_mut(&id) else {
            let collider = Collider::try_from_constructor(constructor.clone(), Some(mesh))?;
            self.0
                .insert(id, vec![(constructor.clone(), collider.clone())]);
            return Some(collider);
        };
        if let Some((_ctor, collider)) = entries.iter().find(|(c, _)| c == &constructor) {
            Some(collider.clone())
        } else {
            let collider = Collider::try_from_constructor(constructor.clone(), Some(mesh))?;
            entries.push((constructor.clone(), collider.clone()));
            Some(collider)
        }
    }
}

fn clear_unused_colliders(
    mut asset_events: MessageReader<AssetEvent<Mesh>>,
    mut collider_cache: ResMut<ColliderCache>,
) {
    for event in asset_events.read() {
        if let AssetEvent::Removed { id } | AssetEvent::Unused { id } = event
            && collider_cache.0.contains_key(id)
        {
            collider_cache.0.remove(id);
        }
    }
}
