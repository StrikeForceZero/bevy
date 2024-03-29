use crate::layout::convert;
use crate::{LayoutContext, LayoutError, Style};
use bevy_ecs::entity::{Entity, EntityHashMap, EntityHashSet};
use bevy_ecs::prelude::Resource;
use bevy_hierarchy::Children;
use bevy_math::UVec2;
use bevy_utils::default;
use bevy_utils::tracing::warn;
use std::fmt;
use std::fmt::{Debug, Display, Formatter};
use taffy::prelude::LayoutTree;
use taffy::Taffy;

#[inline(always)]
fn default_viewport_style() -> taffy::style::Style {
    taffy::style::Style {
        display: taffy::style::Display::Grid,
        // Note: Taffy percentages are floats ranging from 0.0 to 1.0.
        // So this is setting width:100% and height:100%
        size: taffy::geometry::Size {
            width: taffy::style::Dimension::Percent(1.0),
            height: taffy::style::Dimension::Percent(1.0),
        },
        align_items: Some(taffy::style::AlignItems::Start),
        justify_items: Some(taffy::style::JustifyItems::Start),
        ..default()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UiNodeMeta {
    pub(super) camera_entity: Option<Entity>,
    pub(super) root_node_pair: RootNodePair,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RootNodePair {
    // The implicit "viewport" node created by Bevy
    pub(super) implicit_viewport_node: taffy::node::Node,
    // The root (parentless) node specified by the user
    pub(super) user_root_node: taffy::node::Node,
}

#[derive(Resource)]
pub struct UiSurface {
    pub(super) entity_to_taffy: EntityHashMap<taffy::node::Node>,
    pub(super) ui_root_node_meta: EntityHashMap<UiNodeMeta>,
    pub(super) camera_root_nodes: EntityHashMap<EntityHashSet>,
    pub(super) taffy: Taffy,
}

fn _assert_send_sync_ui_surface_impl_safe() {
    fn _assert_send_sync<T: Send + Sync>() {}
    _assert_send_sync::<EntityHashMap<taffy::node::Node>>();
    _assert_send_sync::<EntityHashMap<UiNodeMeta>>();
    _assert_send_sync::<EntityHashMap<EntityHashSet>>();
    _assert_send_sync::<Taffy>();
    _assert_send_sync::<UiSurface>();
}

impl fmt::Debug for UiSurface {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("UiSurface")
            .field("entity_to_taffy", &self.entity_to_taffy)
            .field("ui_root_node_meta", &self.ui_root_node_meta)
            .field("camera_root_nodes", &self.camera_root_nodes)
            .finish()
    }
}

impl Default for UiSurface {
    fn default() -> Self {
        let mut taffy = Taffy::new();
        taffy.disable_rounding();
        Self {
            entity_to_taffy: Default::default(),
            ui_root_node_meta: Default::default(),
            camera_root_nodes: Default::default(),
            taffy,
        }
    }
}

impl UiSurface {
    /// Retrieves the Taffy node associated with the given UI node entity and updates its style.
    /// If no associated Taffy node exists a new Taffy node is inserted into the Taffy layout.
    pub fn upsert_node(
        &mut self,
        camera_entity: Entity,
        ui_node_entity: Entity,
        style: &Style,
        context: &LayoutContext,
        has_parent: bool,
    ) {
        // println!("upsert: {ui_node_entity}, camera: {camera_entity}, has_parent: {has_parent}");
        let mut added = false;
        let taffy_node = *self.entity_to_taffy.entry(ui_node_entity).or_insert_with(|| {
            added = true;
            self
                .taffy
                .new_leaf(convert::from_style(context, style))
                .unwrap()
        });

        if !has_parent {
            self.create_or_update_ui_meta(&ui_node_entity, &camera_entity);
        }

        if !added {
            self.taffy
                .set_style(
                    taffy_node,
                    convert::from_style(context, style),
                )
                .unwrap();
        }
    }

    /// Disassociates the camera from all of its assigned root nodes and removes their viewport nodes
    /// Removes entry in camera_root_nodes
    pub(super) fn remove_camera(&mut self, camera_entity: &Entity) {
        if let Some(root_node_entities) = self.camera_root_nodes.remove(camera_entity) {
            for root_node_entity in root_node_entities {
                self.remove_root_node_viewport(&root_node_entity);
            }
        };
    }

    /// Disassociates the root node from the assigned camera (if any) and removes the viewport node from taffy
    /// Removes entry in ui_root_node_meta
    pub(super) fn remove_root_node_viewport(&mut self, ui_root_node_entity: &Entity) {
        if let Some(mut removed) = self.ui_root_node_meta.remove(ui_root_node_entity) {
            if let Some(camera_entity) = removed.camera_entity.take() {
                if let Some(root_node_entities) = self.camera_root_nodes.get_mut(&camera_entity) {
                    root_node_entities.remove(ui_root_node_entity);
                }
            }
            self.taffy.remove(removed.root_node_pair.implicit_viewport_node).unwrap();
        }
    }

    /// Removes the ui node from the taffy tree, and if it's a root node it also calls remove_root_node_viewport
    pub(super) fn remove_ui_node(&mut self, ui_node_entity: &Entity) {
        self.remove_root_node_viewport(ui_node_entity);
        if let Some(taffy_node) = self.entity_to_taffy.remove(ui_node_entity) {
            self.taffy.remove(taffy_node).unwrap();
        }
        // remove root node entry if this is a root node
        if self.ui_root_node_meta.contains_key(ui_node_entity) {
            self.remove_root_node_viewport(ui_node_entity);
        }
    }

    /// Update the `MeasureFunc` of the taffy node corresponding to the given [`Entity`] if the node exists.
    pub fn try_update_measure(
        &mut self,
        entity: Entity,
        measure_func: taffy::node::MeasureFunc,
    ) -> Option<()> {
        let taffy_node = *self.entity_to_taffy.get(&entity)?;

        self.taffy.set_measure(taffy_node, Some(measure_func)).ok()
    }
    
    pub(super) fn demote_ui_node(&mut self, target_entity: &Entity, parent_entity: &Entity) {
        if let Some(mut ui_meta) = self.ui_root_node_meta.remove(target_entity) {
            if let Some(camera_entity) = ui_meta.camera_entity.take() {
                if let Some(ui_set) = self.camera_root_nodes.get_mut(&camera_entity) {
                    ui_set.remove(target_entity);
                }
            }
            self.taffy.remove(ui_meta.root_node_pair.implicit_viewport_node).unwrap();
            let parent_taffy = self.entity_to_taffy.get(parent_entity).unwrap();
            self.taffy.add_child(*parent_taffy, ui_meta.root_node_pair.user_root_node).unwrap();
        }
    }

    pub(super) fn promote_ui_node(&mut self, target_entity: &Entity) {
        self.ui_root_node_meta.entry(*target_entity)
            .or_insert_with(|| {
                let user_root_node = *self.entity_to_taffy.get(target_entity).unwrap();
                let implicit_viewport_node = self.taffy.new_with_children(default_viewport_style(), &[user_root_node]).unwrap();
                UiNodeMeta {
                    camera_entity: None,
                    root_node_pair: RootNodePair {
                        implicit_viewport_node,
                        user_root_node,
                    }
                }
            });
    }

    /// Update the children of the taffy node corresponding to the given [`Entity`].
    pub fn update_children(&mut self, entity: Entity, children: &Children) {
        println!("update_children: {entity} children: {children:?}");
        let mut taffy_children = Vec::with_capacity(children.len());
        for child in children {
            if let Some(taffy_node) = self.entity_to_taffy.get(child) {
                taffy_children.push(*taffy_node);
            } else {
                warn!(
                    "Unstyled child in a UI entity hierarchy. You are using an entity \
without UI components as a child of an entity with UI components, results may be unexpected."
                );
            }
        }
        let taffy_node = self.entity_to_taffy.get(&entity).unwrap();
        self.taffy
            .set_children(*taffy_node, &taffy_children)
            .unwrap();
    }

    // TODO: cant use impl IntoIterator<Item=&'a Entity> because no len
    #[cfg(test)]
    pub fn _update_children(&mut self, entity: Entity, children: &[Entity]) {
        let mut taffy_children = Vec::with_capacity(children.len());
        for child in children {
            if let Some(taffy_node) = self.entity_to_taffy.get(child) {
                taffy_children.push(*taffy_node);
            } else {
                warn!(
                    "Unstyled child in a UI entity hierarchy. You are using an entity \
without UI components as a child of an entity with UI components, results may be unexpected."
                );
            }
        }

        let taffy_node = self.entity_to_taffy.get(&entity).unwrap();
        self.taffy
            .set_children(*taffy_node, &taffy_children)
            .unwrap();
    }

    fn remove_children_recursive(&mut self, node: taffy::node::Node) {
        let Ok(children) = self.taffy.children(node) else {
            return;
        };
        let mut children_to_remove = children;
        while !children_to_remove.is_empty() {
            let Some(child) = children_to_remove.pop() else {
                continue;
            };
            if let Ok(mut children) = self.taffy.children(child) {
                children_to_remove.append(&mut children);
            }
            println!("removing child: {child:?}");
            self.taffy.remove(child).unwrap();
        }
    }

    /// Removes children from the entity's taffy node if it exists. Does nothing otherwise.
    pub fn try_remove_children(&mut self, entity: Entity) {
        if let Some(taffy_node) = self.entity_to_taffy.get(&entity) {
            self.taffy.set_children(*taffy_node, &[]).unwrap();
        }
    }

    /// Removes the measure from the entity's taffy node if it exists. Does nothing otherwise.
    pub fn try_remove_measure(&mut self, entity: Entity) {
        if let Some(taffy_node) = self.entity_to_taffy.get(&entity) {
            self.taffy.set_measure(*taffy_node, None).unwrap();
        }
    }

    fn mark_ui_node_as_orphaned(&mut self, ui_entity: &Entity) {
        if let Some(meta) = self.ui_root_node_meta.get_mut(ui_entity) {
            // mark it as orphaned
            if let Some(old_camera_entity) = meta.camera_entity.take() {
                if let Some(previous_siblings) = self.camera_root_nodes.get_mut(&old_camera_entity) {
                    previous_siblings.remove(ui_entity);
                }
            }
        }
    }

    fn reset_relationship(&mut self, root_node_entity: &Entity) {
        self.clear_relationship(root_node_entity);
        if let Some(ui_meta) = self.ui_root_node_meta.get(root_node_entity) {
            self.taffy.add_child(ui_meta.root_node_pair.implicit_viewport_node, ui_meta.root_node_pair.implicit_viewport_node).unwrap();
        }
    }

    fn clear_relationship(&mut self, root_node_entity: &Entity) {
        if let Some(ui_meta) = self.ui_root_node_meta.get(root_node_entity) {
            if let Some(parent) = self.taffy.parent(ui_meta.root_node_pair.user_root_node) {
                self.taffy.remove_child(parent, ui_meta.root_node_pair.user_root_node).unwrap();
            }
        }
    }

    fn create_or_update_ui_meta(&mut self, ui_root_node_entity: &Entity, camera_entity: &Entity) -> &mut UiNodeMeta {
        let user_root_node = *self.entity_to_taffy.get(ui_root_node_entity).expect("create_meta called before ui_root_node_entity was added to taffy tree or was previously removed");
        let ui_root_node_entity = *ui_root_node_entity;
        let camera_entity = *camera_entity;

        let mut added = false;
        let ui_node_meta = self
            .ui_root_node_meta
            .entry(ui_root_node_entity)
            .or_insert_with(|| {
                added = true;

                self.camera_root_nodes
                    .entry(camera_entity)
                    .or_default()
                    .insert(ui_root_node_entity);

                let implicit_viewport_node = self.taffy.new_leaf(default_viewport_style()).unwrap();

                self.taffy
                    .add_child(implicit_viewport_node, user_root_node)
                    .unwrap();

                UiNodeMeta {
                    camera_entity: Some(camera_entity),
                    root_node_pair: RootNodePair {
                        implicit_viewport_node,
                        user_root_node,
                    },
                }
            });

        if !added {
            let option_old_camera_entity = ui_node_meta.camera_entity.replace(camera_entity);
            // if we didn't insert, lets check to make the camera reference is the same
            if Some(camera_entity) != option_old_camera_entity {
                if let Some(old_camera_entity) = option_old_camera_entity {
                    // camera reference is not the same so remove it from the old set
                    if let Some(root_node_set) = self.camera_root_nodes.get_mut(&old_camera_entity) {
                        root_node_set.remove(&ui_root_node_entity);
                    }
                }

                self.camera_root_nodes
                    .entry(camera_entity)
                    .or_default()
                    .insert(ui_root_node_entity);
            }
        }

        ui_node_meta
    }

    /// Set the ui node entities without a [`Parent`] as children to the root node in the taffy layout.
    pub fn set_camera_children(
        &mut self,
        camera_entity: Entity,
        children: impl Iterator<Item = Entity> + Debug + Clone,
    ) {
        // self.camera_to_ui_set.entry(camera_entity).or_default().clear();
        let removed_children = self.camera_root_nodes.entry(camera_entity).or_default();
        let mut removed_children = removed_children.clone();

        for ui_entity in children {
            let ui_meta = self.create_or_update_ui_meta(&ui_entity, &camera_entity);

            if let Some(old_camera) = ui_meta.camera_entity.replace(camera_entity) {
                if old_camera != camera_entity {
                    if let Some(old_siblings_set) = self.camera_root_nodes.get_mut(&old_camera) {
                        old_siblings_set.remove(&ui_entity);
                    }
                    // self.taffy.set_style(ui_meta.root_node_pair.implicit_viewport_node, default_viewport_style()).unwrap();
                }
            }
            let Some(ui_meta) = self.ui_root_node_meta.get_mut(&ui_entity) else {
                unreachable!("impossible unless ui_meta was removed from map above");
            };

            // fix taffy relationships
            {
                if let Some(parent) = self.taffy.parent(ui_meta.root_node_pair.user_root_node) {
                    self.taffy
                        .remove_child(parent, ui_meta.root_node_pair.user_root_node)
                        .unwrap();
                }

                self.taffy
                    .add_child(
                        ui_meta.root_node_pair.implicit_viewport_node,
                        ui_meta.root_node_pair.user_root_node,
                    )
                    .unwrap();
            }

            self.camera_root_nodes
                .entry(camera_entity)
                .or_default()
                .insert(ui_entity);

            removed_children.remove(&ui_entity);
        }

       for orphan in removed_children.iter() {
           if let Some(ui_meta) = self.ui_root_node_meta.get_mut(orphan) {
               // mark as orphan
               if let Some(camera_entity) = ui_meta.camera_entity.take() {
                   if let Some(children_set) = self.camera_root_nodes.get_mut(&camera_entity) {
                       children_set.remove(orphan);
                   }
               }
               println!("implicit_viewport_node: {:?} children: {:?}", ui_meta.root_node_pair.implicit_viewport_node, self.taffy.child_count(ui_meta.root_node_pair.implicit_viewport_node).unwrap());
               println!("root_node_pair: {:?} children: {:?}", ui_meta.root_node_pair.user_root_node, self.taffy.child_count(ui_meta.root_node_pair.user_root_node).unwrap());
               // self.taffy.set_children(ui_meta.root_node_pair.implicit_viewport_node, &[]).unwrap();
           }
       }
    }

    /// Compute the layout for each window entity's corresponding root node in the layout.
    pub fn compute_camera_layout(
        &mut self,
        camera_entity: &Entity,
        render_target_resolution: UVec2,
    ) {
        let Some(root_nodes) = self.camera_root_nodes.get(camera_entity) else {
            return;
        };
        for &root_node_entity in root_nodes.iter() {
            let available_space = taffy::geometry::Size {
                width: taffy::style::AvailableSpace::Definite(render_target_resolution.x as f32),
                height: taffy::style::AvailableSpace::Definite(render_target_resolution.y as f32),
            };

            let Some(ui_meta) = self.ui_root_node_meta.get(&root_node_entity) else {
                continue;
            };
            if ui_meta.camera_entity.is_none() {
                panic!("internal map out of sync");
            }

            self.taffy
                .compute_layout(
                    ui_meta.root_node_pair.implicit_viewport_node,
                    available_space,
                )
                .unwrap();
        }
    }

    /// Removes specified camera entities by disassociating them from their associated `implicit_viewport_node`
    /// in the internal map, and subsequently removes the `implicit_viewport_node`
    /// from the `taffy` layout engine for each.
    pub fn remove_camera_entities(&mut self, entities: impl IntoIterator<Item = Entity> + Debug) {
        for entity in entities {
            self.remove_camera(&entity);
        }
    }

    /// Removes the specified entities from the internal map while
    /// removing their `implicit_viewport_node` from taffy,
    /// and then subsequently removes their entry from `entity_to_taffy` and associated node from taffy
    pub fn remove_entities(&mut self, entities: impl IntoIterator<Item = Entity> + Debug) {
        for entity in entities {
            self.remove_ui_node(&entity);
        }
    }

    /// Get the layout geometry for the taffy node corresponding to the ui node [`Entity`].
    /// Does not compute the layout geometry, `compute_window_layouts` should be run before using this function.
    pub fn get_layout(&self, entity: Entity) -> Result<&taffy::layout::Layout, LayoutError> {
        if let Some(taffy_node) = self.entity_to_taffy.get(&entity) {
            self.taffy
                .layout(*taffy_node)
                .map_err(LayoutError::TaffyError)
        } else {
            warn!(
                "Styled child in a non-UI entity hierarchy. You are using an entity \
with UI components as a child of an entity without UI components, results may be unexpected."
            );
            Err(LayoutError::InvalidHierarchy)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ContentSize, FixedMeasure};
    use bevy_math::Vec2;
    #[test]
    fn test_initialization() {
        let ui_surface = UiSurface::default();
        assert!(ui_surface.camera_root_nodes.is_empty());
        assert!(ui_surface.ui_root_node_meta.is_empty());
        assert_eq!(ui_surface.taffy.total_node_count(), 0);
    }

    const DUMMY_LAYOUT_CONTEXT: LayoutContext = LayoutContext {
        scale_factor: 1.0,
        physical_size: Vec2::ONE,
        min_size: 0.0,
        max_size: 1.0,
    };

    fn is_root_node_pair_valid(taffy: &Taffy, root_node_pair: &RootNodePair) -> bool {
        taffy.parent(root_node_pair.user_root_node) == Some(root_node_pair.implicit_viewport_node)
    }

    #[test]
    fn test_upsert() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        // standard upsert
        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );

        assert_eq!(ui_surface.taffy.total_node_count(), 2);
        let meta = ui_surface.ui_root_node_meta.get(&root_node_entity).unwrap();
        assert_eq!(meta.camera_entity, Some(camera_entity));
        assert!(is_root_node_pair_valid(
            &ui_surface.taffy,
            &meta.root_node_pair
        ));
        // test duplicate insert

        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );

        assert_eq!(ui_surface.taffy.total_node_count(), 2);
        let meta = ui_surface.ui_root_node_meta.get(&root_node_entity).unwrap();
        assert_eq!(meta.camera_entity, Some(camera_entity));
        assert!(is_root_node_pair_valid(
            &ui_surface.taffy,
            &meta.root_node_pair
        ));

        // test overwrite
        let camera_entity2 = Entity::from_raw(2);
        ui_surface.upsert_node(
            camera_entity2,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );

        assert_eq!(ui_surface.taffy.total_node_count(), 2);
        let meta = ui_surface.ui_root_node_meta.get(&root_node_entity).unwrap();
        assert_eq!(meta.camera_entity, Some(camera_entity2));
        assert!(is_root_node_pair_valid(
            &ui_surface.taffy,
            &meta.root_node_pair
        ));
    }

    #[test]
    fn test_remove_camera() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );
        assert!(ui_surface.ui_root_node_meta.contains_key(&root_node_entity));
        assert!(ui_surface.camera_root_nodes.contains_key(&camera_entity));

        ui_surface.remove_camera(&camera_entity);
        assert!(!ui_surface.ui_root_node_meta.contains_key(&root_node_entity));
        assert!(!ui_surface.camera_root_nodes.contains_key(&camera_entity));
    }

    #[test]
    fn test_remove_root_node() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );
        assert!(ui_surface.ui_root_node_meta.contains_key(&root_node_entity));

        ui_surface.remove_ui_node(&root_node_entity);
        assert!(!ui_surface.ui_root_node_meta.contains_key(&root_node_entity));
    }

    #[test]
    fn test_try_update_measure() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );
        let mut content_size = ContentSize::default();
        content_size.set(FixedMeasure { size: Vec2::ONE });
        let measure_func = content_size.measure_func.take().unwrap();
        assert!(ui_surface
            .try_update_measure(root_node_entity, measure_func)
            .is_some());
    }

    #[test]
    fn test_update_children() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let child_entity = Entity::from_raw(2);
        let style = Style::default();
        let children = [child_entity];

        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );
        ui_surface.upsert_node(camera_entity, child_entity, &style, &DUMMY_LAYOUT_CONTEXT, true);
        ui_surface._update_children(root_node_entity, &children);

        let taffy_node = ui_surface
            .entity_to_taffy.get(&root_node_entity)
            .unwrap();
        assert_eq!(ui_surface.taffy.children(*taffy_node).unwrap().len(), 1);
    }

    #[test]
    fn test_set_camera_children() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let child_entity = Entity::from_raw(2);
        let style = Style::default();

        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );
        ui_surface.upsert_node(camera_entity, child_entity, &style, &DUMMY_LAYOUT_CONTEXT, true);

        let root_taffy_node = ui_surface.ui_root_node_meta.get(&root_node_entity).unwrap().root_node_pair.user_root_node;
        let child_taffy = ui_surface.entity_to_taffy.get(&child_entity).unwrap();
        // set up the relationship manually
        ui_surface.taffy.add_child(root_taffy_node, *child_taffy).unwrap();

        assert!(ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&root_node_entity), "root node not associated with camera");
        assert!(!ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&child_entity), "child of root node should not be associated with camera");

        let root_taffy_node = ui_surface.ui_root_node_meta.get(&root_node_entity).unwrap().root_node_pair.user_root_node;
        let child_taffy = ui_surface.entity_to_taffy.get(&child_entity).unwrap();
        let root_taffy_children = ui_surface.taffy.children(root_taffy_node).unwrap();
        assert!(root_taffy_children.contains(child_taffy), "root node children do not contain: {child_taffy:?}");
        assert_eq!(ui_surface.taffy.child_count(root_taffy_node).unwrap(), 1, "expected root node child count to be 1");

        ui_surface.set_camera_children(camera_entity, Vec::<Entity>::new().into_iter());

        assert!(!ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&root_node_entity), "root node should have been unassociated with camera");
        assert!(!ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&child_entity), "child of root node should not be associated with camera");

        let root_taffy_node = ui_surface.ui_root_node_meta.get(&root_node_entity).unwrap().root_node_pair.user_root_node;
        let child_taffy = ui_surface.entity_to_taffy.get(&child_entity).unwrap();
        let root_taffy_children = ui_surface.taffy.children(root_taffy_node).unwrap();
        assert!(root_taffy_children.contains(child_taffy), "root node children do not contain: {child_taffy:?}");
        assert_eq!(ui_surface.taffy.child_count(root_taffy_node).unwrap(), 1, "expected root node child count to be 1");

        ui_surface.set_camera_children(camera_entity, vec![root_node_entity].into_iter());


        assert!(ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&root_node_entity), "root node should have been re-associated with camera");
        assert!(!ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&child_entity), "child of root node should not be associated with camera");

        let root_taffy_node = ui_surface.ui_root_node_meta.get(&root_node_entity).unwrap().root_node_pair.user_root_node;
        let child_taffy = ui_surface.entity_to_taffy.get(&child_entity).unwrap();
        let root_taffy_children = ui_surface.taffy.children(root_taffy_node).unwrap();
        assert!(root_taffy_children.contains(child_taffy), "root node children do not contain: {child_taffy:?}");
        assert_eq!(ui_surface.taffy.child_count(root_taffy_node).unwrap(), 1, "expected root node child count to be 1");
    }

    #[test]
    fn test_compute_camera_layout() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(
            camera_entity,
            root_node_entity,
            &style,
            &DUMMY_LAYOUT_CONTEXT,
            false,
        );
        ui_surface.compute_camera_layout(&camera_entity, UVec2::new(800, 600));

        let taffy_node = ui_surface
            .entity_to_taffy.get(&root_node_entity)
            .unwrap();
        assert!(ui_surface.taffy.layout(*taffy_node).is_ok());
    }
}
