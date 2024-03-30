use std::fmt;

use taffy::prelude::LayoutTree;
use taffy::Taffy;

use bevy_ecs::entity::{Entity, EntityHashMap, EntityHashSet};
use bevy_ecs::prelude::Resource;
use bevy_math::UVec2;
use bevy_utils::default;
use bevy_utils::tracing::warn;

use crate::layout::convert;
use crate::{LayoutContext, LayoutError, Style};

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
pub struct RootNodeData {
    pub(super) camera_entity: Option<Entity>,
    // The implicit "viewport" node created by Bevy
    pub(super) implicit_viewport_node: taffy::node::Node,
}

#[derive(Resource)]
pub struct UiSurface {
    pub(super) entity_to_taffy: EntityHashMap<taffy::node::Node>,
    pub(super) root_node_data: EntityHashMap<RootNodeData>,
    pub(super) camera_root_nodes: EntityHashMap<EntityHashSet>,
    pub(super) taffy: Taffy,
}

fn _assert_send_sync_ui_surface_impl_safe() {
    fn _assert_send_sync<T: Send + Sync>() {}
    _assert_send_sync::<EntityHashMap<taffy::node::Node>>();
    _assert_send_sync::<EntityHashMap<RootNodeData>>();
    _assert_send_sync::<EntityHashMap<EntityHashSet>>();
    _assert_send_sync::<Taffy>();
    _assert_send_sync::<UiSurface>();
}

impl fmt::Debug for UiSurface {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("UiSurface")
            .field("entity_to_taffy", &self.entity_to_taffy)
            .field("root_node_data", &self.root_node_data)
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
            root_node_data: Default::default(),
            camera_root_nodes: Default::default(),
            taffy,
        }
    }
}

impl UiSurface {
    /// Retrieves the Taffy node associated with the given UI node entity and updates its style.
    /// If no associated Taffy node exists a new Taffy node is inserted into the Taffy layout.
    pub fn upsert_node(&mut self, ui_node_entity: Entity, style: &Style, context: &LayoutContext) {
        let mut added = false;
        let taffy_node = *self
            .entity_to_taffy
            .entry(ui_node_entity)
            .or_insert_with(|| {
                added = true;
                self.taffy
                    .new_leaf(convert::from_style(context, style))
                    .unwrap()
            });

        if !added {
            self.taffy
                .set_style(taffy_node, convert::from_style(context, style))
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
    /// Removes entry in `root_node_data`
    pub(super) fn remove_root_node_viewport(&mut self, root_node_entity: &Entity) {
        self.mark_root_node_as_orphaned(root_node_entity);
        if let Some(removed) = self.root_node_data.remove(root_node_entity) {
            self.taffy.remove(removed.implicit_viewport_node).unwrap();
        }
    }

    /// Removes the ui node from the taffy tree, and if it's a root node it also calls `remove_root_node_viewport`
    pub(super) fn remove_ui_node(&mut self, ui_node_entity: &Entity) {
        self.remove_root_node_viewport(ui_node_entity);
        if let Some(taffy_node) = self.entity_to_taffy.remove(ui_node_entity) {
            self.taffy.remove(taffy_node).unwrap();
        }
    }

    /// Demotes root node to a child node of the specified parent
    pub(super) fn demote_ui_node(&mut self, target_entity: &Entity, parent_entity: &Entity) {
        // remove camera association
        self.mark_root_node_as_orphaned(target_entity);

        if let Some(root_node_data) = self.root_node_data.remove(target_entity) {
            self.taffy
                .remove(root_node_data.implicit_viewport_node)
                .unwrap();
            let parent_taffy = self.entity_to_taffy.get(parent_entity).unwrap();
            let child_taffy = self.entity_to_taffy.get(target_entity).unwrap();
            self.taffy.add_child(*parent_taffy, *child_taffy).unwrap();
        }
    }

    #[cfg(test)]
    /// Converts ui node to root node
    /// Should only be used for testing - does not set `TargetCamera`
    pub(super) fn promote_ui_node(&mut self, target_entity: &Entity, camera_entity: &Entity) {
        let taffy_node = self.entity_to_taffy.get(target_entity).unwrap();

        // clear the parent - new_with_children doesn't seem to notify the old parent its children have changed
        if let Some(parent) = self.taffy.parent(*taffy_node) {
            self.taffy.remove_child(parent, *taffy_node).unwrap();
        }

        let mut added = false;
        self.root_node_data
            .entry(*target_entity)
            .or_insert_with(|| {
                added = true;
                let user_root_node = *self.entity_to_taffy.get(target_entity).unwrap();
                let implicit_viewport_node = self
                    .taffy
                    .new_with_children(default_viewport_style(), &[user_root_node])
                    .unwrap();
                RootNodeData {
                    camera_entity: None,
                    implicit_viewport_node,
                }
            });

        self.replace_camera_association(*target_entity, Some(*camera_entity));

        // if it's not added we want to re-assign the parent we cleared above
        if !added {
            let Some(root_node_data) = self.root_node_data.get(target_entity) else {
              unreachable!("impossible");
            };
            let taffy_node = self.entity_to_taffy.get(target_entity).unwrap();
            self.taffy.add_child(root_node_data.implicit_viewport_node, *taffy_node).unwrap();
        }
    }

    /// Update the `MeasureFunc` of the taffy node corresponding to the given [`Entity`] if the node exists.
    pub fn try_update_measure(
        &mut self,
        entity: Entity,
        measure_func: taffy::node::MeasureFunc,
    ) -> Option<()> {
        let taffy_node = self.entity_to_taffy.get(&entity)?;

        self.taffy.set_measure(*taffy_node, Some(measure_func)).ok()
    }

    /// Update the children of the taffy node corresponding to the given [`Entity`].
    pub fn update_children(&mut self, entity: Entity, children: &[Entity]) {
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

    /// Removes camera association to root node
    /// Shorthand for calling `replace_camera_association(root_node_entity, None)`
    fn mark_root_node_as_orphaned(&mut self, root_node_entity: &Entity) {
        self.replace_camera_association(*root_node_entity, None);
    }

    /// `Some(camera_entity)` - Updates camera association to root node
    /// `None` - Removes camera association to root node
    /// Does not check to see if they are the same before performing operations
    fn replace_camera_association(
        &mut self,
        root_node_entity: Entity,
        new_camera_entity_option: Option<Entity>,
    ) {
        if let Some(root_node_data) = self.root_node_data.get_mut(&root_node_entity) {
            // Clear existing camera association, if any
            if let Some(old_camera_entity) = root_node_data.camera_entity.take() {
                let prev_camera_root_nodes = self.camera_root_nodes.get_mut(&old_camera_entity);
                if let Some(prev_camera_root_nodes) = prev_camera_root_nodes {
                    prev_camera_root_nodes.remove(&root_node_entity);
                }
            }

            // Establish new camera association, if provided
            if let Some(camera_entity) = new_camera_entity_option {
                root_node_data.camera_entity.replace(camera_entity);
                self.camera_root_nodes
                    .entry(camera_entity)
                    .or_default()
                    .insert(root_node_entity);
            }
        }
    }

    /// Creates or updates a root node
    fn create_or_update_root_node_data(
        &mut self,
        root_node_entity: &Entity,
        camera_entity: &Entity,
    ) -> &mut RootNodeData {
        let user_root_node = *self.entity_to_taffy.get(root_node_entity).expect("create_or_update_root_node_data called before ui_root_node_entity was added to taffy tree or was previously removed");
        let ui_root_node_entity = *root_node_entity;
        let camera_entity = *camera_entity;

        let mut added = false;

        // creates mutable borrow on self that lives as long as the result
        let _ = self
            .root_node_data
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

                RootNodeData {
                    camera_entity: Some(camera_entity),
                    implicit_viewport_node,
                }
            });

        if !added {
            self.replace_camera_association(ui_root_node_entity, Some(camera_entity));
        }

        self.root_node_data
            .get_mut(root_node_entity)
            .unwrap_or_else(|| unreachable!("impossible"))
    }

    /// Set the ui node entities without a [`Parent`] as children to the root node in the taffy layout.
    pub fn set_camera_children(
        &mut self,
        camera_entity: Entity,
        children: impl Iterator<Item = Entity>,
    ) {
        let removed_children = self.camera_root_nodes.entry(camera_entity).or_default();
        let mut removed_children = removed_children.clone();

        for ui_entity in children {
            // creates mutable borrow on self that lives as long as the result
            let _ = self.create_or_update_root_node_data(&ui_entity, &camera_entity);

            // drop the mutable borrow on self by re-fetching
            let root_node_data = self
                .root_node_data
                .get(&ui_entity)
                .unwrap_or_else(|| unreachable!("impossible"));

            // fix taffy relationships
            {
                let taffy_node = *self.entity_to_taffy.get(&ui_entity).unwrap();
                if let Some(parent) = self.taffy.parent(taffy_node) {
                    self.taffy.remove_child(parent, taffy_node).unwrap();
                }

                self.taffy
                    .add_child(root_node_data.implicit_viewport_node, taffy_node)
                    .unwrap();
            }

            removed_children.remove(&ui_entity);
        }

        for orphan in removed_children.iter() {
            self.mark_root_node_as_orphaned(orphan);
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

            let Some(root_node_data) = self.root_node_data.get(&root_node_entity) else {
                continue;
            };
            if root_node_data.camera_entity.is_none() {
                panic!("internal map out of sync");
            }

            self.taffy
                .compute_layout(root_node_data.implicit_viewport_node, available_space)
                .unwrap();
        }
    }

    /// Removes specified camera entities by disassociating them from their associated `implicit_viewport_node`
    /// in the internal map, and subsequently removes the `implicit_viewport_node`
    /// from the `taffy` layout engine for each.
    pub fn remove_camera_entities(&mut self, entities: impl IntoIterator<Item = Entity>) {
        for entity in entities {
            self.remove_camera(&entity);
        }
    }

    /// Removes the specified entities from the internal map while
    /// removing their `implicit_viewport_node` from taffy,
    /// and then subsequently removes their entry from `entity_to_taffy` and associated node from taffy
    pub fn remove_entities(&mut self, entities: impl IntoIterator<Item = Entity>) {
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

    #[cfg(test)]
    /// Tries to get the associated camera entity from root node data
    fn get_associated_camera_entity(&self, root_node_entity: Entity) -> Option<Entity> {
        self.get_root_node_data(root_node_entity)?.camera_entity
    }

    #[cfg(test)]
    /// Tries to get the root node data for a given root node entity
    fn get_root_node_data(&self, root_node_entity: Entity) -> Option<&RootNodeData> {
        self.root_node_data.get(&root_node_entity)
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
        assert!(ui_surface.entity_to_taffy.is_empty());
        assert!(ui_surface.root_node_data.is_empty());
        assert!(ui_surface.camera_root_nodes.is_empty());
        assert_eq!(ui_surface.taffy.total_node_count(), 0);
    }

    const TEST_LAYOUT_CONTEXT: LayoutContext = LayoutContext {
        scale_factor: 1.0,
        physical_size: Vec2::ONE,
        min_size: 0.0,
        max_size: 1.0,
    };

    trait HasValidRootNodeData {
        fn has_valid_root_node_data(&self, root_node_entity: &Entity) -> bool;
    }

    impl HasValidRootNodeData for UiSurface {
        fn has_valid_root_node_data(&self, root_node_entity: &Entity) -> bool {
            let Some(&taffy_node) = self.entity_to_taffy.get(root_node_entity) else {
                return false;
            };
            let Some(root_node_data) = self.root_node_data.get(root_node_entity) else {
                return false;
            };
            self.taffy.parent(taffy_node) == Some(root_node_data.implicit_viewport_node)
        }
    }

    #[test]
    fn test_upsert() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        // standard upsert
        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);

        // should be inserted into taffy
        assert_eq!(ui_surface.taffy.total_node_count(), 1);
        assert!(ui_surface.entity_to_taffy.contains_key(&root_node_entity));

        // test duplicate insert 1
        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);

        // node count should not have increased
        assert_eq!(ui_surface.taffy.total_node_count(), 1);

        // assign root node to camera
        ui_surface.set_camera_children(camera_entity, vec![root_node_entity].into_iter());

        // each root node will create 2 taffy nodes
        assert_eq!(ui_surface.taffy.total_node_count(), 2);

        // root node data should now exist
        assert!(ui_surface.has_valid_root_node_data(&root_node_entity));

        // test duplicate insert 2
        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);

        // node count should not have increased
        assert_eq!(ui_surface.taffy.total_node_count(), 2);

        // root node data should be unaffected
        assert!(ui_surface.has_valid_root_node_data(&root_node_entity));
    }

    #[test]
    fn test_remove_camera_entities() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);

        // assign root node to camera
        ui_surface.set_camera_children(camera_entity, [root_node_entity].into_iter());

        assert!(ui_surface.camera_root_nodes.contains_key(&camera_entity));
        assert!(ui_surface.root_node_data.contains_key(&root_node_entity));
        let _root_node_data = ui_surface
            .get_root_node_data(root_node_entity)
            .expect("expected root node data");
        assert!(ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&root_node_entity));

        ui_surface.remove_camera_entities([camera_entity]);

        // should not affect `entity_to_taffy`
        assert!(ui_surface.entity_to_taffy.contains_key(&root_node_entity));

        // `camera_roots` and `camera_entity_to_taffy` should no longer contain entries for `camera_entity`
        assert!(!ui_surface.camera_root_nodes.contains_key(&camera_entity));

        assert!(!ui_surface.camera_root_nodes.contains_key(&camera_entity));

        // root node data should be removed
        let root_node_data = ui_surface.get_root_node_data(root_node_entity);
        assert_eq!(root_node_data, None);
    }

    #[test]
    fn test_remove_entities() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);

        ui_surface.set_camera_children(camera_entity, [root_node_entity].into_iter());

        assert!(ui_surface.entity_to_taffy.contains_key(&root_node_entity));
        assert!(ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&root_node_entity));
        let _root_node_data = ui_surface.get_root_node_data(root_node_entity).unwrap();

        ui_surface.remove_entities([root_node_entity]);
        assert!(!ui_surface.entity_to_taffy.contains_key(&root_node_entity));

        assert!(!ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .contains(&root_node_entity));
        assert!(ui_surface
            .camera_root_nodes
            .get(&camera_entity)
            .unwrap()
            .is_empty());
    }

    #[test]
    fn test_try_update_measure() {
        let mut ui_surface = UiSurface::default();
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);
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
        let root_node_entity = Entity::from_raw(1);
        let child_entity = Entity::from_raw(2);
        let style = Style::default();

        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);
        ui_surface.upsert_node(child_entity, &style, &TEST_LAYOUT_CONTEXT);

        ui_surface.update_children(root_node_entity, &[child_entity]);

        let parent_node = *ui_surface.entity_to_taffy.get(&root_node_entity).unwrap();
        let child_node = *ui_surface.entity_to_taffy.get(&child_entity).unwrap();
        assert_eq!(ui_surface.taffy.parent(child_node), Some(parent_node));
    }

    #[test]
    fn test_set_camera_children() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let child_entity = Entity::from_raw(2);
        let style = Style::default();

        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);
        ui_surface.upsert_node(child_entity, &style, &TEST_LAYOUT_CONTEXT);

        let root_taffy_node = *ui_surface.entity_to_taffy.get(&root_node_entity).unwrap();
        let child_taffy = *ui_surface.entity_to_taffy.get(&child_entity).unwrap();

        // set up the relationship manually
        ui_surface
            .taffy
            .add_child(root_taffy_node, child_taffy)
            .unwrap();

        ui_surface.set_camera_children(camera_entity, [root_node_entity].into_iter());

        assert!(
            ui_surface
                .camera_root_nodes
                .get(&camera_entity)
                .unwrap()
                .contains(&root_node_entity),
            "root node not associated with camera"
        );
        assert!(
            !ui_surface
                .camera_root_nodes
                .get(&camera_entity)
                .unwrap()
                .contains(&child_entity),
            "child of root node should not be associated with camera"
        );

        let _root_node_data = ui_surface
            .get_root_node_data(root_node_entity)
            .expect("expected root node data");

        assert_eq!(ui_surface.taffy.parent(child_taffy), Some(root_taffy_node));
        let root_taffy_children = ui_surface.taffy.children(root_taffy_node).unwrap();
        assert!(
            root_taffy_children.contains(&child_taffy),
            "root node is not a parent of child node"
        );
        assert_eq!(
            ui_surface.taffy.child_count(root_taffy_node).unwrap(),
            1,
            "expected root node child count to be 1"
        );

        // clear camera's root nodes
        ui_surface.set_camera_children(camera_entity, Vec::<Entity>::new().into_iter());

        assert!(
            !ui_surface
                .camera_root_nodes
                .get(&camera_entity)
                .unwrap()
                .contains(&root_node_entity),
            "root node should have been unassociated with camera"
        );
        assert!(
            !ui_surface
                .camera_root_nodes
                .get(&camera_entity)
                .unwrap()
                .contains(&child_entity),
            "child of root node should not be associated with camera"
        );

        let root_taffy_children = ui_surface.taffy.children(root_taffy_node).unwrap();
        assert!(
            root_taffy_children.contains(&child_taffy),
            "root node is not a parent of child node"
        );
        assert_eq!(
            ui_surface.taffy.child_count(root_taffy_node).unwrap(),
            1,
            "expected root node child count to be 1"
        );

        // re-associate root node with camera
        ui_surface.set_camera_children(camera_entity, vec![root_node_entity].into_iter());

        assert!(
            ui_surface
                .camera_root_nodes
                .get(&camera_entity)
                .unwrap()
                .contains(&root_node_entity),
            "root node should have been re-associated with camera"
        );
        assert!(
            !ui_surface
                .camera_root_nodes
                .get(&camera_entity)
                .unwrap()
                .contains(&child_entity),
            "child of root node should not be associated with camera"
        );

        let child_taffy = ui_surface.entity_to_taffy.get(&child_entity).unwrap();
        let root_taffy_children = ui_surface.taffy.children(root_taffy_node).unwrap();
        assert!(
            root_taffy_children.contains(child_taffy),
            "root node is not a parent of child node"
        );
        assert_eq!(
            ui_surface.taffy.child_count(root_taffy_node).unwrap(),
            1,
            "expected root node child count to be 1"
        );
    }

    #[test]
    fn test_compute_camera_layout() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let style = Style::default();

        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);

        ui_surface.compute_camera_layout(&camera_entity, UVec2::new(800, 600));

        let taffy_node = ui_surface.entity_to_taffy.get(&root_node_entity).unwrap();
        assert!(ui_surface.taffy.layout(*taffy_node).is_ok());
    }

    #[test]
    fn test_promotion() {
        let mut ui_surface = UiSurface::default();
        let camera_entity = Entity::from_raw(0);
        let root_node_entity = Entity::from_raw(1);
        let child_entity = Entity::from_raw(2);
        let style = Style::default();

        ui_surface.upsert_node(root_node_entity, &style, &TEST_LAYOUT_CONTEXT);
        ui_surface.upsert_node(child_entity, &style, &TEST_LAYOUT_CONTEXT);
        ui_surface.update_children(root_node_entity, &[child_entity]);
        assert_eq!(ui_surface.taffy.total_node_count(), 2);

        ui_surface.set_camera_children(camera_entity, [root_node_entity].into_iter());
        assert_eq!(ui_surface.taffy.total_node_count(), 3);


        ui_surface.promote_ui_node(&child_entity, &camera_entity);
        assert_eq!(ui_surface.taffy.total_node_count(), 4);
        assert_eq!(ui_surface.get_associated_camera_entity(child_entity), Some(camera_entity));

        let root_node_entity_taffy = ui_surface.entity_to_taffy.get(&root_node_entity).unwrap();
        let child_entity_taffy = ui_surface.entity_to_taffy.get(&child_entity).unwrap();
        assert!(!ui_surface.taffy.children(*root_node_entity_taffy).unwrap().contains(child_entity_taffy));
    }
}
