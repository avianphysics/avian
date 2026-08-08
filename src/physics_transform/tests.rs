#[cfg(feature = "3d")]
use crate::math::QuatExt;
#[cfg(feature = "2d")]
use crate::math::Rot2Ext;
use crate::{physics_transform::PhysicsTransformConfig, prelude::*};
#[cfg(feature = "3d")]
use approx::assert_relative_eq;
use bevy::prelude::*;

#[test]
fn test_init_transforms_basics() {
    let mut app = App::new();

    // Automatically add `Transform` for every rigid body for this test.
    app.register_required_components::<RigidBody, Transform>();

    // Test all possible config permutations
    for (position_to_transform, transform_to_position) in
        [(true, true), (true, false), (false, true), (false, false)]
    {
        let config = PhysicsTransformConfig {
            position_to_transform,
            transform_to_position,
            ..default()
        };
        app.insert_resource(dbg!(config.clone()));

        // Spawn entities with a full `PhysicsTransform`
        let transform_0 = {
            #[cfg(feature = "2d")]
            {
                PhysicsTransform::new(RVector::new(1., 2.), Rot2::radians(0.5))
            }
            #[cfg(feature = "3d")]
            {
                PhysicsTransform::new(
                    RVector::new(1., 2., 3.),
                    Quat::from_axis_angle(Vec3::Y, 0.5),
                )
            }
        };
        let e_0_with_pos_and_rot = app
            .world_mut()
            .spawn((RigidBody::Dynamic, transform_0))
            .id();

        let transform_1 = {
            #[cfg(feature = "2d")]
            {
                PhysicsTransform::new(RVector::new(-1., 3.), Rot2::radians(0.1))
            }
            #[cfg(feature = "3d")]
            {
                PhysicsTransform::new(
                    RVector::new(-1., 3., -3.),
                    Quat::from_axis_angle(Vec3::X, 0.1),
                )
            }
        };
        let e_1_with_pos_and_rot = app
            .world_mut()
            .spawn((RigidBody::Dynamic, transform_1))
            .id();

        // Spawn an entity with only the translation set.
        let pos_2 = {
            #[cfg(feature = "2d")]
            {
                RVector::new(10., 1.)
            }
            #[cfg(feature = "3d")]
            {
                RVector::new(10., 1., 5.)
            }
        };
        let transform_2 = PhysicsTransform {
            translation: pos_2,
            rotation: PhysicsTransform::PLACEHOLDER.rotation,
        };
        let e_2_with_pos = app
            .world_mut()
            .spawn((RigidBody::Dynamic, transform_2))
            .id();

        // Spawn an entity with only the rotation set.
        let rot_3 = {
            #[cfg(feature = "2d")]
            {
                Rot2::radians(0.4)
            }
            #[cfg(feature = "3d")]
            {
                Quat::from_axis_angle(Vec3::Z, 0.4)
            }
        };
        let transform_3 = PhysicsTransform {
            translation: PhysicsTransform::PLACEHOLDER.translation,
            rotation: rot_3,
        };
        let e_3_with_rot = app
            .world_mut()
            .spawn((RigidBody::Dynamic, transform_3))
            .id();

        // Spawn entities with `Transform`
        let trans_4 = {
            Transform {
                translation: Vec3::new(-1.1, 6., -7.),
                rotation: Quat::from_axis_angle(Vec3::Y, 0.1),
                scale: Vec3::ONE,
            }
        };
        let e_4_with_trans = app.world_mut().spawn((RigidBody::Dynamic, trans_4)).id();

        let trans_5 = {
            Transform {
                translation: Vec3::new(8., -1., 0.),
                rotation: Quat::from_axis_angle(Vec3::Y, -0.1),
                scale: Vec3::ONE,
            }
        };
        let e_5_with_trans = app.world_mut().spawn((RigidBody::Dynamic, trans_5)).id();

        // Spawn entity without any transforms
        let e_6_without_trans = app.world_mut().spawn(RigidBody::Dynamic).id();

        // Spawn entity without a ridid body
        let e_7_without_rb = app.world_mut().spawn(()).id();

        // Run the system
        app.update();

        // Check the results are as expected
        if config.position_to_transform {
            assert!(app.world().get::<Transform>(e_0_with_pos_and_rot).is_some());
            let transform = app.world().get::<Transform>(e_0_with_pos_and_rot).unwrap();
            let expected: Vec3 = {
                #[cfg(feature = "2d")]
                {
                    transform_0.translation.f32().extend(0.)
                }
                #[cfg(feature = "3d")]
                {
                    transform_0.translation.f32()
                }
            };
            assert_eq!(transform.translation, expected);
            let expected = transform_0.rotation.to_quat();
            assert_eq!(transform.rotation, expected);

            assert!(app.world().get::<Transform>(e_1_with_pos_and_rot).is_some());
            let transform = app.world().get::<Transform>(e_1_with_pos_and_rot).unwrap();
            let expected: Vec3 = {
                #[cfg(feature = "2d")]
                {
                    transform_1.translation.f32().extend(0.)
                }
                #[cfg(feature = "3d")]
                {
                    transform_1.translation.f32()
                }
            };
            assert_eq!(transform.translation, expected);
            let expected = transform_1.rotation.to_quat();
            assert_eq!(transform.rotation, expected);

            assert!(app.world().get::<Transform>(e_2_with_pos).is_some());
            let transform = app.world().get::<Transform>(e_2_with_pos).unwrap();
            let expected: Vec3 = {
                #[cfg(feature = "2d")]
                {
                    pos_2.f32().extend(0.)
                }
                #[cfg(feature = "3d")]
                {
                    pos_2.f32()
                }
            };
            assert_eq!(transform.translation, expected);
            let expected = Quat::default();
            assert_eq!(transform.rotation, expected);

            assert!(app.world().get::<Transform>(e_3_with_rot).is_some());
            let transform = app.world().get::<Transform>(e_3_with_rot).unwrap();
            let expected: Vec3 = Vec3::default();
            assert_eq!(transform.translation, expected);
            let expected = rot_3.to_quat();
            assert_eq!(transform.rotation, expected);

            assert!(app.world().get::<Transform>(e_4_with_trans).is_some());
            let transform = app.world().get::<Transform>(e_4_with_trans).unwrap();
            assert_eq!(transform, &trans_4);

            assert!(app.world().get::<Transform>(e_5_with_trans).is_some());
            let transform = app.world().get::<Transform>(e_5_with_trans).unwrap();
            assert_eq!(transform, &trans_5);

            assert!(app.world().get::<Transform>(e_6_without_trans).is_some());
            let transform = app.world().get::<Transform>(e_6_without_trans).unwrap();
            assert_eq!(transform, &Transform::default());

            assert!(app.world().get::<Transform>(e_7_without_rb).is_none());
        }

        if config.transform_to_position {
            let transform = app
                .world()
                .get::<PhysicsTransform>(e_0_with_pos_and_rot)
                .unwrap();
            assert_eq!(transform, &transform_0);

            let transform = app
                .world()
                .get::<PhysicsTransform>(e_1_with_pos_and_rot)
                .unwrap();
            assert_eq!(transform, &transform_1);

            let transform = app.world().get::<PhysicsTransform>(e_2_with_pos).unwrap();
            assert_eq!(transform.translation, pos_2);
            assert_eq!(transform.rotation, Rot::IDENTITY);

            let transform = app.world().get::<PhysicsTransform>(e_3_with_rot).unwrap();
            assert_eq!(transform.translation, RVector::ZERO);
            assert_eq!(transform.rotation, rot_3);

            let transform = app.world().get::<PhysicsTransform>(e_4_with_trans).unwrap();
            let expected = {
                #[cfg(feature = "2d")]
                {
                    trans_4.translation.truncate().real()
                }
                #[cfg(feature = "3d")]
                {
                    trans_4.translation.real()
                }
            };
            assert_eq!(transform.translation, expected);
            #[cfg(feature = "2d")]
            assert_eq!(transform.rotation, Rot2::from_quat(trans_4.rotation));
            #[cfg(feature = "3d")]
            assert_relative_eq!(transform.rotation, trans_4.rotation);

            let transform = app.world().get::<PhysicsTransform>(e_5_with_trans).unwrap();
            let expected = {
                #[cfg(feature = "2d")]
                {
                    trans_5.translation.truncate().real()
                }
                #[cfg(feature = "3d")]
                {
                    trans_5.translation.real()
                }
            };
            assert_eq!(transform.translation, expected);
            #[cfg(feature = "2d")]
            assert_eq!(transform.rotation, Rot2::from_quat(trans_5.rotation));
            #[cfg(feature = "3d")]
            assert_relative_eq!(transform.rotation, trans_5.rotation);

            let transform = app
                .world()
                .get::<PhysicsTransform>(e_6_without_trans)
                .unwrap();
            assert_eq!(transform, &PhysicsTransform::IDENTITY);

            assert!(
                app.world()
                    .get::<PhysicsTransform>(e_7_without_rb)
                    .is_none()
            );
        }
    }
}
