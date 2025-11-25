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

        // Spawn entities with `Position` and `Rotation`
        let (pos_0, rot_0) = {
            #[cfg(feature = "2d")]
            {
                (Position::from_xy(1., 2.), Rotation::radians(0.5))
            }
            #[cfg(feature = "3d")]
            {
                (
                    Position::from_xyz(1., 2., 3.),
                    Rotation(Quaternion::from_axis_angle(Vector::Y, 0.5)),
                )
            }
        };
        let e_0_with_pos_and_rot = app
            .world_mut()
            .spawn((RigidBody::Dynamic, pos_0, rot_0))
            .id();

        let (pos_1, rot_1) = {
            #[cfg(feature = "2d")]
            {
                (Position::from_xy(-1., 3.), Rotation::radians(0.1))
            }
            #[cfg(feature = "3d")]
            {
                (
                    Position::from_xyz(-1., 3., -3.),
                    Rotation(Quaternion::from_axis_angle(Vector::X, 0.1)),
                )
            }
        };
        let e_1_with_pos_and_rot = app
            .world_mut()
            .spawn((RigidBody::Dynamic, pos_1, rot_1))
            .id();

        // Spawn an entity with only `Position`
        let pos_2 = {
            #[cfg(feature = "2d")]
            {
                Position::from_xy(10., 1.)
            }
            #[cfg(feature = "3d")]
            {
                Position::from_xyz(10., 1., 5.)
            }
        };
        let e_2_with_pos = app.world_mut().spawn((RigidBody::Dynamic, pos_2)).id();

        // Spawn an entity with only `Rotation`
        let rot_3 = {
            #[cfg(feature = "2d")]
            {
                Rotation::radians(0.4)
            }
            #[cfg(feature = "3d")]
            {
                Rotation(Quaternion::from_axis_angle(Vector::Z, 0.4))
            }
        };
        let e_3_with_rot = app.world_mut().spawn((RigidBody::Dynamic, rot_3)).id();

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
                    pos_0.f32().extend(0.)
                }
                #[cfg(feature = "3d")]
                {
                    pos_0.f32()
                }
            };
            assert_eq!(transform.translation, expected);
            let expected = Quaternion::from(rot_0).f32();
            assert_eq!(transform.rotation, expected);

            assert!(app.world().get::<Transform>(e_1_with_pos_and_rot).is_some());
            let transform = app.world().get::<Transform>(e_1_with_pos_and_rot).unwrap();
            let expected: Vec3 = {
                #[cfg(feature = "2d")]
                {
                    pos_1.f32().extend(0.)
                }
                #[cfg(feature = "3d")]
                {
                    pos_1.f32()
                }
            };
            assert_eq!(transform.translation, expected);
            let expected = Quaternion::from(rot_1).f32();
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
            let expected = Quaternion::from(rot_3).f32();
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
            assert!(app.world().get::<Position>(e_0_with_pos_and_rot).is_some());
            let pos = app.world().get::<Position>(e_0_with_pos_and_rot).unwrap();
            assert_eq!(pos, &pos_0);
            assert!(app.world().get::<Rotation>(e_0_with_pos_and_rot).is_some());
            let rot = app.world().get::<Rotation>(e_0_with_pos_and_rot).unwrap();
            assert_eq!(rot, &rot_0);

            assert!(app.world().get::<Position>(e_1_with_pos_and_rot).is_some());
            let pos = app.world().get::<Position>(e_1_with_pos_and_rot).unwrap();
            assert_eq!(pos, &pos_1);
            assert!(app.world().get::<Rotation>(e_1_with_pos_and_rot).is_some());
            let rot = app.world().get::<Rotation>(e_1_with_pos_and_rot).unwrap();
            assert_eq!(rot, &rot_1);

            assert!(app.world().get::<Position>(e_2_with_pos).is_some());
            let pos = app.world().get::<Position>(e_2_with_pos).unwrap();
            assert_eq!(pos, &pos_2);
            assert!(app.world().get::<Rotation>(e_2_with_pos).is_some());
            let rot = app.world().get::<Rotation>(e_2_with_pos).unwrap();
            assert_eq!(rot, &Rotation::default());

            assert!(app.world().get::<Position>(e_3_with_rot).is_some());
            let pos = app.world().get::<Position>(e_3_with_rot).unwrap();
            assert_eq!(pos, &Position::default());
            assert!(app.world().get::<Rotation>(e_3_with_rot).is_some());
            let rot = app.world().get::<Rotation>(e_3_with_rot).unwrap();
            assert_eq!(rot, &rot_3);

            assert!(app.world().get::<Position>(e_4_with_trans).is_some());
            let pos = app.world().get::<Position>(e_4_with_trans).unwrap();
            let expected: Position = Position::new({
                #[cfg(feature = "2d")]
                {
                    trans_4.translation.truncate().adjust_precision()
                }
                #[cfg(feature = "3d")]
                {
                    trans_4.translation.adjust_precision()
                }
            });
            assert_eq!(pos, &expected);
            assert!(app.world().get::<Rotation>(e_4_with_trans).is_some());
            let rot = app.world().get::<Rotation>(e_4_with_trans).unwrap();
            #[cfg(feature = "2d")]
            assert_eq!(*rot, Rotation::from(trans_4.rotation));
            #[cfg(feature = "3d")]
            assert_relative_eq!(rot.f32(), trans_4.rotation);

            assert!(app.world().get::<Position>(e_5_with_trans).is_some());
            let pos = app.world().get::<Position>(e_5_with_trans).unwrap();
            let expected: Position = Position::new({
                #[cfg(feature = "2d")]
                {
                    trans_5.translation.truncate().adjust_precision()
                }
                #[cfg(feature = "3d")]
                {
                    trans_5.translation.adjust_precision()
                }
            });
            assert_eq!(pos, &expected);
            assert!(app.world().get::<Rotation>(e_5_with_trans).is_some());
            let rot = app.world().get::<Rotation>(e_5_with_trans).unwrap();
            #[cfg(feature = "2d")]
            assert_eq!(rot, &Rotation::from(trans_5.rotation));
            #[cfg(feature = "3d")]
            assert_relative_eq!(rot.f32(), trans_5.rotation);

            assert!(app.world().get::<Position>(e_6_without_trans).is_some());
            let pos = app.world().get::<Position>(e_6_without_trans).unwrap();
            assert_eq!(pos, &Position::default());
            assert!(app.world().get::<Rotation>(e_6_without_trans).is_some());
            let rot = app.world().get::<Rotation>(e_6_without_trans).unwrap();
            assert_eq!(rot, &Rotation::default());

            assert!(app.world().get::<Position>(e_7_without_rb).is_none());
            assert!(app.world().get::<Rotation>(e_7_without_rb).is_none());
        }
    }
}
