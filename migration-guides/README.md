# Avian's Migration Guide Process

*Adapted from [Bevy's migration guide process](https://github.com/bevyengine/bevy/blob/c392d28d30f34b7d2b69b806136b9643379f9284/release-content/migration_guides.md)*

Hi! Did someone add `M-Migration-Guide` to your PR? If so, you're in the right place.
Let's talk about how this process works.

When we make breaking changes to Avian, we need to communicate them to users so that their libraries and applications can be moved to the new Avian version.
To do this, we write and ship a migration guide for every major Avian version. In order to avoid crunch at the end of the cycle as we write all of these,
Avian asks authors (and reviewers) to write a draft migration guide as part of the pull requests that make breaking changes.

## Where to put your migration guides

All breaking changes on the `main` branch are collected into `migration-guides/x.y-to-main.md`,
where `x` and `y` denote the latest version number (e.g. 0.4). This guide will change and evolve
as the cycle progresses, and when the release is made, it is cleaned up and renamed according to
the new version number (e.g. 0.5).

## What to put in your draft migration guide

Migration guides are intended to briefly communicate:

- what has been changed since the last release?
- why did we make this breaking change?
- how can users migrate their existing code?

Draft migration guides *do not need to be polished*: it's okay if you're not a native English speaker or aren't a wordsmith.
We mainly want a brief summary that answers the questions above, written by the person who introduced the change.
There will be a final editing pass at the end of the cycle where everything is cleaned up.

When writing migration guides, prefer terse, technical language, and be sure to include terms that users might search for.
Migration guides are not read end-to-end: instead, they are navigated via Ctrl+F as the reader follows the compiler errors and bugs.

## Grouping changes into migration guides

Migration guides should reflect the complete experience of migrating from the last major Avian version to the next one.
If there are *multiple* breaking changes layered on top of each other, you should edit the existing migration guide, rather than write a new one.

While some brave users live on Avian's `main` branch, we can trust them to use the draft migration guides and read the PRs in question if needed.
As a result, each draft migration should be given a clear name. These titles should reflect the name of the old feature that was broken or changed.

Right below the title, you should also include the PR number(s) associated with the change.
An example might look like this:

```md
## Sleeping and Simulation Islands

PRs: [#809](https://github.com/avianphysics/avian/pull/809)

Content goes here
```

## Note on the `#[deprecated]` attribute

Rust provides a very helpful [`#[deprecated]` attribute](https://doc.rust-lang.org/reference/attributes/diagnostics.html#the-deprecated-attribute),
which is a compiler-aware way to mark a piece of Rust code as obsolete and slated for removal. This can be a nice a tool
to ease migrations, because it downgrades errors to warnings, and makes the migration information available right in the user's IDE.

However, it is not always possible to use this attribute, and Avian does not consider it to be a substitute to a migration guide entry.

```rust
#[deprecated(since = "0.4.0", note = "This message will appear in the deprecation warning.")]
struct MyStruct;
```

## Style Guide

Keep it short and sweet:

- What, then why, then how to migrate.
- Some helpful standardized phrases:
  - `OldType` is now `NewType`. Replace all references and imports.
  - The `Struct::method` method now requires an additional `magnitude: f32` argument.
  - `Enum` has a new variant, `Enum::NewVariant`, which must be handled during `match` statements.
  - The `Type::method` method has been removed. Use `Type::other_method` instead.
  - The `crate::old_module` module is now `crate::new_module`. Update your imports.
  - `function` now returns `Option<String>` instead of `String`.
- Make sure it's searchable by directly naming the types and methods involved.
- Use backticks for types, methods, and modules (e.g. `Vec<T>` or `core::mem::swap`).
- Use bullet points when listing affected types / functions of a breaking change, or when listing several complex steps for migrating. Avoid bullets for simple migrations, however.
- It's often useful to give a code example explaining what a migration may look like.

  ```rust
  // 0.3
  fn print_started_collisions(mut collision_reader: EventReader<CollisionStarted>) {
      for CollisionStarted(collider1, collider2) in collision_reader.read() {
          println!("{collider1} and {collider2} started colliding");
      }
  }

  // 0.4
  fn print_started_collisions(mut collision_reader: MessageReader<CollisionStart>) {
      // Note: The event now also stores `body1` and `body2`
      for event in collision_reader.read() {
          println!("{} and {} started colliding", event.collider1, event.collider2);
      }
  }
  ```

  Often you will want to give two examples of the same piece of code, one for the old version and one for the new. You can designate which is which using comments, such as `// 0.3` and `// 0.4`. Avoid code diffs if possible, as they do not syntax highlight Rust code.

- Make sure to reference the currently published version of a crate when writing a migration guide.
  See [docs.rs](https://docs.rs/avian3d) for a quick reference to the existing public API.
