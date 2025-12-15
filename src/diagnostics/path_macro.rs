/// A procedural macro to generate diagnostic paths.
///
/// # Example
///
/// The macro can be used like this:
///
/// ```ignore
/// impl_diagnostic_paths! {
///     pub struct SolverDiagnostics {
///         PREPARE_CONSTRAINTS: "avian/solver/prepare_constraints",
///         INTEGRATE_VELOCITIES: "avian/solver/integrate_velocities",
///         // ...
///     }
/// }
/// ```
///
/// It expands to:
///
/// ```
/// pub struct SolverDiagnostics;
///
/// impl SolverDiagnostics {
///     pub const PREPARE_CONSTRAINTS: &'static bevy::diagnostic::DiagnosticPath = &bevy::diagnostic::DiagnosticPath::const_new("avian/solver/prepare_constraints");
///     pub const INTEGRATE_VELOCITIES: &'static bevy::diagnostic::DiagnosticPath = &bevy::diagnostic::DiagnosticPath::const_new("avian/solver/integrate_velocities");
///     // ...
/// }
/// ```
macro_rules! impl_diagnostic_paths {
    ($(#[$meta:meta])* impl $name:ident { $($path:ident: $path_str:expr,)* }) => {
        #[allow(missing_docs)]
        impl $name {
            $(
                pub const $path: &'static bevy::diagnostic::DiagnosticPath = &bevy::diagnostic::DiagnosticPath::const_new($path_str);
            )*
        }
    };
}

pub(crate) use impl_diagnostic_paths;
