#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    t.pass("tests/derive/happy_path.rs");
    t.compile_fail("tests/derive/struct_derive.rs");
    t.compile_fail("tests/derive/union_derive.rs");
    t.compile_fail("tests/derive/too_many_layers.rs");
    t.compile_fail("tests/derive/multiple_defaults.rs");
    t.compile_fail("tests/derive/no_default.rs");
    t.compile_fail("tests/derive/variant_with_fields.rs");
}
