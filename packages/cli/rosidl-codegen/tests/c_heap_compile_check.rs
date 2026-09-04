//! Manual syntax check for `mode = "heap"` generated C (RFC-0033 / Phase 229.5,
//! C path). Generates a heap C message and runs `gcc -fsyntax-only` against
//! minimal stub `nros/{types,cdr,platform}.h` headers (the real headers need
//! per-build config + opaque-size probes, orthogonal to this check). Ignored by
//! default; run with:
//!   cargo test -p rosidl-codegen --test c_heap_compile_check -- --ignored

use rosidl_codegen::{CapacityResolver, generate_c_message_package, generate_c_service_package};
use rosidl_parser::{parse_message, parse_service};
use std::{fs, process::Command};

const TYPES_H: &str = r#"
#ifndef NROS_TYPES_STUB_H
#define NROS_TYPES_STUB_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef struct { const char* type_name; const char* type_hash; size_t serialized_size_max; } nros_message_type_t;
// Issue 0345 — the service emitter defines a `struct nros_service_type_t`
// type-support object (mirrors nros_generated.h:2312).
typedef struct nros_service_type_t {
    const char* type_name;
    const char* type_hash;
} nros_service_type_t;
typedef int32_t nros_ret_t;
struct nros_publisher_t;
nros_ret_t nros_publish_raw(struct nros_publisher_t* p, const uint8_t* buf, size_t n);

// phase-417 W5.e — the typed service/client glue the service pack now emits.
// The stub grows with the surface the templates NAME; a stub that stops short
// of it turns "the generated code is wrong" into "the stub is short", which is
// the failure mode this comment exists to prevent.
#define NROS_RET_OK 0
#define NROS_RET_ERROR -1
#define NROS_RET_INVALID_ARGUMENT -3
#define NROS_RET_TRY_AGAIN -14

#define NROS_SERVICE_TYPED_OK 0
#define NROS_SERVICE_TYPED_ERR_REQUEST_DESERIALIZE -1
#define NROS_SERVICE_TYPED_ERR_RESPONSE_SERIALIZE -2
#define NROS_SERVICE_TYPED_ERR_REQUEST_SERIALIZE -3
#define NROS_SERVICE_TYPED_ERR_RESPONSE_DESERIALIZE -4
#define NROS_SERVICE_TYPED_ERR_NO_CALLBACK -5

struct nros_node_t;
struct nros_service_t;
struct nros_client_t;

typedef bool (*nros_service_callback_t)(const uint8_t* request_data, size_t request_len,
                                        uint8_t* response_data, size_t response_capacity,
                                        size_t* response_len, void* context);
typedef void (*nros_response_callback_t)(const uint8_t* response, size_t response_len,
                                         void* context);

void nros_service_typed_report_error(int32_t error, const char* type_name);
nros_ret_t nros_service_init(struct nros_service_t* service, const struct nros_node_t* node,
                             const struct nros_service_type_t* type_info, const char* service_name,
                             nros_service_callback_t callback, void* context);
nros_ret_t nros_client_set_response_callback(struct nros_client_t* client,
                                             nros_response_callback_t callback, void* context);
nros_ret_t nros_client_send_request_async(struct nros_client_t* client,
                                          const uint8_t* request_data, size_t request_len);
nros_ret_t nros_client_take_response(struct nros_client_t* client, uint8_t* response_data,
                                     size_t response_capacity, size_t* response_len);
nros_ret_t nros_client_call(struct nros_client_t* client, const uint8_t* request_data,
                            size_t request_len, uint8_t* response_data, size_t response_capacity,
                            size_t* response_len);
#endif
"#;

const PLATFORM_H: &str = r#"
#ifndef NROS_PLATFORM_STUB_H
#define NROS_PLATFORM_STUB_H
#include <stddef.h>
void* nros_platform_malloc(size_t size);
void nros_platform_free(void* ptr);
#endif
"#;

const BORROWED_H: &str = r#"
#ifndef NROS_BORROWED_STUB_H
#define NROS_BORROWED_STUB_H
#include <stdint.h>
#include <stddef.h>
#include <nros/cdr.h>
typedef struct { const char* data; size_t size; } nros_view_str_t;
typedef struct { const uint8_t* data; size_t size; } nros_view_bytes_t;
static inline int32_t nros_cdr_borrow_string(const uint8_t** p, const uint8_t* e,
                                             const uint8_t* o, nros_view_str_t* out) {
    uint32_t n; if (nros_cdr_read_u32(p, e, o, &n) < 0) return -1;
    if ((size_t)(e - *p) < n) return -1;
    out->data = (const char*)*p; out->size = n > 0 ? (size_t)(n - 1) : 0; *p += n; return 0;
}
#endif
"#;

const CDR_H: &str = r#"
#ifndef NROS_CDR_STUB_H
#define NROS_CDR_STUB_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define W(name, ty) int nros_cdr_write_##name(uint8_t** p, const uint8_t* e, const uint8_t* o, ty v);
#define R(name, ty) int nros_cdr_read_##name(const uint8_t** p, const uint8_t* e, const uint8_t* o, ty* v);
W(u8, uint8_t) R(u8, uint8_t)
// Issue 0345: phase-303 W4 added the XCDR2 DHEADER/encapsulation seam to every
// generated TU. This stub predated it, so `generated_heap_c_message_compiles`
// had been failing since — invisibly, because it is `#[ignore]`d and no lane
// runs `--ignored` (issue 0328).
int32_t nros_cdr_write_encaps_header(uint8_t** p, const uint8_t* e);
int64_t nros_cdr_begin_dheader(uint8_t** p, const uint8_t* e, const uint8_t* o);
int nros_cdr_end_dheader(int64_t mark, uint8_t** p, const uint8_t* e, const uint8_t* o);
int64_t nros_cdr_begin_dheader_read(const uint8_t** p, const uint8_t* e, const uint8_t* o);
int nros_cdr_end_dheader_read(int64_t scope, const uint8_t** p, const uint8_t* e, const uint8_t* o);
W(i8, int8_t) R(i8, int8_t)
W(u16, uint16_t) R(u16, uint16_t)
W(i16, int16_t) R(i16, int16_t)
W(u32, uint32_t) R(u32, uint32_t)
W(i32, int32_t) R(i32, int32_t)
W(u64, uint64_t) R(u64, uint64_t)
W(i64, int64_t) R(i64, int64_t)
W(f32, float) R(f32, float)
W(f64, double) R(f64, double)
W(bool, bool) R(bool, bool)   // cdr.h:227 uses bool*, not uint8_t*
int nros_cdr_write_string(uint8_t** p, const uint8_t* e, const uint8_t* o, const char* s);
int nros_cdr_read_string(const uint8_t** p, const uint8_t* e, const uint8_t* o, char* d, size_t n);
#undef W
#undef R
#endif
"#;

#[test]
#[ignore = "spawns gcc -fsyntax-only"]
fn generated_heap_c_message_compiles() {
    let resolver = CapacityResolver::from_toml_str(
        r#"
        [fields]
        "my_msgs/Blob.data" = { cap = 0, mode = "heap" }
        "my_msgs/Blob.vals" = { cap = 0, mode = "heap" }
        "my_msgs/Blob.label" = { cap = 0, mode = "heap" }
        "my_msgs/Blob.tags" = { cap = 0, mode = "heap" }
        "#,
    )
    .unwrap();
    // Heap primitive seqs + heap string + heap string[] + scalar + owned bounded seq.
    let msg = parse_message(
        "uint8[] data\nfloat32[] vals\nstring label\nstring[] tags\nint32 seq\nint8[<=4] small\n",
    )
    .unwrap();
    let pkg = generate_c_message_package("my_msgs", "Blob", &msg, "h", &resolver).unwrap();

    let tmp = tempfile::tempdir().unwrap();
    let nros = tmp.path().join("nros");
    fs::create_dir_all(&nros).unwrap();
    fs::write(nros.join("types.h"), TYPES_H).unwrap();
    fs::write(nros.join("cdr.h"), CDR_H).unwrap();
    fs::write(nros.join("platform.h"), PLATFORM_H).unwrap();
    fs::write(tmp.path().join("my_msgs_msg_blob.h"), &pkg.header).unwrap();
    let c_path = tmp.path().join("my_msgs_msg_blob.c");
    fs::write(&c_path, &pkg.source).unwrap();

    let out = Command::new("gcc")
        .args(["-fsyntax-only", "-std=c11", "-Wall", "-Wextra", "-Werror"])
        .arg("-I")
        .arg(tmp.path())
        .arg(&c_path)
        .output()
        .expect("spawn gcc");
    assert!(
        out.status.success(),
        "generated heap C failed to compile:\n{}\n--- header ---\n{}\n--- source ---\n{}",
        String::from_utf8_lossy(&out.stderr),
        pkg.header,
        pkg.source,
    );
}

/// Issue 0345 — the same check for a SERVICE payload. Before 0345 the C service
/// templates had no `is_heap` branches and emitted no `_fini`, so a heap-configured
/// `.srv` field produced the heap struct type with an owned serde body: this test
/// would not have compiled. It also exercises the generated
/// `{request,response}_fini`, which is the piece that makes heap ownership
/// expressible in C at all.
#[test]
#[ignore = "spawns gcc -fsyntax-only"]
fn generated_heap_c_service_compiles_and_exposes_fini() {
    let resolver = CapacityResolver::from_toml_str(
        r#"
        [fields]
        "my_srvs/Blob_Request.data" = { cap = 0, mode = "heap" }
        "my_srvs/Blob_Request.label" = { cap = 0, mode = "heap" }
        "my_srvs/Blob_Response.vals" = { cap = 0, mode = "heap" }
        "#,
    )
    .unwrap();
    let srv =
        parse_service("uint8[] data\nstring label\nint32 seq\n---\nfloat32[] vals\nbool ok\n")
            .unwrap();
    let pkg = generate_c_service_package("my_srvs", "Blob", &srv, "h", &resolver).unwrap();

    // The fini surface must exist for both payloads — heap without it is a leak.
    for sym in [
        "my_srvs_srv_blob_request_fini",
        "my_srvs_srv_blob_response_fini",
    ] {
        assert!(
            pkg.header.contains(sym),
            "header must declare {sym}:\n{}",
            pkg.header
        );
        assert!(pkg.source.contains(sym), "source must define {sym}");
    }
    assert!(
        pkg.source.contains("nros_platform_free(msg->data.data)"),
        "request fini must free the heap sequence:\n{}",
        pkg.source
    );

    let tmp = tempfile::tempdir().unwrap();
    let nros = tmp.path().join("nros");
    fs::create_dir_all(&nros).unwrap();
    fs::write(nros.join("types.h"), TYPES_H).unwrap();
    fs::write(nros.join("cdr.h"), CDR_H).unwrap();
    fs::write(nros.join("platform.h"), PLATFORM_H).unwrap();
    fs::write(tmp.path().join(&pkg.header_name), &pkg.header).unwrap();
    let c_path = tmp.path().join(&pkg.source_name);
    fs::write(&c_path, &pkg.source).unwrap();

    let out = Command::new("gcc")
        .args(["-fsyntax-only", "-std=c11", "-Wall", "-Wextra", "-Werror"])
        .arg("-I")
        .arg(tmp.path())
        .arg(&c_path)
        .output()
        .expect("spawn gcc");
    assert!(
        out.status.success(),
        "generated heap C service failed to compile:\n{}\n--- header ---\n{}\n--- source ---\n{}",
        String::from_utf8_lossy(&out.stderr),
        pkg.header,
        pkg.source,
    );
}

/// Issue 0346 — a borrowed SERVICE payload must compile: the view aliases the raw
/// callback buffer, so there is no malloc and no `_fini` for it, while the OWNED
/// struct stays for the publish path.
#[test]
#[ignore = "spawns gcc -fsyntax-only"]
fn generated_borrowed_c_service_compiles() {
    let resolver = CapacityResolver::from_toml_str(
        r#"
        [fields]
        "my_srvs/Peek_Request.name" = { cap = 0, mode = "view" }
        "#,
    )
    .unwrap();
    let srv = parse_service("string name\nint32 seq\n---\nbool ok\n").unwrap();
    let pkg = generate_c_service_package("my_srvs", "Peek", &srv, "h", &resolver).unwrap();

    assert!(
        pkg.header.contains("my_srvs_srv_peek_request_View"),
        "a borrowed view must be emitted:\n{}",
        pkg.header
    );
    assert!(
        pkg.header
            .contains("my_srvs_srv_peek_request_deserialize_view"),
        "the borrowed deserializer must be declared:\n{}",
        pkg.header
    );

    let tmp = tempfile::tempdir().unwrap();
    let nros = tmp.path().join("nros");
    fs::create_dir_all(&nros).unwrap();
    fs::write(nros.join("types.h"), TYPES_H).unwrap();
    fs::write(nros.join("cdr.h"), CDR_H).unwrap();
    fs::write(nros.join("platform.h"), PLATFORM_H).unwrap();
    // The pack emits `#include <nros/view.h>` (issue 0346 renamed the header
    // `borrowed.h` -> `view.h`); this stub kept the old NAME, so the test had
    // been failing on a missing include ever since — invisibly, because it is
    // `#[ignore]`d and no lane runs `--ignored` (issue 0328, the same blind
    // spot the DHEADER note above records).
    fs::write(nros.join("view.h"), BORROWED_H).unwrap();
    fs::write(tmp.path().join(&pkg.header_name), &pkg.header).unwrap();
    let c_path = tmp.path().join(&pkg.source_name);
    fs::write(&c_path, &pkg.source).unwrap();

    let out = Command::new("gcc")
        .args(["-fsyntax-only", "-std=c11", "-Wall", "-Wextra", "-Werror"])
        .arg("-I")
        .arg(tmp.path())
        .arg(&c_path)
        .output()
        .expect("spawn gcc");
    assert!(
        out.status.success(),
        "generated borrowed C service failed to compile:\n{}\n--- header ---\n{}\n--- source ---\n{}",
        String::from_utf8_lossy(&out.stderr),
        pkg.header,
        pkg.source,
    );
}
