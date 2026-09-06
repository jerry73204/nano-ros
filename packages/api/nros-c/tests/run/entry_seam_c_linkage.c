/* phase-432 W3.1 prerequisite — a PURE C entry must be able to link the
 * embedded board seam.
 *
 * `nros_board_network_wait()` is the network-readiness hook the three RTOS
 * `run_tiers.c` files call `extern`. Its weak default used to live in
 * `<nros/main.hpp>`, a C++ header, so it reached an image only through the
 * generated `.cpp` entry. A pure C entry COMPILED and then failed at LINK
 * with `undefined reference to nros_board_network_wait` — which is why this
 * probe LINKS rather than only compiling. `-fsyntax-only` is green either way,
 * and so is the weak-symbol allowlist gate: that one checks WHERE the
 * attribute lives, never whether C can reach the symbol.
 *
 * This TU includes ONLY C headers on purpose. Do not add a C++ header, and do
 * not link the nros-c archive: the point is that the seam resolves from the
 * headers a C entry already includes.
 *
 * Built TWICE by the `check-c` lane. Alone, the weak no-op must run and the
 * program must exit 0. Linked with the sibling `entry_seam_c_override.c`, the
 * STRONG definition must win and its marker must appear on stdout — that
 * override is the reason the hook is weak at all, and nothing else in this
 * tree exercises it (measured: zero strong definitions across packages/,
 * examples/, third-party/; the real consumers are out of tree).
 */

#include <nros/main.h>

#include <stdio.h>

int main(void) {
    nros_board_network_wait();
    printf("entry_seam_c_linkage: seam resolved from C headers alone\n");
    return 0;
}
