/* phase-432 W3.1 prerequisite — the board-side STRONG override of
 * `nros_board_network_wait()`, the affordance the weak default exists for.
 *
 * A board or Entry app that must block until the link / DHCP lease is ready
 * (e.g. ASI's `configure_network()` prologue) defines this symbol strongly and
 * the linker prefers it over the weak body in `<nros/main.h>`. Linked beside
 * `entry_seam_c_linkage.c`, the marker below MUST appear — the `check-c` lane
 * greps for it, so a change that made the weak body unoverridable (or that
 * turned it into a strong definition) fails there rather than in an
 * out-of-tree consumer's image.
 *
 * Deliberately does NOT include `<nros/main.h>`: an overriding TU is a board
 * file, not an entry, and must not have to pull the entry header to take the
 * hook. */

#include <stdio.h>

void nros_board_network_wait(void);

void nros_board_network_wait(void) { printf("STRONG-OVERRIDE-RAN\n"); }
