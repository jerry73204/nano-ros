// phase-428 W5 finding 9 — a failed `rclcpp::Node::create_*` must not read as
// success.
//
// RUNTIME probe, not a syntax one: the whole defect was that nothing happened.
// The seven `create_*` verbs wrote `(void)node_.create_…(…)` and returned the
// `shared_ptr` regardless, so a failed create handed back a valid-looking
// pointer to a dead entity — and `rclcpp::spin(node)` then returned
// immediately on the uninitialised node, so `init -> create -> spin ->
// shutdown` ran to completion and exited 0.
//
// All seven now funnel through `rclcpp::detail::require_created`, which is
// what this exercises. Three properties, and the third is the one a
// success/failure exit code alone would miss:
//
//   1. a create that SUCCEEDED returns normally (the path every working
//      ported node is on);
//   2. a create that FAILED does not return — the process dies on SIGABRT;
//   3. it dies LOUDLY. RFC-0089 Part I's requirement is not "stop", it is
//      "say so", so the child's stderr is captured and checked for the
//      migration text. A silent abort would satisfy (2) and still be the
//      wrong answer.
//
// Forked, because a test that asserts a process dies cannot be that process.
//
// Why abort and not a null `shared_ptr` or an `ok()` flag: see the doc comment
// on `require_created` in `nros/nros.hpp`, which weighs all three against
// RFC-0089 Part I. Short version: upstream throws, we have no exceptions, and
// of the options that remain only this one is loud for `create_wall_timer`,
// whose returned pointer a ported node stores and never dereferences.

#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <nros/nros.hpp>

// Links no nano-ros archive: `require_created` is a header-only inline over
// `nros::Result` and touches no FFI symbol. The recipe passes
// `--unresolved-symbols=ignore-all` for the issue-0360 variant anchors the
// config headers plant — see the rationale beside the compile line in
// `just/check.just`.

namespace {

struct Outcome {
    int status;         // raw waitpid status
    char stderr_[4096]; // what the child wrote to fd 2, NUL-terminated
};

// Run `body` in a child process with its stderr piped back.
template <typename F> Outcome run_forked(F body) {
    Outcome out;
    out.status = 0;
    out.stderr_[0] = '\0';

    int fds[2];
    if (::pipe(fds) != 0) {
        ::std::fprintf(stderr, "failed_create_aborts: pipe failed\n");
        ::std::abort();
    }
    ::fflush(nullptr);
    const pid_t pid = ::fork();
    if (pid == 0) {
        ::close(fds[0]);
        ::dup2(fds[1], 2);
        ::close(fds[1]);
        body();
        // Reached only when `body` did NOT abort. A distinctive code so the
        // parent can tell "returned normally" from "died some other way".
        ::_exit(17);
    }
    ::close(fds[1]);

    ::size_t used = 0;
    for (;;) {
        const ::ssize_t n = ::read(fds[0], out.stderr_ + used, sizeof(out.stderr_) - 1 - used);
        if (n <= 0) break;
        used += static_cast<::size_t>(n);
        if (used >= sizeof(out.stderr_) - 1) break;
    }
    out.stderr_[used] = '\0';
    ::close(fds[0]);

    if (::waitpid(pid, &out.status, 0) != pid) {
        ::std::fprintf(stderr, "failed_create_aborts: waitpid failed\n");
        ::std::abort();
    }
    return out;
}

void expect(bool cond, const char* what) {
    if (!cond) {
        ::std::fprintf(stderr, "failed_create_aborts: FAILED: %s\n", what);
        ::std::abort();
    }
}

void expect_aborts_loudly(const Outcome& o, const char* verb, const char* what) {
    expect(WIFSIGNALED(o.status) && WTERMSIG(o.status) == SIGABRT, what);
    expect(::std::strstr(o.stderr_, verb) != nullptr,
           "the diagnostic must name the verb that failed");
    expect(::std::strstr(o.stderr_, "node.create_publisher(pub, topic, qos)") != nullptr,
           "the diagnostic must name the Result-returning API to use instead");
}

} // namespace

int main() {
    // (1) A create that SUCCEEDED must be transparent.
    {
        const Outcome o = run_forked([] {
            ::rclcpp::detail::require_created(::nros::Result::success(), "create_publisher",
                                              "chatter");
        });
        expect(WIFEXITED(o.status) && WEXITSTATUS(o.status) == 17,
               "a successful create must return normally, not abort");
        expect(o.stderr_[0] == '\0', "a successful create must say nothing");
    }

    // (2) + (3) A create that FAILED must not return, and must say why.
    // Pre-fix, the seven call sites discarded exactly this value and carried on.
    {
        const Outcome o = run_forked([] {
            ::rclcpp::detail::require_created(::nros::Result(::nros::ErrorCode::NotInitialized),
                                              "create_publisher", "chatter");
        });
        expect_aborts_loudly(o, "create_publisher",
                             "a failed create must abort, never return a handle to a dead entity");
    }

    // Same for a timer: the case a null return could never have covered,
    // because a ported node stores the handle and never dereferences it.
    {
        const Outcome o = run_forked([] {
            ::rclcpp::detail::require_created(::nros::Result(::nros::ErrorCode::TransportError),
                                              "create_wall_timer", "");
        });
        expect_aborts_loudly(o, "create_wall_timer",
                             "a failed create_wall_timer must abort rather than never fire");
    }

    ::std::printf("failed_create_aborts: OK\n");
    return 0;
}
