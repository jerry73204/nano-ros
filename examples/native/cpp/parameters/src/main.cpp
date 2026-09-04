/// @file main.cpp
/// @brief C++ parameters example — exercises nros::ParameterServer.
///
/// Declares bool / int64 / double / string parameters plus a
/// fixed-capacity `nros::Seq<double, N>` sequence parameter (Phase
/// 242.3), reads them back, updates values, exercises the bounds checks,
/// and prints the results. Used by the `cpp_parameters` integration test
/// (Phase 117.9 / 242.3).

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <nros/parameter.hpp>
#include <nros/result.hpp>

namespace {

int run() {
    nros::ParameterServer<8> params;

    if (!params.declare_parameter<bool>("use_sim_time", true).ok()) {
        std::fprintf(stderr, "declare bool failed\n");
        return 1;
    }
    if (!params.declare_parameter<int64_t>("max_iters", 100).ok()) {
        std::fprintf(stderr, "declare int failed\n");
        return 1;
    }
    if (!params.declare_parameter<double>("ctrl_period", 0.15).ok()) {
        std::fprintf(stderr, "declare double failed\n");
        return 1;
    }
    if (!params.declare_parameter<const char*>("frame_id", "base_link").ok()) {
        std::fprintf(stderr, "declare string failed\n");
        return 1;
    }

    if (params.parameter_count() != 4) {
        std::fprintf(stderr, "expected 4 params, got %zu\n", params.parameter_count());
        return 1;
    }

    bool use_sim_time = false;
    int64_t max_iters = 0;
    double ctrl_period = 0.0;
    char frame_id[64] = {0};

    if (!params.get_parameter<bool>("use_sim_time", use_sim_time).ok()) return 2;
    if (!params.get_parameter<int64_t>("max_iters", max_iters).ok()) return 2;
    if (!params.get_parameter<double>("ctrl_period", ctrl_period).ok()) return 2;
    if (!params.get_parameter("frame_id", frame_id, sizeof(frame_id)).ok()) return 2;

    if (use_sim_time != true) return 3;
    if (max_iters != 100) return 3;
    if (ctrl_period < 0.149 || ctrl_period > 0.151) return 3;
    if (std::strcmp(frame_id, "base_link") != 0) return 3;

    if (!params.set_parameter<double>("ctrl_period", 0.05).ok()) return 4;
    if (!params.get_parameter<double>("ctrl_period", ctrl_period).ok()) return 4;
    if (ctrl_period < 0.049 || ctrl_period > 0.051) return 4;

    if (!params.set_parameter<const char*>("frame_id", "map").ok()) return 4;
    if (!params.get_parameter("frame_id", frame_id, sizeof(frame_id)).ok()) return 4;
    if (std::strcmp(frame_id, "map") != 0) return 4;

    if (params.has_parameter("missing")) return 5;

    bool tmp = false;
    rclcpp::Result missing = params.get_parameter<bool>("missing", tmp);
    if (missing.ok()) return 5;

    // ---- Sequence parameters (Phase 242.3) ----

    // Declare a fixed-capacity double sequence (MPC weight-matrix shape).
    if (!params.declare_parameter("mpc_weights", nros::Seq<double, 8>{1.5, 2.5, 3.5}).ok()) {
        std::fprintf(stderr, "declare seq failed\n");
        return 6;
    }
    if (!params.has_parameter("mpc_weights")) return 6;
    if (params.parameter_count() != 5) {
        std::fprintf(stderr, "expected 5 params, got %zu\n", params.parameter_count());
        return 6;
    }

    nros::Seq<double, 8> weights;
    if (!params.get_parameter("mpc_weights", weights).ok()) return 7;
    if (weights.size() != 3) return 7;
    if (weights[0] != 1.5 || weights[1] != 2.5 || weights[2] != 3.5) return 7;

    // Bounds: getting into a too-small Seq must be rejected, not UB.
    nros::Seq<double, 2> too_small;
    if (params.get_parameter("mpc_weights", too_small).ok()) return 7;

    // Update the sequence value (server owns the new bytes).
    if (!params.set_parameter("mpc_weights", nros::Seq<double, 8>{4.0, 5.0, 6.0, 7.0}).ok())
        return 8;
    if (!params.get_parameter("mpc_weights", weights).ok()) return 8;
    if (weights.size() != 4 || weights[3] != 7.0) return 8;

    // Bounds: a value exceeding the declared capacity must be rejected.
    if (!params.declare_parameter("small_seq", nros::Seq<double, 3>{0.1}).ok()) return 8;
    if (params.set_parameter("small_seq", nros::Seq<double, 8>{1, 2, 3, 4, 5}).ok()) return 8;

    // Seq<T,N> itself is bounded: over-capacity push_back is a no-op.
    nros::Seq<double, 2> bounded;
    if (!bounded.push_back(1.0) || !bounded.push_back(2.0)) return 8;
    if (bounded.push_back(3.0)) return 8; // rejected, not UB
    if (bounded.size() != 2) return 8;

    std::printf(
        "OK use_sim_time=%d max_iters=%lld ctrl_period=%f frame_id=%s mpc_weights[0]=%f n=%zu\n",
        static_cast<int>(use_sim_time), static_cast<long long>(max_iters), ctrl_period, frame_id,
        weights[0], weights.size());
    return 0;
}

} // namespace

int main() {
    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
#endif
    return run();
}
