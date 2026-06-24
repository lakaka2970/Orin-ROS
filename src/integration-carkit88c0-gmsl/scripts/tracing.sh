BASE="/sys/kernel/debug"
tracing_base="${BASE}/tracing"

    echo 1 > "${tracing_base}/tracing_on"
    echo 30720 > "${tracing_base}/buffer_size_kb"
    echo 1 > "${tracing_base}/events/tegra_rtcpu/enable"
    echo 1 > "${tracing_base}/events/freertos/enable"
    echo 2 > "${BASE}/camrtc/log-level"
    echo 1 > "${tracing_base}/events/camera_common/enable"
    echo > "${tracing_base}/trace"

    echo "Run cat ${tracing_base}/trace_pipe to show the trace"

sudo less +F /sys/kernel/debug/tracing/trace_pipe
