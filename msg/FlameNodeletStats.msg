# This message contains statistics from FlameNodelet.
std_msgs/Header header

int32 img_id
float64 timestamp

# Timing stats
int32 queue_size # Number of items in input queue.
float32 fps # Frames-per-second throughout.
float32 fps_max # Theoretical FPS based on main_ms.
float32 main_ms # Time to execute one iteration of main loop.
float32 waiting_ms # Time waiting for input.
float32 process_frame_ms # Time to process a frame.
float32 publishing_ms # Time to publish output.
float32 debug_publishing_ms # Time to publish debug output.
float32 latency_ms # Difference between getting input and publishing.

# Max possible load.
float32 max_load_cpu # Number of possible concurrent threads.
float32 max_load_mem # Amount of memory in MB.
float32 max_load_swap # Amount of available SWAP.

# System load.
float32 sys_load_cpu # CPU load on the system.
float32 sys_load_mem # Memory load on the system.
float32 sys_load_swap # SWAP load on the system.

# Process load.
int32 pid # Process ID.
float32 pid_load_cpu # CPU load for this process.
float32 pid_load_mem # Memory load for this process.
float32 pid_load_swap # Swap load for this process.