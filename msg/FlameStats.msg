# This message contains statistics from Flame.
std_msgs/Header header

int32 img_id
float64 timestamp

int32 num_feats
int32 num_vtx
int32 num_tris
int32 num_edges
float32 coverage # Fraction of frame with depths.

# Depth estimation/tracking stats.
int32 num_idepth_updates
int32 num_fail_max_var
int32 num_fail_max_dropouts
int32 num_fail_ref_patch_grad
int32 num_fail_ambiguous_match
int32 num_fail_max_cost

# Regularizer stats.
float32 nltgv2_total_smoothness_cost # Total cost.
float32 nltgv2_avg_smoothness_cost # Cost per vertex.
float32 nltgv2_total_data_cost
float32 nltgv2_avg_data_cost

float32 total_photo_error # Photo error over all valid pixels.
float32 avg_photo_error # Photo error/pixel.

# Timing stats
float32 fps # Frames-per-second throughout
float32 fps_max # Theoretical FPS based on update time.
float32 update_ms # Main update function runtime.
float32 update_locking_ms # Time waiting for update_lock.
float32 frame_creation_ms
float32 interpolate_ms
float32 keyframe_ms
float32 detection_ms
float32 detection_loop_ms
float32 update_idepths_ms
float32 project_features_ms
float32 project_graph_ms
float32 sync_graph_ms
float32 triangulate_ms
float32 median_filter_ms
float32 lowpass_filter_ms


