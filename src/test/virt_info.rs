#[derive(Clone)]
pub struct VirtInfo {
    pub world_img: image::DynamicImage,
    pub radstand: f32,
    pub achsenlänge: f32,
    pub color_sensor_distance_from_rotation_point: f32,
    /// (fwd, right), rot
    pub distance_sensor_pos: ((f32, f32), f32),
    pub color_sensor_radius: u32,
    pub max_speed: f32,
}
