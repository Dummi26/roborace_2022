use std::sync::mpsc::{Sender, Receiver};

pub struct VirtualRobot {
    recv: Receiver<VirtualRequest>,
    virt_info: super::virt_info::VirtInfo,
    robot_pos: (f32, f32),
    robot_dir: f32,
    /// this is the angle of the front wheels. the robot must drive robot_radstand forward to turn this angle.
    robot_turn_speed: f32,
    robot_speed: f32,
}

impl VirtualRobot {
    pub fn new(info: super::virt_info::VirtInfo) -> (std::thread::JoinHandle<()>, Sender<VirtualRequest>) {
        let (sender, recv) = std::sync::mpsc::channel();
        (Self {
            recv,
            virt_info: info,
            robot_pos: (0.02, 0.665 /*0.856*/),
            robot_dir: std::f32::consts::PI * 0.5,
            robot_turn_speed: 0.0,
            robot_speed: 0.0,
        }.run_thread(), sender)
    }

    pub fn run_thread(mut self) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            let sleep_duration = std::time::Duration::from_secs_f64(0.01);
            loop {
                while let Ok(recv) = self.recv.try_recv() {
                    match recv {
                        VirtualRequest::ColorSensorBrightness(rp) => {
                            let pixels = self.virt_info.world_img.as_rgba8().unwrap();
                            let mut brightness = 0.0;
                            let mut count = 0;
                            let color_sensor_pos = self.get_robot_pos_color_sensor();
                            let x = (self.virt_info.world_img.width() as f32 * color_sensor_pos.0) as u32;
                            let y = (self.virt_info.world_img.height() as f32 * color_sensor_pos.1) as u32;
                            for x in x.saturating_sub(self.virt_info.color_sensor_radius) .. (x+1+self.virt_info.color_sensor_radius).min(self.virt_info.world_img.width()) {
                                for y in y.saturating_sub(self.virt_info.color_sensor_radius) .. (y+1+self.virt_info.color_sensor_radius).min(self.virt_info.world_img.height()) {
                                    let px = &pixels.get_pixel(x, y).0;
                                    brightness += (255 - px[2]) as f64 / 255.0;
                                    count += 1;
                                }
                            }
                            rp.send(((brightness / count as f64) * 80.0) as i32).unwrap();
                        }
                        VirtualRequest::GetDistance(rp) => {
                            let pixels = self.virt_info.world_img.as_rgba8().unwrap();
                            let sensor_dir = self.robot_dir + self.virt_info.distance_sensor_pos.1;
                            let sensor_dir = (sensor_dir.sin(), -sensor_dir.cos());
                            let x = self.robot_pos.0 + self.virt_info.distance_sensor_pos.0.0 * self.robot_dir.sin();
                            let y = self.robot_pos.1 + self.virt_info.distance_sensor_pos.0.1 * self.robot_dir.cos();
                            let mut dist = 0.0;
                            loop {
                                if dist > 2.0 { dist = 2.0; break; };
                                // NOTE: Factor 0.1 because world is 10m⨯10m
                                let pos = (x + sensor_dir.0 * dist * 0.1, y + sensor_dir.1 * dist * 0.1);
                                let posa = ((pos.0 * pixels.width() as f32), (pos.1 * pixels.height() as f32));
                                if posa.0 < 0.0 || posa.1 < 0.0 || posa.0 as u32 >= pixels.width() || posa.1 as u32 >= pixels.height() { break; }
                                let px = pixels.get_pixel(posa.0 as u32, posa.1 as u32);
                                if px.0[0] > 150 && px.0[1] < 150 && px.0[2] < 150 {
                                    break;
                                }
                                dist += 0.01;
                            }
                            rp.send(dist * 100.0 /* m to cm */).unwrap();
                        },
                        VirtualRequest::SetTurningAngleDeg(ang) => self.robot_turn_speed = ang * std::f32::consts::PI / 180.0,
                        VirtualRequest::SetSpeed(speed) => self.robot_speed = self.virt_info.max_speed * speed as f32 / 100.0,
                        VirtualRequest::VirtGetPos(rp) => {
                            rp.send((self.robot_pos, self.robot_dir, self.robot_turn_speed)).unwrap();
                        }
                    }
                }
                self.robot_dir += self.robot_turn_speed * 25.0 * self.robot_speed;
                self.robot_pos.0 += self.robot_dir.sin() * self.robot_speed;
                self.robot_pos.1 -= self.robot_dir.cos() * self.robot_speed;
                std::thread::sleep(sleep_duration);
            }
        })
    }

    pub fn get_robot_pos_color_sensor(&self) -> (f32, f32) {
        let offset = Self::general_robot_pos_color_sensor(self.robot_dir, self.robot_turn_speed, &self.virt_info);
        (
            self.robot_pos.0 + offset.0,
            self.robot_pos.1 + offset.1
        )
    }
    pub fn general_robot_pos_color_sensor(rot: f32, steering: f32, virt_info: &super::virt_info::VirtInfo) -> (f32, f32) {
        (rot.sin() * virt_info.radstand + (rot + steering).sin() * virt_info.color_sensor_distance_from_rotation_point,
        - rot.cos() * virt_info.radstand - (rot + steering).cos() * virt_info.color_sensor_distance_from_rotation_point)
    }
    pub fn general_robot_pos_front_axle_center(rot: f32, steering: f32, virt_info: &super::virt_info::VirtInfo) -> (f32, f32) {
        (rot.sin() * virt_info.radstand ,
        - rot.cos() * virt_info.radstand)
    }
}

pub enum VirtualRequest {
    ColorSensorBrightness(Sender<i32>),
    /// In cm
    GetDistance(Sender<f32>),
    SetTurningAngleDeg(f32),
    SetSpeed(i32),
    /// ((x, y), angle, steering)
    VirtGetPos(Sender<((f32, f32), f32, f32)>),
}
