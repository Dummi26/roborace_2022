use std::{time::{Instant, Duration}, collections::HashMap, ops::Sub};

use speedy2d::{window::{WindowHandler, VirtualKeyCode}, color::Color, dimen::Vector2, image::{ImageDataType, ImageSmoothingMode}, shape::Rectangle};

use crate::test::virtual_robot::VirtualRequest;

pub fn main(virtual_robot_sender: std::sync::mpsc::Sender<crate::test::virtual_robot::VirtualRequest>, info: super::virt_info::VirtInfo) {
    match speedy2d::Window::new_centered("ev3 pc testing playground", (1280, 720)) {
        Ok(window) => {
            let mut window_handler = EditorWindowHandler {
                size: (0, 0),
                resized: false,
                mouse_pos: (0.0, 0.0),
                keyboard_modifiers_state: speedy2d::window::ModifiersState::default(),
                virtual_robot_sender,
                virt_info: info,
            };
            window.run_loop(window_handler);
        },
        Err(err) => panic!("Error creating window with speedy2d: {:?}", err),
    }
}

struct EditorWindowHandler {
    size: (u32, u32),
    resized: bool,
    mouse_pos: (f32, f32),
    keyboard_modifiers_state: speedy2d::window::ModifiersState,
    virtual_robot_sender: std::sync::mpsc::Sender<crate::test::virtual_robot::VirtualRequest>,
    virt_info: super::virt_info::VirtInfo,
}

impl WindowHandler for EditorWindowHandler {
    fn on_start(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        info: speedy2d::window::WindowStartupInfo
    )
    {
    }

    fn on_user_event(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        user_event: ()
    )
    {
    }

    fn on_resize(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        size_pixels: speedy2d::dimen::Vector2<u32>
    )
    {
        self.size = (size_pixels.x, size_pixels.y);
        self.resized = true;
    }

    fn on_mouse_grab_status_changed(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        mouse_grabbed: bool
    )
    {
    }

    fn on_fullscreen_status_changed(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        fullscreen: bool
    )
    {
    }

    fn on_scale_factor_changed(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        scale_factor: f64
    )
    {
    }

    fn on_draw(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        graphics: &mut speedy2d::Graphics2D
    )
    {
        // CLEAR
        graphics.clear_screen(Color::BLACK);

        // DRAW WORLD IMAGE
        let handle = graphics.create_image_from_raw_pixels(ImageDataType::RGBA, ImageSmoothingMode::NearestNeighbor, Vector2::new(self.virt_info.world_img.width(), self.virt_info.world_img.height()), self.virt_info.world_img.as_bytes()).unwrap();
        graphics.draw_rectangle_image_tinted(Rectangle::new(Vector2::new(0.0, 0.0), Vector2::new(self.size.0 as f32, self.size.1 as f32)), Color::from_rgba(1.0, 1.0, 1.0, 1.0), &handle);

        // DRAW VEHICLE POSITION AND ROTATION
        let (pos, rot, steering) = {
            let ch = std::sync::mpsc::channel();
            self.virtual_robot_sender.send(VirtualRequest::VirtGetPos(ch.0));
            ch.1.recv().unwrap()
        };
        let color_sensor_offset = super::virtual_robot::VirtualRobot::general_robot_pos_color_sensor(rot, steering, &self.virt_info);
        let front_axle_center_offset = super::virtual_robot::VirtualRobot::general_robot_pos_front_axle_center(rot, steering, &self.virt_info);
        let posv_color_sensor = Vector2 { x: self.size.0 as f32 * (pos.0 + color_sensor_offset.0), y: self.size.1 as f32 * (pos.1 + color_sensor_offset.1) };
        let posv_front_axle_center = Vector2 { x: self.size.0 as f32 * (pos.0 + front_axle_center_offset.0), y: self.size.1 as f32 * (pos.1 + front_axle_center_offset.1) };
        let posv_back_axle_center = Vector2 { x: self.size.0 as f32 * pos.0, y: self.size.1 as f32 * pos.1 };
        graphics.draw_line(
            posv_back_axle_center,
            posv_front_axle_center,
            3.0, Color::from_rgba(0.2, 0.2, 1.0, 1.0)
        );
        graphics.draw_line(
            posv_front_axle_center,
            posv_color_sensor,
            1.0, Color::from_rgba(1.0, 0.2, 0.2, 1.0)
        );
        // REDRAW
        helper.request_redraw();
    }

    fn on_mouse_move(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        position: speedy2d::dimen::Vector2<f32>
    )
    {
        self.mouse_pos = (position.x / self.size.0 as f32, position.y / self.size.1 as f32);
    }

    fn on_mouse_button_down(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        button: speedy2d::window::MouseButton
    )
    {
    }

    fn on_mouse_button_up(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        button: speedy2d::window::MouseButton
    )
    {
    }

    fn on_mouse_wheel_scroll(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        distance: speedy2d::window::MouseScrollDistance
    )
    {
    }

    fn on_key_down(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        virtual_key_code: Option<speedy2d::window::VirtualKeyCode>,
        scancode: speedy2d::window::KeyScancode
    )
    {
    }

    fn on_key_up(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        virtual_key_code: Option<speedy2d::window::VirtualKeyCode>,
        scancode: speedy2d::window::KeyScancode
    )
    {
    }

    fn on_keyboard_char(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        unicode_codepoint: char
    )
    {
    }

    fn on_keyboard_modifiers_changed(
        &mut self,
        helper: &mut speedy2d::window::WindowHelper<()>,
        state: speedy2d::window::ModifiersState
    )
    {
        self.keyboard_modifiers_state = state;
    }
}
